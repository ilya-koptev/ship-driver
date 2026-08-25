#!/usr/bin/env python3
# ship-driver.py — custom multi-channel ship-bus driver.
# 4 equivalent LoRa transmitters (MOD1-4 = /dev/ttyMOD1..4). Each channel independently owns its
# port and runs the SAME ship logic against one active ship (Modbus addrs identical on every ship:
# UPS=10, pwm=11/12/13; ships are told apart by LoRa address -> change the channel modem address to
# switch ship). Per channel: Modbus RTU + inline DFPlayer + mode state machine + poll schedule.
# Charge thermal protection (charge blocked >+50 °C) is done by WB-UPS-v3 itself; driver only sets
# full charge current. Tunables live in /etc/ship-driver.conf (defaults below if absent).
# boatN MQTT device = operational API + visualisation (driven by external software); LoRa config
# lives in the conf, only LoRa_address is live on boatN (writes the modem immediately, applying the
# conf channel/air/power + the new address). Wired ship pre-config stays on the "Ship Setup" dashboard.
# shipN (N from ships.list) mirrors the physics of whichever channel currently serves that ship, so
# history is kept per SHIP, not per radio point; radio metrics stay on boatN. boatN is unchanged —
# it is the external API. NOTE: every device must be listed in /etc/mosquitto/acl/ship.conf, or the
# broker drops our connection on the first publish (MQTT 3.1.1 has no way to report a denied publish).
import time, threading, queue, json, os, re, signal, socket, math
import serial
import paho.mqtt.client as mqtt

# ---- hardware topology: per-board profile {channel -> (tty, config-gpio LINE NAME)}, auto-selected by board model ----
# The config pin is referenced by its gpiod line NAME ("MODn RTS") and resolved to a sysfs number at start —
# portable across boards (WB7/WB8 number the line differently but name it the same).
_MODMAP={"mod1":("/dev/ttyMOD1","MOD1 RTS"),"mod2":("/dev/ttyMOD2","MOD2 RTS"),
         "mod3":("/dev/ttyMOD3","MOD3 RTS"),"mod4":("/dev/ttyMOD4","MOD4 RTS")}
HW_PROFILES={   # WB7 & WB8 name MOD slots identically (/dev/ttyMODn + "MODn RTS"); the by-name gpio resolve
 "wb8":_MODMAP,  # handles the different numbers (WB8: 519/522/271/106, WB7-A40i: 36/84/52/270)
 "wb7":_MODMAP,
}
GPIO_FALLBACK={"MOD1 RTS":519,"MOD2 RTS":522,"MOD3 RTS":271,"MOD4 RTS":106}   # WB8 numbers if debugfs name map is unavailable
def gpio_names():   # gpiod line-name -> global sysfs gpio number, parsed from debugfs
    m={}
    try:
        for ln in open("/sys/kernel/debug/gpio"):
            mt=re.search(r"gpio-(\d+)\s+\(\s*([^|)]+?)\s*[|)]",ln)
            if mt: m[mt.group(2).strip()]=int(mt.group(1))
    except Exception: pass
    return m
GPIO_NAMES=gpio_names()
def resolve_gpio(name):
    if isinstance(name,int): return name
    return GPIO_NAMES.get(name, GPIO_FALLBACK.get(name))
def detect_board():
    try: model=open("/proc/device-tree/model","rb").read().decode("ascii","ignore").strip("\x00\n ").lower()
    except Exception: model=""
    if "board 8" in model: return "wb8"
    if "rev. 7" in model or "rev 7" in model or "a40i" in model or "wb7" in model: return "wb7"   # WB7 (e.g. "Wiren Board rev. 7.4.2 (A40i)")
    return "wb8"   # default
BOARD=detect_board()
_profile=HW_PROFILES.get(BOARD) or HW_PROFILES["wb8"]
if BOARD not in HW_PROFILES:
    print("WARN: no hw profile for board '%s' — using wb8 channel map (config-GPIOs may be wrong)"%BOARD,flush=True)
CHANNELS={}
for _n,(_tty,_g) in _profile.items():
    _num=resolve_gpio(_g)
    if _num is None: print("WARN: config-GPIO line '%s' (%s) not found"%(_g,_n),flush=True)
    CHANNELS[_n]=(_tty,_num)
print("board=%s channels=%s"%(BOARD,{n:(t,g) for n,(t,g) in CHANNELS.items()}),flush=True)
PWM_ADDR_REG=253; PWM_BAUD_REG=254   # PWM8A04: регистр адреса и код скорости (3 = 9600) — установлено 11.08 на живых модулях
PWM_SETUP_CONTROLS=["address","new_address","read","write","found_address","baud_code","freq","duty","status"]
RS485="/dev/ttyRS485-1"   # Ship Setup dashboard: wired ship LoRa-modem config (ship modem in config mode by its switch)
STATE_FILE="/etc/ship-driver-state.json"   # persist per-channel enabled across reboot
CONF_FILE="/etc/ship-driver.conf"          # tunable settings (see DEFAULTS)

PWM_SLAVES=[11,12,13]; FREQ_REG={1:0,2:1,3:2}; DUTY_REG={1:112,2:113,3:114}
ALL_CH=[(s,c) for s in PWM_SLAVES for c in (1,2,3)]
UPS=10; UPS_VIN=2; UPS_CUR=5; UPS_CHG=8; UPS_TEMP=9; UPS_CHG_SETPOINT=18   # UPS_VIN=2: input voltage (x0.001)
# WT901C485 (JY-901): один блок 0x34..0x54 = 33 регистра за ОДНУ транзакцию (~220 мс) —
# ускорения, гироскоп, магнитометр, углы, температура и кватернионы. Смещения внутри блока:
IMU_BASE=0x34; IMU_LEN=33
IMU_SHORT=13          # 0x34..0x40: ускорения, гироскоп, магнитометр, углы, температура — всё, что на дашборде
IMU_Q_BASE=0x51       # кватернионы лежат отдельно, за 16 неиспользуемыми регистрами
IMU_Q_PERIOD=60.0     # они нужны для разбора по логам, а не для дашборда -> берём редко
IMU_ACC=0; IMU_GYR=3; IMU_MAG=6; IMU_ANG=9; IMU_TEMP=12; IMU_Q=29   # 0x51..0x54 -> индекс 29
ACC_SCALE=16.0/32768.0; GYR_SCALE=2000.0/32768.0; ANG_SCALE=180.0/32768.0; Q_SCALE=1.0/32768.0
IMU_PUB=["accel_x","accel_y","accel_z","gyro_x","gyro_y","gyro_z","mag_x","mag_y","mag_z",
         "roll","pitch","course","sensor_temp","q0","q1","q2","q3"]
# motor/light -> (pwm slave, channel) wiring is loaded from the conf "wiring" section below

SEARCH="SEARCH"; SAIL="SAILING"; CHARGE="CHARGING"; IDLE="IDLE"; SERVICE="SERVICE"; OFF="OFF"

# ---- tunable defaults (overridden per-key by /etc/ship-driver.conf) ----
DEFAULTS={
 "main":{
 "baud":9600, "resp_timeout_s":0.8,
 # Ток заряда. Аппаратная защита УПС рвёт заряд при +50 °C, после чего он идёт рывками — поэтому
 # драйвер сам держит температуру чуть ниже порога и при этом выжимает максимально возможный ток.
 "charge":{"full_ma":2000,
           "thermal":{"enabled":True,"t_target_c":48.0,"min_ma":300,
                      "kp_ma_per_c":400.0,"ki_ma_per_c_s":3.0,"lead_min":5.0,"slope_win_s":240,
                      "deadband_ma":40,"min_write_s":20},
           # индикатор посадки катушек: ожидаемое Vin = v_open - r_eff*Ibat; недобор dev_full_v = 0 %
           "link":{"v_open_v":12.7,"r_eff_ohm":0.35,"dev_full_v":2.0,"smooth":5}},
 "init":{"freq":400,"motor":40,"light":0},
 "limits":{"motor_min":40,"motor_max":80,"mp3_track_max":15},
 "rates":{
   "CHARGING":{"current":5,"temp":10,"charge":20,"pwm_readback":30,"freq_check":10,"course":10},
   "SAILING": {"current":2,"temp":60,"charge":60,"pwm_readback":30,"freq_check":10,"course":1},
   "IDLE":    {"current":5,"temp":60,"charge":60,"pwm_readback":60,"freq_check":10,"course":10},
   "sail_timeout_s":30.0, "offline_fails":2, "read_tries":2, "tx_gap_ms":0,
   "search_period":0.05, "service_period":1.0},
 "enabled_at_start":{"mod1":True,"mod2":True,"mod3":True,"mod4":True},
 },
 "lora":{   # per-MOD channel plan (top level)
   "mod1":{"channel":14,"air_rate":62.5,"address":3,"power":22},
   "mod2":{"channel":16,"air_rate":62.5,"address":23,"power":22},
   "mod3":{"channel":17,"air_rate":62.5,"address":43,"power":22},
   "mod4":{"channel":19,"air_rate":62.5,"address":63,"power":22}},
 "ships":{
   "default":{   # shared air_rate/power + fallback wiring for any address not in the list
     "air_rate":62.5,"power":22,
     "motors":{"back_left":{"slave":12,"channel":1},"front_left":{"slave":12,"channel":2},
               "back_right":{"slave":11,"channel":1},"front_right":{"slave":11,"channel":2}},
     "lights":{"nav_lights":{"slave":11,"channel":3},"morse_lamp":{"slave":12,"channel":3},"deck_lights":{"slave":13,"channel":1},
               "cabin_light1":{"slave":13,"channel":2},"cabin_light2":{"slave":13,"channel":3}}},
   # Борта 1..10. Разводка у элемента необязательна — чего нет, берётся из "default".
   # Для каждого борта из этого списка заводится своё MQTT-устройство shipN (история физики по кораблю, а не по точке).
   "list":[{"address":n} for n in range(1,11)],
 },
 "chargers":[   # wireless charging stations on Modbus-RTU-over-TCP gateways. relay = XKT-801 transmitter + hold magnets (separate channels); MAI = tx current (voltage drop on a shunt). Each charger has its own "gateway".
   {"gateway":"192.168.69.33:8886",
    "transmitter":{"address":87,"channel":2,"invert":True},   # WB-MRM2-mini K2 -> XKT-801 (NC relay: coil 0 = ON, so invert)
    "magnets":{"address":87,"channel":1,"invert":True},       # WB-MRM2-mini K1 -> hold magnets (NC: invert)
    "sensor":{"address":3,"input":1,"shunt_ohm":1.2}}],        # WB-MAI6 IN1, I = V / shunt
}
def deep_merge(base,over):
    for k,v in (over or {}).items():
        if isinstance(v,dict) and isinstance(base.get(k),dict): deep_merge(base[k],v)
        else: base[k]=v
    return base
def load_conf():
    base=json.loads(json.dumps(DEFAULTS))
    try: deep_merge(base,json.load(open(CONF_FILE)))
    except FileNotFoundError: pass
    except Exception as e: print("conf load err (using defaults):",e,flush=True)
    return base
C=load_conf()

M=C["main"]
BAUD=M["baud"]; RESP_TO=M["resp_timeout_s"]
CHG_FULL=M["charge"]["full_ma"]   # верхний предел тока заряда (регистр 18 УПС принимает 300..2000 мА)
_TH=M["charge"].get("thermal",{})
THERM_ON=bool(_TH.get("enabled",True)); T_TARGET=float(_TH.get("t_target_c",48.0))
CHG_MIN=int(_TH.get("min_ma",300)); TH_KP=float(_TH.get("kp_ma_per_c",400.0))
TH_KI=float(_TH.get("ki_ma_per_c_s",3.0)); TH_LEAD=float(_TH.get("lead_min",5.0))
TH_DEAD=int(_TH.get("deadband_ma",40)); TH_WRITE_S=float(_TH.get("min_write_s",20))
TH_SLOPE_WIN=float(_TH.get("slope_win_s",240))   # окно оценки скорости роста температуры (МНК)
_LK=M["charge"].get("link",{})
LK_VOPEN=float(_LK.get("v_open_v",12.7)); LK_REFF=float(_LK.get("r_eff_ohm",0.35))
LK_DEVFULL=float(_LK.get("dev_full_v",2.0)); LK_SMOOTH=int(_LK.get("smooth",5))
INIT_FREQ=M["init"]["freq"]; INIT_MOTOR=M["init"]["motor"]; INIT_LIGHT=M["init"]["light"]
MOTOR_MIN=M["limits"]["motor_min"]; MOTOR_MAX=M["limits"]["motor_max"]
MP3_TRACK_MAX=M["limits"]["mp3_track_max"]; MP3_VOL_MAX=30   # max volume hardcoded
RATES={CHARGE:M["rates"]["CHARGING"], SAIL:M["rates"]["SAILING"], IDLE:M["rates"]["IDLE"]}
SAIL_TIMEOUT=M["rates"]["sail_timeout_s"]; OFFLINE_FAILS=M["rates"]["offline_fails"]
SEARCH_PERIOD=M["rates"]["search_period"]; SERVICE_PERIOD=M["rates"]["service_period"]
SENSOR_GIVEUP=3   # столько неудач подряд -> считаем, что на этом борту датчика нет, и перестаём его дёргать
READ_TRIES=int(M["rates"].get("read_tries",2)); READ_RETRY_GAP=0.04   # 1 повтор по умолчанию; пауза перед повтором, чтобы опоздавший кадр не столкнулся
TX_GAP=max(0.0,float(M["rates"].get("tx_gap_ms",0))/1000.0)   # пауза ПЕРЕД каждой транзакцией: даёт модему домолчать/переключить TX-RX (0 = как было)
COMMS_WIN=300.0   # окно скользящих счётчиков связи, с
LOST_MISSES=int(M["rates"].get("lost_misses",6))   # столько неответов подряд по ОДНОМУ модулю = авария: идти с этим нельзя
DIAG_LOG=True     # писать в журнал строку на каждый промах чтения (тип/slave/RSSI/байты) — для разбора природы ошибок
FREQ_BASE=850.125; SPED_BASE=0x60; OPTION_BASE=0x60   # band base + E220 SPED/OPTION base bytes (UART 9600, subpkt128, RSSI) — fixed
REG5_TXMODE=0x03   # E220 reg 0x05: transparent, LBT OFF, WOR=3 (match working .6 modems; some modules ship with LBT on)
REG_TAIL=[0x00,0x00]   # regs 0x06,0x07 (CRYPT high/low) — .6 reference; written so the full dump matches
RSSI_BYTE=True     # E220 reg 0x05 бит7: модем дописывает 1 байт RSSI после КАЖДОГО принятого кадра.
                   # Включаем ТОЛЬКО на береговых MOD-модемах (в lora_op). Борта (setup_op) — БЕЗ этого бита,
                   # иначе лишний байт побьёт их локальный Modbus. Флаг обязан совпадать с реальным конфигом модема.
LORA_DEFAULT_RAW="xxxx6760xx03000010"   # .6 reference 9-byte dump (x = variable addr/channel; 67/60 SPED/OPTION; 03 reg5; 0000 crypt; 10 version)
LORA_PLAN=C["lora"]   # {mod1..4: {channel,air_rate,address,power}} (top-level)
GRKCH_CHANNELS={14,16,17,19}   # ГКРЧ-allowed LoRa channels
def grkch(ch):
    try: return "✓ in band (GKRCh)" if int(ch) in GRKCH_CHANNELS else "⚠ out of band"
    except Exception: return "?"
ADDR_MAX=65535   # ship_number (= LoRa address) control max (hardcoded)
ENABLED_AT_START=set(n for n,v in M["enabled_at_start"].items() if v)
SENSOR_FALLBACK={"enabled":True,"address":14,"invert":False}
def parse_wiring(w,base=None):   # -> (motors[(name,slave,ch)], motor_map{name:(slave,ch)}, lights[...], sensor{...}) — всё это per-ship
    # base = "ships.default": в элементе списка разводку можно не указывать вовсе (у всех бортов она одинаковая),
    # тогда берётся типовая. Иначе на 10 кораблей страница настроек раздувалась бы в сотню одинаковых полей.
    b=base or {}
    motors=[(n,m["slave"],m["channel"]) for n,m in (w.get("motors") or b.get("motors") or {}).items()]
    lights=[(n,l["slave"],l["channel"]) for n,l in (w.get("lights") or b.get("lights") or {}).items()]
    sen=dict(SENSOR_FALLBACK); sen.update(b.get("sensor") or {}); sen.update(w.get("sensor") or {})   # датчик курса стоит не на каждом борту
    return motors,{n:(s,c) for n,s,c in motors},lights,sen
SD=C["ships"]; SHIP_DEFAULT=SD["default"]; SHIP_LIST=SD.get("list",[])
DEFAULT_WIRING=parse_wiring(SHIP_DEFAULT)                       # fallback wiring (address not in list)
DEFAULT_AIR_RATE=SHIP_DEFAULT["air_rate"]; DEFAULT_POWER=SHIP_DEFAULT["power"]   # shared for all ships
SHIP_WIRING={int(s["address"]):parse_wiring(s,SHIP_DEFAULT) for s in SHIP_LIST}  # LoRa address -> wiring
SHIP_NUMBERS=sorted({int(s["address"]) for s in SHIP_LIST})     # борта из конфига -> у каждого своё устройство shipN
SHIP_NUMSET=set(SHIP_NUMBERS)
# Ship Setup пишет в борт по кабелю: адрес и канал берутся из полей, всё остальное — из ships.default
# (раньше air_rate/power были зашиты в код и могли расходиться с планом в конфиге).
SETUP_DEFAULTS={"channel":14,"address":3,"air_rate":DEFAULT_AIR_RATE,"power":DEFAULT_POWER}
def wiring_for(addr):
    try: return SHIP_WIRING.get(int(addr),DEFAULT_WIRING)
    except Exception: return DEFAULT_WIRING
# ---- charging stations (Modbus-RTU-over-TCP via a serial-gateway) ----
CHG_LIST=C.get("chargers",[])
CHG_PERIOD=3.0            # charger bus poll period, s
MAI_VOLT_SCALE=1e-6       # WB-MAI6 input-voltage raw (s32) -> volts
MAI_IN0=0x0700           # WB-MAI6 fw2.4: IN n voltage input reg = MAI_IN0 + 2*(n-1), s32
MRM_COIL0=0; MRM_STATE0=96   # WB-MRM2-mini: K ch -> coil (ch-1); real contact state -> discrete 96+(ch-1)
CHARGER_CONTROLS=["transmitter","magnets","transmitter_current","charge_link"]
# control set is the SAME on every ship -> names/count fixed (from default), only the register mapping varies per ship
MOTOR_NAMES=[n for n,_,_ in DEFAULT_WIRING[0]]
LIGHT_NAMES=[n for n,_,_ in DEFAULT_WIRING[2]]
NLIGHTS=len(LIGHT_NAMES)
_MT={"front_right":"Front Right","back_right":"Back Right","front_left":"Front Left","back_left":"Back Left"}
MOTOR_TITLE={n:_MT.get(n,n.replace("_"," ").title()) for n in MOTOR_NAMES}
_LT={"nav_lights":"Navigation lights","morse_lamp":"Morse signal lamp","deck_lights":"Deck lights","cabin_light1":"Cabin light 1","cabin_light2":"Cabin light 2"}
LIGHT_TITLE={n:_LT.get(n,n.replace("_"," ").title()) for n in LIGHT_NAMES}   # dashboard titles (nautical, English)
KEEP_ON_RELEASE={"nav_lights"}   # что НЕ гасим, отпуская борт: ходовые огни горят и на брошенном катере
# Телеметрия точки: (control, units, title). Один список на все дашборды — у корабля те же подписи, что у точки.
BOAT_TELE=(("battery_current","A","Battery current"),("battery_temperature","°C","Battery temperature"),("charge_level","%","Charge level"),
           ("battery_voltage","V","Battery voltage"),("input_voltage","V","Input voltage"),("rssi","dBm","LoRa RSSI"),
           ("comms_errors","","Comms errors (5 min)"),("link_quality","%","Link quality"),("link_score","","Link score (0-100)"),
           ("charge_setpoint","mA","Charge current setpoint"),("read_failures","","Read failures (5 min)"),
           ("err_timeout","","Errors: no reply (5 min)"),("err_frame","","Errors: framing/CRC (5 min)"),
           ("retry_fixed","","Fixed by retry (5 min)"),("lat_p95","ms","Read latency p95"),
           ("course","°","Course (yaw)"),("roll","°","Roll"),("pitch","°","Pitch"),
           ("gyro_x","°/s","Gyro X"),("gyro_y","°/s","Gyro Y"),("gyro_z","°/s","Turn rate (gyro Z)"),
           ("accel_x","g","Accel X"),("accel_y","g","Accel Y"),("accel_z","g","Accel Z"),
           ("mag_x","","Mag X"),("mag_y","","Mag Y"),("mag_z","","Mag Z"),("sensor_temp","°C","Sensor temp"),
           ("q0","","Quaternion q0"),("q1","","Quaternion q1"),("q2","","Quaternion q2"),("q3","","Quaternion q3"))
# ---- зеркало по кораблям (shipN) ----
# Борт кочует между радиоточками, поэтому историю физики надо вести ПО КОРАБЛЮ. boat1..4 при этом не меняется —
# это внешний API (сторонний софт, ACL), shipN лишь ДОПОЛНИТЕЛЬНАЯ копия тех же значений.
# Радиометрики (RSSI, счётчики связи, link_score) на корабль НЕ уходят: они про антенну берега и её эфир, а не про борт.
SHIP_RADIO_ONLY={"rssi","comms_errors","link_quality","link_score","read_failures","err_timeout","err_frame","retry_fixed","lat_p95"}
SHIP_TELE=tuple(t for t in BOAT_TELE if t[0] not in SHIP_RADIO_ONLY)
SHIP_CMD=MOTOR_NAMES+LIGHT_NAMES+["mp3_track","mp3_volume"]   # это уходит на точку как есть
SHIP_CTL=["radio_point","active"]   # управление на уровне БОРТА: где стоит и занимает ли точку
SHIP_MIRROR=set(["mode"]+[t[0] for t in SHIP_TELE]+SHIP_CMD)
SHIP_CONTROLS=["radio_point","active","mode"]+[t[0] for t in SHIP_TELE]+SHIP_CMD   # для сноса устройства при остановке
BOAT_CONTROLS=["enabled","mode","battery_current","battery_temperature","charge_level","battery_voltage","input_voltage","rssi","comms_errors","link_quality","link_score","charge_setpoint","read_failures","err_timeout","err_frame","retry_fixed","lat_p95"]+IMU_PUB+MOTOR_NAMES+LIGHT_NAMES+["mp3_track","mp3_volume","ship_number"]
BOAT_EXTRA=[c for c in BOAT_CONTROLS if c not in ("enabled","mode","ship_number")]   # shown only while polling (online); removed in SEARCH/OFF
SETUP_CONTROLS=["ship_number","LoRa_address","LoRa_channel","LoRa_freq","LoRa_grkch","LoRa_air_rate","LoRa_power","LoRa_lbt","LoRa_uart","LoRa_subpacket","LoRa_rssi_ambient","LoRa_rssi_byte","LoRa_mode","LoRa_wor","LoRa_version","LoRa_raw","LoRa_default","LoRa_read","LoRa_apply","LoRa_status"]   # ship_setup dashboard controls (for teardown on shutdown)

MP3={"play":0x08,"vol":0x06,"pause":0x0E,"resume":0x0D,"stop":0x16,"next":0x01,"prev":0x02}
def mp3_frame(cmd,param=0): return bytes([0x7E,0xFF,0x06,cmd,0x00,(param>>8)&0xFF,param&0xFF,0xEF])
AIR_CODE={"2.4":2,"4.8":3,"9.6":4,"19.2":5,"38.4":6,"62.5":7}
AIR_NAME={0:"2.4",1:"2.4",2:"2.4",3:"4.8",4:"9.6",5:"19.2",6:"38.4",7:"62.5"}
PWR_CODE={"22":0,"17":1,"13":2,"10":3}; PWR_NAME={0:"22",1:"17",2:"13",3:"10"}
# E220 register decode tables (per datasheet)
BAUD_NAMES={0:"1200",1:"2400",2:"4800",3:"9600",4:"19200",5:"38400",6:"57600",7:"115200"}
PARITY_NAMES={0:"8N1",1:"8O1",2:"8E1",3:"8N1"}
SUBPKT_NAMES={0:"200",1:"128",2:"64",3:"32"}
WOR_NAMES={0:"500",1:"1000",2:"1500",3:"2000",4:"2500",5:"3000",6:"3500",7:"4000"}
def decode_e220(b):   # b = 9 register bytes (0x00..0x08); 0x08 = version. Returns decoded fields.
    d={}
    d["address"]=(b[0]<<8)|b[1]
    d["uart"]=BAUD_NAMES.get((b[2]>>5)&7,"?")+" "+PARITY_NAMES.get((b[2]>>3)&3,"?")
    d["air_rate"]=AIR_NAME.get(b[2]&7,"?")
    d["subpacket"]=SUBPKT_NAMES.get((b[3]>>6)&3,"?")
    d["rssi_ambient"]="on" if (b[3]>>5)&1 else "off"
    d["power"]=PWR_NAME.get(b[3]&3,"?")
    d["channel"]=b[4]
    d["rssi_byte"]="on" if (b[5]>>7)&1 else "off"
    d["mode"]="fixed" if (b[5]>>6)&1 else "transparent"
    d["lbt"]="on" if (b[5]>>4)&1 else "off"
    d["wor"]=WOR_NAMES.get(b[5]&7,"?")
    d["version"]="0x%02x"%b[8] if len(b)>8 else "?"
    return d

def crc16(d):
    c=0xFFFF
    for b in d:
        c^=b
        for _ in range(8): c=(c>>1)^0xA001 if c&1 else c>>1
    return bytes([c&0xFF,(c>>8)&0xFF])
def s16(v): return v-65536 if v>=32768 else v
def s32(hi,lo): v=(hi<<16)|lo; return v-0x100000000 if v>=0x80000000 else v
def gpio_set(n,v):
    if n is None: return
    base="/sys/class/gpio/gpio%d"%n
    if not os.path.exists(base):
        try: open("/sys/class/gpio/export","w").write(str(n))
        except Exception: pass
    try: open(base+"/direction","w").write("out")
    except Exception: pass
    open(base+"/value","w").write("1" if v else "0")

class Channel(threading.Thread):
    def __init__(self,drv,name,tty,gpio,enabled):
        super().__init__(daemon=True)
        self.drv=drv; self.name=name; self.tty=tty; self.gpio=gpio
        self.dev="boat"+name[-1]; self.enabled=enabled
        self.ser=None; self.q=queue.Queue()
        self.mode=None; self.online=False; self.fails=0   # mode=None so the first set_mode always fires (incl. OFF gpio)
        self.declared_full=False   # whether the full control set is currently published (vs collapsed to enabled+mode)
        self.last_cmd=0.0; self.lora_read=False
        self.mod_miss={s:0 for s in PWM_SLAVES}   # неответы подряд по каждому pwm-модулю
        self.faulted=set()                        # модули, по которым авария объявлена (повторно не дёргаем)
        self.chg_setpoint=CHG_FULL; self.tele={}; self.rssi=None   # RSSI линка (дБм), из хвостового байта каждого ответа борта
        self._th_i=0.0; self._th_win=[]; self._th_w=0.0            # состояние теплового ПИД-регулятора заряда
        self._rd_att=[]; self._rd_miss=[]   # тайминги попыток/промахов чтения за скользящее окно (сырое качество линка)
        self._rd_fail=[]                    # отказы ПОСЛЕ всех ретраев — «настоящие» ошибки, видимые оператору
        self._rd_kind={"to":[],"short":[],"crc":[],"hdr":[]}   # подписи отказа: to=ничего не пришло (похоже на радио), остальные=фрейминг/тайминги
        self._rd_retry_ok=[]                # промахи, вылеченные повтором (подпись тайминга, не фединга)
        self._lat=[]                        # (t, мс) латентность успешных чтений
        # True при старте процесса: своего прошлого состояния мы не знаем (self.motor нулевой), а ESC надо взвести —
        # значит первый выход на связь = полный init_ship. Иначе resume_ship записал бы нули как «желаемый» газ.
        self.force_init=True                # также ставится при осознанной смене корабля
        self.f16=None                       # поддержка Modbus func16 (блочная запись): None=не пробовали, True/False=выяснено
        self._q_at=0.0                                # когда последний раз читали кватернионы
        self.sensor_fails=0; self.sensor_gone=False   # датчик курса стоит не на всех бортах -> после N неудач перестаём опрашивать
        self.motor={n:0 for n in MOTOR_NAMES}; self.light={n:0 for n in LIGHT_NAMES}
        self.due={}
        self.lora=dict(LORA_PLAN[name])   # {channel,air_rate,address,power} from conf; refreshed by reading the modem at start
        self.apply_wiring()               # pick motor/light register map for this ship (by LoRa address)
    def apply_wiring(self):
        self.motors,self.motor_map,self.lights,self.sensor=wiring_for(self.lora["address"])
        self.light_map={n:(s,c) for n,s,c in self.lights}
        self.sensor_fails=0; self.sensor_gone=False   # сменилась разводка/борт -> заново проверяем датчик

    # ---- serial / modbus ----
    def open(self):
        if self.ser is None:
            self.ser=serial.Serial(self.tty,BAUD,8,"N",1,timeout=RESP_TO)
            gpio_set(self.gpio,0)   # TRANSPARENT (relay) mode for normal operation — for ALL channels
    def close(self):
        if self.ser is not None:
            try: self.ser.close()
            except Exception: pass
            self.ser=None
    def _txn(self,req,n):
        if TX_GAP: time.sleep(TX_GAP)          # пауза перед транзакцией (модем домолчал / переключил TX-RX); 0 = выключено
        self.ser.reset_input_buffer(); self.ser.write(req); self.ser.flush()
        end=time.monotonic()+RESP_TO; buf=b""
        while time.monotonic()<end and len(buf)<n:
            c=self.ser.read(n-len(buf))
            if c: buf+=c
        if RSSI_BYTE and len(buf)==n:              # за целым кадром модем дописал 1 байт RSSI (packet-byte)
            extra=self.ser.read(1)                  # он пришёл вплотную к кадру -> уже в буфере, берётся мгновенно
            if len(extra)==1: self.rssi=-(256-extra[0])   # E220: dBm = -(256 - байт)
        return buf   # кадр возвращаем БЕЗ RSSI-байта — read_regs/write_reg не меняются
    def read_regs(self,slave,func,addr,n,tries=None,stats=True):
        # tries=1 и stats=False — для НЕобязательной телеметрии (датчик курса):
        # повтор ей не нужен, а её промахи не должны портить метрики связи борта.
        # Диагностика природы отказов: каждая неудачная попытка классифицируется.
        #   to    — не пришло НИ БАЙТА (пакет потерян целиком) -> подпись радио
        #   short — пришёл обрывок кадра                        -> фрейминг/тайминги
        #   hdr   — чужой slave/func/длина (мусор из прошлого обмена) -> тайминги
        #   crc   — кадр целый, но CRC не сошёлся               -> помеха/искажение
        # Плюс latency успешных чтений и «вылечено повтором» — фединг так быстро не проходит.
        req=bytes([slave,func,(addr>>8)&0xFF,addr&0xFF,(n>>8)&0xFF,n&0xFF]); req+=crc16(req)
        need=5+2*n
        # В SEARCH (борт не на связи) неответ — это НЕ ошибка связи, а «борта здесь нет»:
        # не повторяем (зря занимали бы эфир вдвое) и не засоряем ни счётчики, ни журнал.
        live=self.online
        lim=(tries or READ_TRIES) if live else 1
        for attempt in range(lim):
            if attempt: time.sleep(READ_RETRY_GAP)            # пауза перед повтором (опоздавший по радио кадр не столкнётся)
            t0=time.monotonic()
            r=self._txn(req,need); t=time.monotonic()
            if live and stats: self._rd_att.append(t)         # учёт качества линка: сырая попытка
            if len(r)==0: kind="to"
            elif len(r)<need: kind="short"
            elif r[0]!=slave or r[1]!=func or r[2]!=2*n: kind="hdr"
            elif crc16(r[:3+2*n])!=r[3+2*n:5+2*n]: kind="crc"
            else:
                if stats:
                    self._lat.append((t,(t-t0)*1000.0))
                    if attempt: self._rd_retry_ok.append(t)   # промах вылечен повтором
                return [ (r[3+2*i]<<8)|r[4+2*i] for i in range(n) ]
            if live:                                          # промахи зондирования в SEARCH не считаем и не логируем
                if stats:
                    self._rd_miss.append(t)
                    self._rd_kind[kind].append(t)
                if DIAG_LOG:
                    print("[%s] промах чтения: тип=%s slave=%d reg=%d n=%d попытка=%d/%d ждал=%.0fмс rssi=%s байт=%d %s"
                          %(self.name,kind,slave,addr,n,attempt+1,lim,(t-t0)*1000.0,
                            self.rssi,len(r),r[:12].hex() if r else ""),flush=True)
        if live and stats: self._rd_fail.append(time.monotonic())   # отказ после всех попыток = «настоящая» ошибка
        return None
    def write_regs(self,slave,addr,vals):
        # Modbus func 16 — несколько ПОДРЯД идущих регистров одним кадром (вместо N кадров по func 6).
        # Экономит эфир: оба мотора модуля = 1 транзакция вместо 2, инициализация модуля = 1 вместо 3.
        # Если модуль func 16 не понимает — откатываемся на поштучную запись и больше не пробуем.
        if not vals: return True
        if self.f16 is False: return all(self.write_reg(slave,addr+i,v) for i,v in enumerate(vals))
        n=len(vals)
        req=bytes([slave,16,(addr>>8)&0xFF,addr&0xFF,(n>>8)&0xFF,n&0xFF,2*n])
        for v in vals: req+=bytes([(int(v)>>8)&0xFF,int(v)&0xFF])
        req+=crc16(req)
        r=self._txn(req,8)
        ok=(len(r)>=8 and r[0]==slave and r[1]==16 and crc16(r[:6])==r[6:8])
        if ok:
            if self.f16 is None:
                self.f16=True; print("[%s] func16 (блочная запись) поддерживается"%self.name,flush=True)
            return True
        if self.f16 is None:   # первая проба не удалась — считаем func16 неподдержанным и дальше пишем поштучно
            self.f16=False
            print("[%s] func16 не подтверждён (ответ %s) -> поштучная запись"%(self.name,r[:4].hex() if r else "нет"),flush=True)
            return all(self.write_reg(slave,addr+i,v) for i,v in enumerate(vals))
        return False   # func16 работал раньше -> это транзиентная потеря, значение дожмёт сверка readback
    def write_reg(self,slave,addr,val):
        val&=0xFFFF; req=bytes([slave,6,(addr>>8)&0xFF,addr&0xFF,(val>>8)&0xFF,val&0xFF]); req+=crc16(req)
        r=self._txn(req,8); return len(r)>=8 and r[0]==slave and r[1]==6
    def send_mp3(self,fr):
        self.ser.reset_input_buffer(); self.ser.write(fr); self.ser.flush(); time.sleep(0.25)

    # ---- MQTT helpers ----
    def sdev(self):
        # устройство корабля, который СЕЙЧАС на этой точке; None, если такого номера нет в списке кораблей
        try: n=int(self.lora["address"])
        except Exception: return None
        if n not in SHIP_NUMSET: return None
        # борт мог переехать на другую точку, а его номер остался в адресе этой — тогда писать в его
        # топики не наше дело: зеркалит только та точка, которой борт принадлежит
        if self.drv.channel_for_ship(n) is not self: return None
        return "ship%d"%n
    def pub(self,ctrl,val):
        if self.drv.mqtt is None: return
        self.drv.mqtt.publish("/devices/%s/controls/%s"%(self.dev,ctrl),str(val),retain=True)
        if ctrl in SHIP_MIRROR:   # физику дублируем на вкладку борта -> история пишется по кораблю, а не по радиоточке
            sd=self.sdev()
            if sd: self.drv.mqtt.publish("/devices/%s/controls/%s"%(sd,ctrl),str(val),retain=True)
    def puberr(self,ctrl,err):   # WB convention: /controls/<c>/meta/error = "r" (read error) -> homeui greys/colours it; "" = ok
        if self.drv.mqtt is None: return
        self.drv.mqtt.publish("/devices/%s/controls/%s/meta/error"%(self.dev,ctrl),err,retain=True)
        if ctrl in SHIP_MIRROR:
            sd=self.sdev()
            if sd: self.drv.mqtt.publish("/devices/%s/controls/%s/meta/error"%(sd,ctrl),err,retain=True)

    def wr(self,slave,addr,val,ctrl=None):
        """Запись с проверкой результата. Раньше возврат write_reg игнорировался,
        и команда на молчащий модуль публиковалась как применённая — оператор видел
        газ, которого в железе нет (проверено 24.08: back_right=45 висел минуту при
        полностью мёртвом модуле 11)."""
        for _ in (1,2):
            if self.write_reg(slave,addr,val):
                if ctrl: self.puberr(ctrl,"")
                return True
        print("[%s] ЗАПИСЬ НЕ ПРОШЛА: slave=%d reg=%d val=%d%s"
              %(self.name,slave,addr,val," (%s)"%ctrl if ctrl else ""),flush=True)
        if ctrl: self.puberr(ctrl,"w")
        return False

    # ---- command handling (this thread) ----
    def handle(self,ctrl,val):
        try: fv=float(val)
        except Exception: fv=0.0
        iv=int(fv); is_cmd=True
        if ctrl=="enabled":
            self.enabled=(val in ("1","true","on")); is_cmd=False
            self.pub("enabled",1 if self.enabled else 0)   # точку может выключить и сам драйвер (с вкладки борта) -> значение надо отдать
            if not self.enabled:
                self.release_ship()   # пока связь ещё есть: газ в холостой, свет и звук долой
                self.online=False; self.set_mode(OFF); self.close()
        elif ctrl in self.motor_map:
            s,c=self.motor_map[ctrl]; want=max(MOTOR_MIN,min(MOTOR_MAX,iv))
            if not self.online:
                # раньше команда тут молча исчезала. Отказ теперь ЯВНЫЙ, но желаемое
                # в self.motor НЕ пишем: иначе resume_ship на реконнекте выдал бы этот
                # газ в железо до арминга ESC (рывок), а decide() показал бы SAILING.
                is_cmd=False; self.puberr(ctrl,"w"); self.pub(ctrl,self.motor.get(ctrl,INIT_MOTOR))
                print("[%s] команда ОТКЛОНЕНА: %s=%d, борт не на связи (%s)"%(self.name,ctrl,want,self.mode),flush=True)
            else:
                self.motor[ctrl]=want; self.pub(ctrl,want); self.wr(s,DUTY_REG[c],want,ctrl)
        elif ctrl in self.light_map:
            is_cmd=False   # свет не относится к ходу -> не взводит режим SAILING
            s,c=self.light_map[ctrl]; want=max(0,min(100,iv))
            if not self.online:
                self.puberr(ctrl,"w"); self.pub(ctrl,self.light.get(ctrl,INIT_LIGHT))
                print("[%s] команда ОТКЛОНЕНА: %s=%d, борт не на связи (%s)"%(self.name,ctrl,want,self.mode),flush=True)
            else:
                self.light[ctrl]=want; self.pub(ctrl,want); self.wr(s,DUTY_REG[c],want,ctrl)
        elif ctrl=="mp3_track" and self.online:
            is_cmd=False; iv=max(0,min(MP3_TRACK_MAX,iv)); self.send_mp3(mp3_frame(MP3["stop"]) if iv<=0 else mp3_frame(MP3["play"],iv)); self.pub("mp3_track",iv)
        elif ctrl=="mp3_volume" and self.online:
            is_cmd=False; v=max(0,min(MP3_VOL_MAX,iv)); self.send_mp3(mp3_frame(MP3["vol"],v)); self.pub("mp3_volume",v)
        elif ctrl=="ship_number":
            # ship number = LoRa address. Persist FIRST (survives reboot even if the slow modem write is interrupted), then write modem.
            is_cmd=False; changed=(iv!=self.lora["address"])
            if changed: self.release_ship()   # отпускаем ПРЕЖНИЙ борт, пока модем ещё настроен на него
            self.lora["address"]=iv; self.apply_wiring(); self.pub(ctrl,iv); self.drv.save()
            self.drv.pub_ship_points()   # борт сменился -> у кого какая точка (и у осиротевшего борта прочерк вместо чужого режима)
            self.lora_op(self.lora)
            if changed:   # switched to a DIFFERENT boat -> re-detect so init_ship (freq=400 + idle, which arms the motor ESCs) runs for it
                self.force_init=True   # смена корабля -> полный init_ship (сброс в холостой + переарм), НЕ resume: не тащим газ с прежнего борта
                self.online=False; self.fails=0; self.due={}; self.offline_since=time.monotonic()
                print("[%s] ship_number -> %d: forcing re-init (SEARCH) for the new boat"%(self.name,iv),flush=True)
        else: is_cmd=False
        if ctrl=="enabled": self.drv.save(); self.drv.pub_ship_points()   # active у бортов этой точки изменился
        if is_cmd: self.last_cmd=time.monotonic()
    def drain(self):
        while True:
            try: ctrl,val=self.q.get_nowait()
            except queue.Empty: break
            try: self.handle(ctrl,val)
            except Exception as e: print("[%s] cmd err %s %s"%(self.name,ctrl,e),flush=True)

    # ---- LoRa modem config of THIS channel's transmitter (config-authoritative) ----
    # target = write (a dict) or self.lora (conf). Reads the modem; if it differs from target+reg5, writes it. Returns responded.
    def lora_op(self,write):
        target=write if write else self.lora
        was_open=self.ser is not None; ok=False
        try:
            self.open()
            gpio_set(self.gpio,1); time.sleep(0.4)
            self.ser.reset_input_buffer(); self.ser.write(bytes([0xC1,0x00,0x08])); self.ser.flush(); time.sleep(0.4); r=self.ser.read(64)
            if len(r)>=11 and r[0]==0xC1:
                ok=True; b=r[3:11]
                air=AIR_CODE.get(("%g"%target["air_rate"]),7); pw=PWR_CODE.get(str(int(target["power"])),0)
                # full register dump (0x00-0x07) per .6 reference: vars (addr/air/chan/power) + fixed (SPED/OPTION hi, reg5=03, crypt=00 00)
                reg5=REG5_TXMODE|0x80 if RSSI_BYTE else REG5_TXMODE   # бит7 = packet-byte RSSI (только береговой модем)
                des=bytes([(int(target["address"])>>8)&0xFF,int(target["address"])&0xFF,SPED_BASE|air,OPTION_BASE|pw,int(target["channel"])&0xFF,reg5]+REG_TAIL)
                print("[%s] modem raw=%s want=%s"%(self.name,b.hex(),des.hex()),flush=True)
                if bytes(b[0:len(des)])!=des:   # any byte differs -> write the whole dump
                    print("[%s] writing config to modem (all bytes)"%self.name,flush=True)
                    self.ser.reset_input_buffer(); self.ser.write(bytes([0xC0,0x00,len(des)])+des); self.ser.flush(); time.sleep(0.5); self.ser.read(64)
                    self.ser.reset_input_buffer(); self.ser.write(bytes([0xC1,0x00,0x08])); self.ser.flush(); time.sleep(0.4); r2=self.ser.read(64)
                    if len(r2)>=11 and r2[0]==0xC1: b=r2[3:11]; print("[%s] modem after write raw=%s"%(self.name,b.hex()),flush=True)
                self.lora={"channel":b[4],"air_rate":float(AIR_NAME.get(b[2]&7,"62.5")),
                           "address":(b[0]<<8)|b[1],"power":int(PWR_NAME.get(b[3]&3,"22"))}
                self.apply_wiring(); self.pub("ship_number",self.lora["address"]); self.drv.pub_ship_points()
            else: print("[%s] modem: no response"%self.name,flush=True)
        except Exception as e: print("[%s] lora err %s"%(self.name,e),flush=True)
        finally:
            gpio_set(self.gpio,0)
            if not was_open: self.close()
        return ok
    def probe(self):   # present? tty exists AND modem answers. Config-authoritative: write conf+reg5 to modem if it differs.
        if not os.path.exists(self.tty): return False
        for _ in range(3):
            if self.lora_op(None): self.lora_read=True; return True
        return False

    # ---- ship logic ----
    def init_ship(self):
        self.sensor_fails=0; self.sensor_gone=False   # новый борт -> заново проверяем наличие датчика курса
        print("[%s] init_ship ship=%d: freq=%d, motors->idle(%d), lights->%d"%(self.name,self.lora["address"],INIT_FREQ,INIT_MOTOR,INIT_LIGHT),flush=True)
        # Блочно (func16): по одному кадру на модуль вместо трёх — было 18 транзакций на инициализацию, стало 6.
        for s in PWM_SLAVES: self.write_regs(s,DUTY_REG[1],[0,0,0])             # 1) power (duty) off on every channel first
        for s in PWM_SLAVES: self.write_regs(s,FREQ_REG[1],[INIT_FREQ]*3)      # 2) then pwm frequency = 400
        for n,s,c in self.motors: self.motor[n]=INIT_MOTOR
        for n,s,c in self.lights: self.light[n]=INIT_LIGHT
        self.push_duty()                                                        # 3) моторы в холостой + свет — блоками по модулям
        for n,s,c in self.motors: self.pub(n,self.motor[n])
        for n,s,c in self.lights: self.pub(n,self.light[n])
        self.mod_miss={k:0 for k in self.mod_miss}; self.faulted=set()
    def pwm_kept_state(self):
        # True, если все pwm-каналы всё ещё держат INIT_FREQ -> модули не теряли питание (был провал связи, не ребут)
        for s in sorted(set([sl for _,sl,_ in self.motors]+[sl for _,sl,_ in self.lights])):
            r=self.read_regs(s,3,FREQ_REG[1],3)
            if r is None or any(f!=INIT_FREQ for f in r): return False
        return True
    def push_duty(self):
        # Разложить желаемые скважности по модулям и записать блоками (func16): 1 кадр на модуль вместо 3.
        want={}
        for n,s,c in self.motors: want.setdefault(s,{})[c]=self.motor[n]
        for n,s,c in self.lights: want.setdefault(s,{})[c]=self.light[n]
        for s,chans in want.items():
            cs=sorted(chans)
            if cs==list(range(cs[0],cs[0]+len(cs))):                     # каналы подряд -> один кадр
                self.write_regs(s,DUTY_REG[cs[0]],[chans[c] for c in cs])
            else:
                for c in cs: self.write_reg(s,DUTY_REG[c],chans[c])      # с дырой — поштучно
    def release_ship(self):
        # Уходим от борта (переключились на другой или выключили точку) — оставляем его в безопасном виде:
        # газ в холостой, свет погашен кроме ходовых огней, звук выключен. Без этого брошенный катер
        # продолжает идти на прежнем газу, пока не сядет: точка просто перестаёт с ним говорить, а
        # скважность остаётся в его ШИМ-модулях.
        # ВАЖНО: зовётся ДО смены адреса модема и до закрытия порта, иначе команды уйдут в никуда.
        if not self.online:
            return False   # борта и так нет на связи — писать некуда, только эфир занимать
        for n,_,_ in self.motors: self.motor[n]=INIT_MOTOR
        for n,_,_ in self.lights:
            if n not in KEEP_ON_RELEASE: self.light[n]=0
        self.push_duty()
        try: self.send_mp3(mp3_frame(MP3["stop"]))
        except Exception as e: print("[%s] звук не выключился: %s"%(self.name,e),flush=True)
        for n,_,_ in self.motors: self.pub(n,self.motor[n])
        for n,_,_ in self.lights: self.pub(n,self.light[n])
        self.pub("mp3_track",0)
        print("[%s] отпускаю борт %d: газ в холостой, свет погашен (кроме %s), звук выкл"
              %(self.name,self.lora["address"],", ".join(sorted(KEEP_ON_RELEASE))),flush=True)
        return True
    def resume_ship(self):
        # реконнект после короткого провала связи: модули живы, просто заново утверждаем последний заданный газ/свет — БЕЗ сброса в холостой
        self.push_duty()
        for n,s,c in self.motors: self.pub(n,self.motor[n])
        for n,s,c in self.lights: self.pub(n,self.light[n])
    def pub_comms(self):
        # скользящее окно COMMS_WIN. Ключевое различие:
        #   comms_errors   — сырые промахи попыток (качество канала, до маскировки ретраем)
        #   read_failures  — отказы ПОСЛЕ всех ретраев = реально потерянные данные (то, что видит оператор)
        cut=time.monotonic()-COMMS_WIN
        self._rd_att=[t for t in self._rd_att if t>=cut]
        self._rd_miss=[t for t in self._rd_miss if t>=cut]
        self._rd_fail=[t for t in self._rd_fail if t>=cut]
        self._rd_retry_ok=[t for t in self._rd_retry_ok if t>=cut]
        for k in self._rd_kind: self._rd_kind[k]=[t for t in self._rd_kind[k] if t>=cut]
        self._lat=[(t,v) for (t,v) in self._lat if t>=cut]
        errs=len(self._rd_miss); att=len(self._rd_att)
        self.pub("comms_errors",errs)
        self.pub("link_quality",round(100.0*(1-errs/att),1) if att else 100.0)
        self.pub("read_failures",len(self._rd_fail))
        self.pub("err_timeout",len(self._rd_kind["to"]))                                    # «радийная» подпись
        self.pub("err_frame",len(self._rd_kind["short"])+len(self._rd_kind["hdr"])+len(self._rd_kind["crc"]))  # фрейминг/тайминги/искажение
        self.pub("retry_fixed",len(self._rd_retry_ok))
        lat=None
        if self._lat:
            v=sorted(x for _,x in self._lat)
            lat=round(v[min(len(v)-1,int(0.95*len(v)))],0)
            self.pub("lat_p95",lat)
        self.pub_link_score(errs,att,len(self._rd_fail),lat)
    def pub_link_score(self,errs,att,fails,lat):
        # Единый индекс качества связи 0..100 из четырёх сырых показателей.
        # Пороги — из измеренной нормы: RSSI ~-53, link_quality ~96.6 %, read_failures 0, lat_p95 ~117 мс.
        cl=lambda x: max(0.0,min(100.0,x))
        q_fail=100.0*math.exp(-fails/12.0)                      # 1->92, 2->85, 5->66, 10->43 (одиночная потеря ~0.3% чтений — не повод для тревоги)
        q_loss=cl((100.0*(1-errs/att)-80.0)/20.0*100.0) if att else 100.0   # 100%->100, 95->75, 90->50, <=80->0
        parts=[(q_fail,0.45),(q_loss,0.25)]
        if self.rssi is not None: parts.append((cl((self.rssi+100.0)/50.0*100.0),0.20))   # -50->100, -75->60, -90->25
        # задержка: штраф только за РОСТ. Базовая p95 зависит от борта (без датчика ~95 мс, с инклинометром ~220 мс
        # из-за длинного блочного чтения) — поэтому до 250 мс штрафа нет, иначе индекс наказывал бы за наличие датчика.
        if lat is not None:       parts.append((cl(100.0-max(0.0,lat-250.0)/6.0),0.10))   # <=250мс->100, 400->75, 600->42
        w=sum(x[1] for x in parts)
        score=sum(v*k for v,k in parts)/w if w else 100.0
        score=min(score,q_fail+20.0)   # реальные провалы всегда тянут вниз, даже при отличном сигнале
        self.pub("link_score",int(round(cl(score))))
    def poll_current(self):
        r=self.read_regs(UPS,4,UPS_VIN,4)   # one block: regs 2..5 = Vin, Vout, Vbat, Ibat (input voltage rides along at the 5 s current rate)
        self.pub_comms()   # счётчики связи обновляем каждым опросом (учитывают и этот промах, если был)
        if r is None: self.puberr("battery_current","r"); self.puberr("battery_voltage","r"); self.puberr("input_voltage","r"); return False
        self.tele["current"]=s16(r[3])*0.001; self.pub("battery_current",round(self.tele["current"],3)); self.puberr("battery_current","")
        self.tele["vbat"]=r[2]*0.001; self.tele["vin"]=r[0]*0.001   # нужны станции для индикатора связи катушек
        self.pub("battery_voltage",round(r[2]*0.001,2)); self.puberr("battery_voltage","")   # Vbat (АКБ, ~8 В) — рядом с зарядом
        self.pub("input_voltage",round(r[0]*0.001,2)); self.puberr("input_voltage","")
        if self.rssi is not None: self.pub("rssi",self.rssi); self.puberr("rssi","")   # обновлён чтением выше
        return True
    def pwm_alive(self):   # ship reachable via pwm even when UPS is off — probe each pwm8a04 frequency register
        for s in PWM_SLAVES:
            if self.read_regs(s,3,FREQ_REG[1],1) is not None: return True
        return False
    def poll_temp(self):
        r=self.read_regs(UPS,4,UPS_TEMP,1)
        if r is None: self.puberr("battery_temperature","r"); return False
        self.tele["temp"]=s16(r[0])*0.01; self.pub("battery_temperature",round(self.tele["temp"],2)); self.puberr("battery_temperature","")
        if THERM_ON and self.mode==CHARGE: self.thermal_pid(self.tele["temp"])
        return True
    def thermal_pid(self,t):
        # Держим температуру батареи у T_TARGET и при этом выжимаем максимально возможный ток заряда.
        # Зачем: аппаратная защита УПС рвёт заряд при +50 °C, после чего он идёт рывками и греет впустую.
        # Процесс инерционный, поэтому регулируем по ПРЕДСКАЗАННОЙ температуре: t + скорость_роста * TH_LEAD.
        # Это тот же D-член, но в понятном виде — «где будем через TH_LEAD минут». Чистого D по шуму нет.
        now=time.monotonic()
        self._th_win.append((now,t)); self._th_win=[(x,y) for x,y in self._th_win if now-x<=TH_SLOPE_WIN]
        # Наклон — методом наименьших квадратов по всему окну, а НЕ разностью крайних точек.
        # Датчик отдаёт температуру ступеньками ~0.1 °C, и разность двух точек давала скачки
        # ±0.3 °C/мин: умноженные на горизонт и Kp, они мотали ток 300<->1100 мА и писали регистр
        # 18 сотни раз в сутки. МНК по 240 с гасит это на порядок (проверено на модели с квантованием).
        slope=0.0
        w=self._th_win
        if len(w)>2:
            n=len(w); mx=sum(x for x,_ in w)/n; my=sum(y for _,y in w)/n
            den=sum((x-mx)**2 for x,_ in w)
            if den>0: slope=sum((x-mx)*(y-my) for x,y in w)/den*60.0    # °C/мин
        e=T_TARGET-(t+slope*TH_LEAD)
        if t>=T_TARGET+1.0:                       # вплотную к отсечке — сразу в минимум, интеграл сбрасываем
            self._th_i=0.0; out=CHG_MIN
        else:
            sat=(self.chg_setpoint>=CHG_FULL and e>0) or (self.chg_setpoint<=CHG_MIN and e<0)
            if not sat:                            # анти-виндап: в насыщении не копим
                step=now-getattr(self,"_th_t",now)
                self._th_i=max(0.0,min(float(CHG_FULL),self._th_i+TH_KI*e*min(step,60.0)))
            out=TH_KP*e+self._th_i
        self._th_t=now
        out=int(max(CHG_MIN,min(CHG_FULL,out)))
        # регистр 18 — настроечный, пишем редко: только заметное изменение и не чаще TH_WRITE_S
        if abs(out-self.chg_setpoint)>=TH_DEAD and now-self._th_w>=TH_WRITE_S:
            self._th_w=now
            if self.write_reg(UPS,UPS_CHG_SETPOINT,out):
                self.chg_setpoint=out; self.pub("charge_setpoint",out)
                print("[%s] заряд: t=%.1f °C (прогноз %.1f, %+.2f °C/мин) -> ток %d мА"%(self.name,t,t+slope*TH_LEAD,slope,out),flush=True)
    def poll_charge(self):
        r=self.read_regs(UPS,4,UPS_CHG,1)
        if r is None: self.puberr("charge_level","r"); return False
        self.pub("charge_level",round(r[0]*0.01,1)); self.puberr("charge_level",""); return True
    def poll_course(self):
        # Датчик WT901C485: всё сырьё одним блоком (углы он считает сам, интегрировать не нужно).
        # Публикуем широко — для разбора по логам: ускорения, гироскоп, магнитометр, углы, кватернионы.
        # Датчик стоит не на каждом борту -> после SENSOR_GIVEUP неудач замолкаем.
        sen=getattr(self,"sensor",SENSOR_FALLBACK)
        if not sen.get("enabled",True) or self.sensor_gone: return True
        addr=int(sen.get("address",14))
        # Читаем КОРОТКИЙ блок 0x34..0x40. Раньше читался один блок из 33 регистров: он тащил
        # 16 неиспользуемых регистров (0x41..0x50) только чтобы одной транзакцией достать
        # кватернионы, и падал примерно в 5 % чтений. Разбор 25.08 по журналу за 48 ч: 482 отказа
        # чтения 33 регистров, из них 435 (90 %) сразу лечились чтением 13 — то есть дело было в
        # длине, а не в датчике. Каждый отказ стоил 4 транзакции по ~810 мс = ~3.2 с занятого
        # канала, из-за чего мигали красным и посторонние контролы.
        r=self.read_regs(addr,3,IMU_BASE,IMU_SHORT,tries=1,stats=False)
        if r is None:
            self.sensor_fails+=1
            if self.sensor_fails>=SENSOR_GIVEUP:
                self.sensor_gone=True
                # Пометку НЕ снимаем: опрос выключен, значения застыли. Раньше здесь стоял
                # puberr(c,"") и course выглядел живым, хотя его уже никто не обновлял.
                for c in IMU_PUB: self.puberr(c,"r")
                print("[%s] датчик (адрес %d) не отвечает %d раз -> опрос выключен для этого борта, значения застыли"%(self.name,addr,self.sensor_fails),flush=True)
            else:
                for c in IMU_PUB: self.puberr(c,"r")
            return False
        self.sensor_fails=0
        g=lambda i: s16(r[i])
        for i,ax in enumerate("xyz"):
            self.pub("accel_"+ax,round(g(IMU_ACC+i)*ACC_SCALE,3))
            self.pub("gyro_"+ax,round(g(IMU_GYR+i)*GYR_SCALE,2))
            self.pub("mag_"+ax,g(IMU_MAG+i))
        roll=g(IMU_ANG)*ANG_SCALE; pitch=g(IMU_ANG+1)*ANG_SCALE; yaw=g(IMU_ANG+2)*ANG_SCALE
        if sen.get("invert"): yaw=-yaw
        if yaw<=-180: yaw+=360
        elif yaw>180: yaw-=360
        self.pub("roll",round(roll,2)); self.pub("pitch",round(pitch,2)); self.pub("course",round(yaw,1))
        if len(r)>IMU_TEMP: self.pub("sensor_temp",round(g(IMU_TEMP)*0.01,1))
        now=time.monotonic()                       # кватернионы — своим коротким чтением и редко
        if now-self._q_at>=IMU_Q_PERIOD:
            self._q_at=now
            q=self.read_regs(addr,3,IMU_Q_BASE,4,tries=1,stats=False)
            if q is not None:
                for i in range(4): self.pub("q%d"%i,round(s16(q[i])*Q_SCALE,4))
        for c in IMU_PUB: self.puberr(c,"")
        return True
    def poll_pwm_readback(self):
        # read-back of all motors+lights via ONE block per pwm8a04 (regs 112..114 = ch1/2/3), then distribute
        ok=True; block={}
        for s in sorted(set([sl for _,sl,_ in self.motors]+[sl for _,sl,_ in self.lights])):
            block[s]=self.read_regs(s,3,DUTY_REG[1],3)   # DUTY_REG[1]=112 -> [ch1,ch2,ch3]
        for n,s,c in self.motors+self.lights:
            r=block.get(s)
            if r is None: ok=False; self.puberr(n,"r")
            else:
                hw=r[c-1]; want=(self.motor if n in self.motor else self.light).get(n)
                # Раньше сюда публиковалось железное значение, а self.motor не менялся:
                # ползунок показывал одно, драйвер считал другое, decide() давал IDLE при
                # видимом газе. Хозяин — драйвер: при расхождении утверждаем своё.
                if want is not None and hw!=want:
                    print("[%s] %s: в железе %d, задано %d -> переписываю"%(self.name,n,hw,want),flush=True)
                    self.wr(s,DUTY_REG[c],want,n)
                else:
                    self.pub(n,hw); self.puberr(n,"")
        return ok
    def poll_freq_check(self):
        # pwm freq вернулась к дефолту = модуль ребутнулся (браунаут/просадка 5 В).
        # Одной частоты мало: после ребута ESC НЕ вооружены, и если восстановить
        # только freq — моторы остаются мёртвыми ("не очухивается", инцидент 57%+).
        # Поэтому форсируем ПОЛНУЮ реинициализацию: online=False -> SEARCH переловит
        # модуль и вызовет init_ship (freq=400 + переарм ESC + холостой 40).
        for s in sorted(set([sl for _,sl,_ in self.motors]+[sl for _,sl,_ in self.lights])):
            r=self.read_regs(s,3,FREQ_REG[1],3)   # FREQ_REG[1]=0 -> [ch1,ch2,ch3]
            if r is None:
                # раньше тут был молчаливый continue: навсегда пропавший модуль не поднимал
                # ничего, кроме meta/error="r" на своих контролах (случай 24.08: модуль 11
                # молчал 7.5 часов, правые моторы и навигация мертвы, борт шёл на левых).
                self.mod_miss[s]=self.mod_miss.get(s,0)+1
                if self.mod_miss[s]>=LOST_MISSES: self.module_fault(s)
                continue
            self.mod_miss[s]=0; self.faulted.discard(s)
            for i,f in enumerate(r):
                if f!=INIT_FREQ:
                    print("[%s] pwm addr %d ch%d freq drift %d->%d: модуль ребутнулся -> форсирую реинициализацию (переарм ESC)"%(self.name,s,i+1,f,INIT_FREQ),flush=True)
                    self.online=False; self.fails=0; self.due={}; self.offline_since=time.monotonic()   # -> run(): SEARCH -> init_ship переармит ESC и вернёт холостой
                    return True
        return True
    def module_fault(self,slave):
        """Модуль не отвечает совсем: его каналы неуправляемы, идти с этим нельзя.
        Гасим ОСТАЛЬНЫЕ моторы в холостой и пишем ошибку. Повторно не дёргаем,
        пока модуль не ответит."""
        if slave in self.faulted: return
        self.faulted.add(slave)
        dead=[nm for nm,sl,c in self.motors+self.lights if sl==slave]
        print("[%s] ОШИБКА: модуль %d не ответил %d раз подряд. Неуправляемы: %s. "
              "Гашу остальные моторы в холостой (%d)."
              %(self.name,slave,self.mod_miss.get(slave,0),", ".join(dead),INIT_MOTOR),flush=True)
        for nm,sl,c in self.motors:
            if sl==slave: continue                 # его каналы всё равно не пишутся
            self.motor[nm]=INIT_MOTOR
            self.wr(sl,DUTY_REG[c],INIT_MOTOR,nm)
            self.pub(nm,INIT_MOTOR)

    GROUPS={"current":"poll_current","temp":"poll_temp","charge":"poll_charge","pwm_readback":"poll_pwm_readback","freq_check":"poll_freq_check","course":"poll_course"}
    def decide(self):
        if not self.online: return SEARCH
        if any(v>INIT_MOTOR for v in self.motor.values()): return SAIL   # держим SAILING, пока хоть один мотор выше холостого
        if time.monotonic()-self.last_cmd < SAIL_TIMEOUT: return SAIL     # грейс-окно после последней команды мотором
        return CHARGE if self.tele.get("current",0)>0 else IDLE
    def set_mode(self,m):
        if m!=self.mode:
            self.mode=m; self.pub("mode",m)
            want_full=(m not in (SEARCH,OFF))   # polling -> show full dashboard; not polling -> only enabled+mode
            if want_full!=self.declared_full: self.drv.boat_controls(self,want_full)
            if m==CHARGE:   # входим в заряд с полного тока; регулятор сам снизит по мере нагрева
                self.chg_setpoint=CHG_FULL; self.write_reg(UPS,UPS_CHG_SETPOINT,CHG_FULL)
                self.pub("charge_setpoint",CHG_FULL)
                self._th_i=float(CHG_FULL); self._th_win=[]; self._th_w=0.0; self._th_t=time.monotonic()
            if m==OFF: gpio_set(self.gpio,1)   # disabled -> put MOD modem into config mode (off-air)

    def run(self):
        while True:
            self.drain()
            if not self.enabled:
                self.set_mode(OFF); time.sleep(0.5); continue
            self.open()
            if not self.lora_read:                         # at start: read modem settings once (read-only, never auto-write)
                self.lora_op(None); self.lora_read=True
            now=time.monotonic()
            if not self.online:
                self.set_mode(SEARCH)
                if self.poll_current() or self.pwm_alive():
                    dt=now-getattr(self,"offline_since",now)
                    self.fails=0; self.online=True; self.due={}   # due={} -> re-poll all groups at once on (re)connect
                    self.drv.boat_controls(self,True)             # restore full dashboard before init/poll fills values
                    if self.pwm_kept_state() and not self.force_init:   # провал связи (не смена корабля), модули живы -> НЕ сбрасываем газ
                        print("[%s] снова на связи после %.1f с (провал связи, питание не терялось) -> восстанавливаю газ без init"%(self.name,dt),flush=True)
                        self.resume_ship()
                    else:                                              # смена корабля ИЛИ ребут модуля -> полный init (сброс в холостой + переарм)
                        print("[%s] снова на связи после %.1f с (%s) -> init_ship"%(self.name,dt,"смена корабля" if self.force_init else "модуль ребутнулся"),flush=True)
                        self.init_ship(); self.last_cmd=0.0
                    self.force_init=False
                    self.set_mode(self.decide())
                    self.drv.bind_ship(self)   # борт тут реально есть -> закрепляем привязку борт<->точка
                else: time.sleep(SEARCH_PERIOD)
                continue
            self.set_mode(self.decide())
            if not getattr(self,"_warned_groups",False):   # tolerate confs from older versions (renamed/removed poll groups)
                unknown=sorted({g for r in RATES.values() for g in r if g not in self.GROUPS})
                if unknown: print("[%s] игнорирую незнакомые группы опроса в conf: %s"%(self.name,unknown),flush=True)
                self._warned_groups=True
            did=False
            for g,per in RATES[self.mode].items():
                fn=self.GROUPS.get(g)
                if fn is None: continue                     # unknown group -> skip, don't crash the channel thread
                if now>=self.due.get(g,0):
                    ok=getattr(self,fn)(); self.due[g]=now+per; did=True
                    if g=="current":
                        if ok or self.pwm_alive(): self.fails=0   # UPS may be off; ship still alive if any pwm8a04 answers
                        else:
                            self.fails+=1
                            print("[%s] промах связи #%d (UPS и все pwm не ответили)"%(self.name,self.fails),flush=True)
                            if self.fails>=OFFLINE_FAILS:
                                print("[%s] -> offline после %d промахов, ухожу в SEARCH"%(self.name,self.fails),flush=True)
                                self.offline_since=now; self.online=False; self.set_mode(SEARCH)
            if not did: time.sleep(0.2)

class ModbusTCP:
    # Modbus-RTU framed over a transparent TCP serial-gateway (e.g. EBYTE): same RTU frames + CRC16, sent over a socket.
    def __init__(self,host,port,timeout=1.0):
        self.host=host; self.port=int(port); self.timeout=timeout; self.sock=None; self.lock=threading.Lock()
    def connect(self):
        if self.sock is None:
            self.sock=socket.create_connection((self.host,self.port),self.timeout); self.sock.settimeout(self.timeout)
    def close(self):
        if self.sock is not None:
            try: self.sock.close()
            except Exception: pass
            self.sock=None
    def _drain(self):   # discard any stale bytes buffered by the gateway before a new transaction
        try:
            self.sock.setblocking(False)
            while self.sock.recv(512): pass
        except Exception: pass
        finally:
            try: self.sock.setblocking(True); self.sock.settimeout(self.timeout)
            except Exception: pass
    def _txn(self,req,n):
        with self.lock:
            self.connect(); self._drain()
            try: self.sock.sendall(req)
            except Exception: self.close(); raise
            end=time.monotonic()+self.timeout; buf=b""
            while len(buf)<n and time.monotonic()<end:
                try: c=self.sock.recv(n-len(buf))
                except socket.timeout: break
                except Exception: self.close(); raise
                if not c: self.close(); break
                buf+=c
            return buf
    def read_input(self,slave,addr,n):   # func 4 -> list of n 16-bit regs
        req=bytes([slave,4,(addr>>8)&0xFF,addr&0xFF,(n>>8)&0xFF,n&0xFF]); req+=crc16(req)
        r=self._txn(req,5+2*n)
        if len(r)>=5+2*n and r[0]==slave and r[1]==4 and r[2]==2*n and crc16(r[:3+2*n])==r[3+2*n:5+2*n]:
            return [(r[3+2*i]<<8)|r[4+2*i] for i in range(n)]
        return None
    def read_discrete(self,slave,addr,n):   # func 2 -> list of n bits
        nb=(n+7)//8; req=bytes([slave,2,(addr>>8)&0xFF,addr&0xFF,(n>>8)&0xFF,n&0xFF]); req+=crc16(req)
        r=self._txn(req,5+nb)
        if len(r)>=5+nb and r[0]==slave and r[1]==2 and crc16(r[:3+nb])==r[3+nb:5+nb]:
            return [(r[3+(i//8)]>>(i%8))&1 for i in range(n)]
        return None
    def write_coil(self,slave,addr,on):   # func 5
        req=bytes([slave,5,(addr>>8)&0xFF,addr&0xFF,0xFF if on else 0x00,0x00]); req+=crc16(req)
        r=self._txn(req,8); return len(r)>=8 and r[0]==slave and r[1]==5

class ChargerBus(threading.Thread):
    # Single thread; one ModbusTCP per DISTINCT gateway (each charger may have its own gateway; chargers that
    # share a gateway share one socket+lock). Coils are written with our own RTU frames, so at start we only
    # REFLECT the real relay state, never toggle it.
    def __init__(self,drv,chargers):
        super().__init__(daemon=True)
        self.drv=drv; self.chargers=chargers; self.q=queue.Queue()
        self._lk=[]   # окно сглаживания индикатора посадки
        self.buses={}   # gateway str -> ModbusTCP
    def dev(self,i): return "charger%d"%(i+1)
    def bus_for(self,ch):
        g=ch.get("gateway")
        if not g: return None
        b=self.buses.get(g)
        if b is None:
            host,_,port=g.partition(":"); b=ModbusTCP(host or "127.0.0.1", port or 8886); self.buses[g]=b
        return b
    def pub(self,dev,ctrl,val):
        if self.drv.mqtt is not None: self.drv.mqtt.publish("/devices/%s/controls/%s"%(dev,ctrl),str(val),retain=True)
    def puberr(self,dev,ctrl,err):
        if self.drv.mqtt is not None: self.drv.mqtt.publish("/devices/%s/controls/%s/meta/error"%(dev,ctrl),err,retain=True)
    def relay_set(self,ch,out,on):
        bus=self.bus_for(ch)
        if bus is None: return False
        v=(not on) if out.get("invert") else on
        return bus.write_coil(int(out["address"]),MRM_COIL0+(int(out["channel"])-1),bool(v))
    def relay_state(self,ch,out):
        bus=self.bus_for(ch)
        if bus is None: return None
        r=bus.read_discrete(int(out["address"]),MRM_STATE0+(int(out["channel"])-1),1)
        if r is None: return None
        st=bool(r[0]); return (not st) if out.get("invert") else st
    def read_current(self,ch):   # WB-MAI6 input voltage (s32) / shunt -> amps
        bus=self.bus_for(ch)
        if bus is None: return None
        s=ch["sensor"]; reg=MAI_IN0+2*(int(s.get("input",1))-1)
        r=bus.read_input(int(s["address"]),reg,2)
        if r is None: return None
        return s32(r[0],r[1])*MAI_VOLT_SCALE/float(s.get("shunt_ohm",1.2))
    def link_pct(self):
        # Индикатор посадки катушек — по ПРОСАДКЕ входного напряжения, а не по току.
        # Почему не «факт/уставка»: когда батарея заряжена, УПС сам перестаёт брать ток (борт при этом
        # продолжает питаться от площадки) — и метрика по току показывала бы «плохую связь» там, где всё в порядке.
        # Просадка от такого не зависит: при хорошей передаче Vin держится у холостых ~12.7 В и садится
        # примерно на 0.35 В на каждый ампер заряда; при кривой посадке проваливается на 2 В и больше.
        # Проверено на 9 днях истории: хорошие сессии дают недобор 0.03…0.26 В, плохие — 2.0…2.2 В.
        for ch in self.drv.channels.values():
            vin=ch.tele.get("vin",0.0)
            if vin<=5: continue                                  # борт не на паду (энергии нет)
            ib=max(0.0,ch.tele.get("current",0.0))               # ток заряда, А (разряд не учитываем)
            dev=(LK_VOPEN-LK_REFF*ib)-vin                        # недобор напряжения против ожидаемого
            p=max(0,min(100,int(round(100.0*(1.0-dev/LK_DEVFULL)))))
            self._lk.append(p); self._lk=self._lk[-LK_SMOOTH:]   # сглаживаем: на старте/остановке заряда бывают выбросы
            med=sorted(self._lk)[len(self._lk)//2]
            return med,"недобор %.2f В"%dev
        return None,"борта на паду нет"
    def handle(self,dev,ctrl,val):
        on=(val in ("1","true","on"))
        for i,ch in enumerate(self.chargers):
            if self.dev(i)!=dev: continue
            out=ch.get(ctrl) if ctrl in ("transmitter","magnets") else None
            if out is None: return
            ok=self.relay_set(ch,out,on); self.pub(dev,ctrl,1 if on else 0)
            print("[%s] %s -> %s (%s)"%(dev,ctrl,on,"ok" if ok else "FAIL"),flush=True); return
    def run(self):
        while True:
            while True:
                try: dev,ctrl,val=self.q.get_nowait()
                except queue.Empty: break
                try: self.handle(dev,ctrl,val)
                except Exception as e: print("[chg] cmd err %s %s"%(ctrl,e),flush=True)
            for i,ch in enumerate(self.chargers):
                dev=self.dev(i)
                try:
                    cur=self.read_current(ch)
                    if cur is None: self.puberr(dev,"transmitter_current","r")
                    else: self.pub(dev,"transmitter_current",round(cur,3)); self.puberr(dev,"transmitter_current","")
                    tx_on=None
                    for ctrl in ("transmitter","magnets"):
                        out=ch.get(ctrl)
                        if not out: continue
                        st=self.relay_state(ch,out)
                        if st is None: self.puberr(dev,ctrl,"r")
                        else:
                            self.pub(dev,ctrl,1 if st else 0); self.puberr(dev,ctrl,"")
                            if ctrl=="transmitter": tx_on=st
                    # связь катушек: считаем только когда этот передатчик включён и на нём кто-то заряжается
                    pct,why=self.link_pct() if tx_on else (None,"передатчик выключен")
                    if pct is None: self.puberr(dev,"charge_link","r")
                    else: self.pub(dev,"charge_link",pct); self.puberr(dev,"charge_link","")
                except Exception as e:
                    print("[chg] poll err %s %s"%(dev,e),flush=True)
                    b=self.bus_for(ch)
                    if b is not None: b.close()
            time.sleep(CHG_PERIOD)

class Driver:
    def __init__(self):
        st=self.load()
        self.channels={}
        for n,(tty,g) in CHANNELS.items():
            en=st.get(n,{}).get("enabled", n in ENABLED_AT_START)
            ch=Channel(self,n,tty,g,en); self.channels[n]=ch
            sn=st.get(n,{}).get("ship_number")   # persist last-entered ship number across reboot (written to modem at start)
            if sn is not None:
                try: ch.lora["address"]=int(sn); ch.apply_wiring()
                except Exception: pass
        self.mqtt=None
        # Какая радиоточка выделена борту. Единственное, что хранится на уровне корабля: сам факт
        # «борт активен» НЕ хранится, а выводится из того, кто сейчас занимает точку и включена ли она —
        # иначе появился бы второй источник правды, который начал бы расходиться с boatN.
        # По умолчанию борта раскиданы по точкам по кругу: борт без точки — состояние ненормальное,
        # его нечем показать в панели (точки 0 не существует). Само по себе это ничего не включает:
        # борт занимает точку только если та несёт его номер.
        npt=max(1,len(CHANNELS))
        self.ship_pt={n:((n-1)%npt)+1 for n in SHIP_NUMBERS}
        for k,v in (st.get("ships") or {}).items():
            try:
                p=int((v or {}).get("point",0))
                if int(k) in self.ship_pt and 1<=p<=npt: self.ship_pt[int(k)]=p
            except Exception: pass
        # Из номера, лежащего в модеме точки, привязку НЕ выводим: там может лежать метка, вписанная
        # когда-то руками, а борта на этой точке нет уже неделю. Привязка появляется, только когда борт
        # на точке реально ответил (bind_ship), либо когда её задал оператор.
        self.setup_number=int(SETUP_DEFAULTS["address"]); self.setup_channel=int(SETUP_DEFAULTS["channel"])   # Ship Setup editable: number + channel
        self.setup_air=float(SETUP_DEFAULTS["air_rate"]); self.setup_power=int(SETUP_DEFAULTS["power"])        # preserved from last read, used on write
        self.setupq=queue.Queue(); self.shipq=queue.Queue()
        self.chargerbus=ChargerBus(self,[dict(c) for c in CHG_LIST]) if CHG_LIST else None
    def load(self):
        try: return json.load(open(STATE_FILE))
        except Exception: return {}
    def save(self):
        # Точки остаются как были (совместимо с прежним файлом), борта добавлены отдельным ключом:
        # "ships" не конфликтует с именами "mod1".."mod4", поэтому миграция не нужна ни туда, ни обратно.
        try:
            st={n:{"enabled":c.enabled,"ship_number":c.lora["address"]} for n,c in self.channels.items()}
            st["ships"]={str(n):{"point":p} for n,p in sorted(self.ship_pt.items())}
            json.dump(st,open(STATE_FILE,"w"))
        except Exception as e: print("state save err",e,flush=True)
    def setup_mqtt(self):
        try:
            from paho.mqtt.client import CallbackAPIVersion
            c=mqtt.Client(CallbackAPIVersion.VERSION1)   # paho-mqtt 2.x (WB8/trixie)
        except ImportError:
            c=mqtt.Client()                               # paho-mqtt 1.x (older Debian / WB7)
        c.on_connect=self.on_connect; c.on_message=self.on_message
        c.connect("localhost",1883,60); self.mqtt=c; c.loop_start()
    def setname(self,dev,title):
        # newer WB firmware reads the device title from the /devices/<id>/meta JSON object;
        # /meta/name is legacy (older WB7). Publish both.
        self.mqtt.publish("/devices/%s/meta"%dev,json.dumps({"driver":"ship-driver","title":{"en":title,"ru":title}}),retain=True)
        self.mqtt.publish("/devices/%s/meta/name"%dev,title,retain=True)
    def pub_ctrl_meta(self,dev,name,m):
        # homeui Devices page reads the JSON /meta; homeui DASHBOARD cells resolve only via the
        # legacy per-field topics (/meta/type, /meta/order, ...). Publish both for full compatibility.
        base="/devices/%s/controls/%s/meta"%(dev,name)
        self.mqtt.publish(base,json.dumps(m),retain=True)
        self.mqtt.publish(base+"/type",str(m.get("type","text")),retain=True)
        self.mqtt.publish(base+"/order",str(m.get("order",1)),retain=True)
        self.mqtt.publish(base+"/readonly","1" if m.get("readonly") else "0",retain=True)
        for k in ("min","max","units","precision"):
            if m.get(k) is not None: self.mqtt.publish(base+"/"+k,str(m[k]),retain=True)
    def clear_ctrl(self,dev,name):   # wipe a control incl. legacy meta subtopics
        for sub in ("/meta","/meta/type","/meta/order","/meta/readonly","/meta/min","/meta/max","/meta/units","/meta/precision","/meta/error",""):
            self.mqtt.publish("/devices/%s/controls/%s%s"%(dev,name,sub),"",retain=True)
    def boat_controls(self,ch,full):
        # full=True: publish the whole dashboard; full=False: keep only enabled+mode, remove the rest
        # (the "extra" controls are telemetry/commands that only make sense while the channel polls a live ship).
        if self.mqtt is None: return
        d=ch.dev; o=[0]
        def ctl(name,meta,val=None):
            o[0]+=1; m=dict(meta,order=o[0])
            if isinstance(m.get("title"),str): m["title"]={"en":m["title"],"ru":m["title"]}   # homeui needs title as {lang:...} object
            self.pub_ctrl_meta(d,name,m)
            if val is not None: self.mqtt.publish("/devices/%s/controls/%s"%(d,name),str(val),retain=True)
        ctl("enabled",{"type":"switch","readonly":False,"title":"Enabled"},1 if ch.enabled else 0)
        ctl("mode",{"type":"text","readonly":True,"title":"Mode"})
        ctl("ship_number",{"type":"value","readonly":False,"min":0,"max":ADDR_MAX,"title":"Ship number"},ch.lora["address"])   # always visible (set ship even while searching)
        if full:
            for nm,u,t in BOAT_TELE:
                ctl(nm,{"type":"value","readonly":True,"units":u,"title":t})
            for n2 in MOTOR_NAMES: ctl(n2,{"type":"range","readonly":False,"min":MOTOR_MIN,"max":MOTOR_MAX,"title":MOTOR_TITLE[n2]})
            for n in LIGHT_NAMES: ctl(n,{"type":"range","readonly":False,"min":0,"max":100,"title":LIGHT_TITLE.get(n,n)})
            ctl("mp3_track",{"type":"range","readonly":False,"min":0,"max":MP3_TRACK_MAX,"title":"Audio track"})
            ctl("mp3_volume",{"type":"range","readonly":False,"min":0,"max":MP3_VOL_MAX,"title":"Volume"})
        else:
            for c in BOAT_EXTRA: self.clear_ctrl(d,c)   # remove control: clear meta (incl. legacy), error, value
        ch.declared_full=full
    def channel_for_ship(self,n):
        # Точка борта — та, к которой он ПРИВЯЗАН и которая при этом несёт его номер. Нужны оба условия:
        #   метка без привязки — могла остаться в модеме с прошлой недели, борта там давно нет;
        #   привязка без метки — номер на точке уже сменили, борт с неё снят.
        # Заодно снимается неоднозначность: после переезда номер остаётся и в модеме прежней точки
        # (стирать его — лишняя запись в модем), но владельцем она уже не считается.
        ch=self.pt_channel(self.ship_pt.get(n,0))
        try: return ch if (ch is not None and int(ch.lora["address"])==n) else None
        except Exception: return None
    def ship_controls(self,n):
        # Вкладка борта: те же значения, что на его точке, но БЕЗ радиометрик (они про берег).
        # Устройство объявляется всегда, даже если борт сейчас ни на одной точке: иначе история корабля
        # рвалась бы при каждом переключении, а ради неё всё и делается. Значения просто перестают обновляться.
        d="ship%d"%n; o=[0]
        def ctl(name,meta,val=None):
            o[0]+=1; m=dict(meta,order=o[0])
            if isinstance(m.get("title"),str): m["title"]={"en":m["title"],"ru":m["title"]}
            self.pub_ctrl_meta(d,name,m)
            if val is not None: self.mqtt.publish("/devices/%s/controls/%s"%(d,name),str(val),retain=True)
        self.setname(d,"Ship %d"%n)
        # Точки 0 не существует: пока борт ни к одной не приписан, поле пустое (пустые value-ячейки
        # рисуются нормально — так же сделаны поля Ship Setup). Вписать можно только 1..4.
        # Подписи держим короткими: в узкой плашке длинный заголовок переносится и ломает вид.
        ctl("radio_point",{"type":"value","readonly":False,"min":1,"max":len(CHANNELS),"title":"Radio point"})
        ctl("active",{"type":"switch","readonly":False,"title":"Active"})
        ctl("mode",{"type":"text","readonly":True,"title":"Mode"})
        for nm,u,t in SHIP_TELE: ctl(nm,{"type":"value","readonly":True,"units":u,"title":t})
        # Ползунок БЕЗ значения homeui рисует голым числом без подписи, и плашка рассыпается — поэтому
        # значение есть у каждого всегда. У борта на точке берём то, что точка сейчас держит (не выдумываем),
        # у остальных — холостой газ и погашенный свет.
        ch=self.channel_for_ship(n)
        mot=(lambda k: ch.motor.get(k,INIT_MOTOR)) if ch is not None else (lambda k: INIT_MOTOR)
        lit=(lambda k: ch.light.get(k,0)) if ch is not None else (lambda k: 0)
        for n2 in MOTOR_NAMES: ctl(n2,{"type":"range","readonly":False,"min":MOTOR_MIN,"max":MOTOR_MAX,"title":MOTOR_TITLE[n2]},mot(n2))
        for n2 in LIGHT_NAMES: ctl(n2,{"type":"range","readonly":False,"min":0,"max":100,"title":LIGHT_TITLE.get(n2,n2)},lit(n2))
        ctl("mp3_track",{"type":"range","readonly":False,"min":0,"max":MP3_TRACK_MAX,"title":"Audio track"},0)
        ctl("mp3_volume",{"type":"range","readonly":False,"min":0,"max":MP3_VOL_MAX,"title":"Volume"},0)
    def pt_channel(self,p): return self.channels.get("mod%d"%p) if p else None
    def bind_ship(self,ch):
        # Борт ОТВЕТИЛ на этой точке — вот это и есть доказательство, что он тут стоит. Только по такому
        # событию привязка и меняется сама (номер могли сменить прямо с boatN). Метка в модеме молчащей
        # точки доказательством не является: она могла остаться там с прошлой недели.
        try: n=int(ch.lora["address"])
        except Exception: return
        p=int(ch.name[-1])
        if n in self.ship_pt and self.ship_pt[n]!=p:
            self.ship_pt[n]=p; self.save()
            print("[%s] борт %d отозвался здесь -> привязываю его к этой точке"%(ch.name,n),flush=True)
        self.pub_ship_points()
    def ship_active(self,n):
        ch=self.channel_for_ship(n)
        return bool(ch is not None and ch.enabled)
    def pub_ship_points(self):
        # active выводится, а не хранится: борт активен, если он занимает свою точку и точка включена.
        # Поэтому «включили один — остальные выключились» получается само: на точке ровно один адрес.
        # Осиротевшему борту в режим ставим прочерк, иначе висел бы чужой CHARGING от прошлой привязки.
        if self.mqtt is None: return
        for n in SHIP_NUMBERS:
            ch=self.channel_for_ship(n); on=bool(ch is not None and ch.enabled)
            self.mqtt.publish("/devices/ship%d/controls/radio_point"%n,str(self.ship_pt.get(n,0)),retain=True)
            self.mqtt.publish("/devices/ship%d/controls/active"%n,"1" if on else "0",retain=True)
            self.mqtt.publish("/devices/ship%d/controls/mode"%n,(ch.mode if (on and ch.mode) else "—"),retain=True)
    # ---- управление на уровне борта: транслируем в уже обкатанные команды точки ----
    def ship_worker(self):
        while True:
            n,ctrl,val=self.shipq.get()
            try: self.handle_ship(n,ctrl,val)
            except Exception as e: print("[ship%d] err %s %s"%(n,ctrl,e),flush=True)
    def handle_ship(self,n,ctrl,val):
        if ctrl=="radio_point":
            # принимаем только существующую точку: 1..N. Ноль, пустое и мусор игнорируем — освободить
            # точку можно тумблером active, а «борт без точки» в панели нечем показать.
            try: p=int(float(val))
            except Exception: p=0
            if not (1<=p<=len(CHANNELS)):
                print("[ship%d] радиоточки %r не существует, оставляю %s"%(n,val,self.ship_pt.get(n)),flush=True)
                self.pub_ship_points(); return
            old=self.ship_pt.get(n,0)
            if p==old: self.pub_ship_points(); return
            was=self.ship_active(n)
            self.ship_pt[n]=p; self.save()
            print("[ship%d] радиоточка %s -> %s"%(n,old or "нет",p or "нет"),flush=True)
            if was:                                   # активный борт переезжает: старую точку глушим, новую поднимаем
                oc=self.pt_channel(old)
                if oc is not None: oc.q.put(("enabled","0"))
                # p=0 — это «снять борт с точки», поднимать нечего; состояние опубликует сама точка,
                # когда выполнит выключение (публиковать сейчас рано: в очереди ещё не разобрано)
                if p: self.activate(n,p)
            else: self.pub_ship_points()
        elif ctrl=="active":
            if val in ("1","true","on"):
                p=self.ship_pt.get(n,0)
                if not p: print("[ship%d] включать нечего: не выбрана радиоточка"%n,flush=True)
                else: self.activate(n,p); return
            else:
                ch=self.channel_for_ship(n)
                if ch is not None: ch.q.put(("enabled","0"))
            self.pub_ship_points()
    def activate(self,n,p):
        ch=self.pt_channel(p)
        if ch is None:
            print("[ship%d] радиоточка mod%d недоступна (модем при старте не отозвался)"%(n,p),flush=True)
            self.pub_ship_points(); return
        # Команды кладём в очередь САМОЙ точки: их выполнит её поток тем же кодом, что и команды с boatN
        # (смена номера = переарм ESC, enabled = открыть порт). Никакой новой логики и никаких гонок.
        if int(ch.lora["address"])!=n: ch.q.put(("ship_number",str(n)))
        ch.q.put(("enabled","1"))
        print("[ship%d] занимает mod%d (прочие борта этой точки становятся неактивными)"%(n,p),flush=True)
        self.pub_ship_points()
    def declare(self):
        for n,ch in self.channels.items():
            self.setname(ch.dev,"boat%s (channel %s)"%(n[-1],ch.lora["channel"]))
            self.boat_controls(ch, ch.online and ch.enabled)   # collapsed until the channel actually polls a ship
            self.mqtt.subscribe("/devices/%s/controls/+/on"%ch.dev)
        # ---- вкладки кораблей (shipN) — зеркало физики + приём команд ----
        for n in SHIP_NUMBERS:
            self.ship_controls(n)
            self.mqtt.subscribe("/devices/ship%d/controls/+/on"%n)
        self.pub_ship_points()
        # ---- Ship Setup dashboard (RS485-1 wired config) — unchanged ----
        sd="ship_setup"
        self.setname(sd,"Ship Setup (RS485-1)")
        so=[0]
        def sctl(name,meta,val=None):
            so[0]+=1; m=dict(meta,order=so[0])
            if isinstance(m.get("title"),str): m["title"]={"en":m["title"],"ru":m["title"]}   # homeui needs title as {lang:...} object
            self.pub_ctrl_meta(sd,name,m)
            if val is not None: self.mqtt.publish("/devices/%s/controls/%s"%(sd,name),str(val),retain=True)
        # all fields start empty; filled by "Read" (setup_op) — nothing shown until we read a modem
        sctl("ship_number",{"type":"value","readonly":False,"min":0,"max":ADDR_MAX,"title":"Ship number"},"")
        sctl("LoRa_address",{"type":"value","readonly":True,"title":"LoRa address"},"")
        sctl("LoRa_channel",{"type":"value","readonly":False,"min":0,"max":83,"title":"LoRa channel"},"")
        sctl("LoRa_freq",{"type":"value","readonly":True,"units":"MHz","title":"Frequency"},"")
        sctl("LoRa_grkch",{"type":"text","readonly":True,"title":"Band (GKRCh)"},"")
        sctl("LoRa_air_rate",{"type":"value","readonly":True,"units":"kbps","title":"Air rate"},"")
        sctl("LoRa_power",{"type":"value","readonly":True,"units":"dBm","title":"Power"},"")
        sctl("LoRa_lbt",{"type":"text","readonly":True,"title":"LBT"},"")
        sctl("LoRa_uart",{"type":"text","readonly":True,"title":"UART"},"")
        sctl("LoRa_subpacket",{"type":"value","readonly":True,"units":"bytes","title":"Subpacket"},"")
        sctl("LoRa_rssi_ambient",{"type":"text","readonly":True,"title":"Ambient RSSI"},"")
        sctl("LoRa_rssi_byte",{"type":"text","readonly":True,"title":"Packet RSSI"},"")
        sctl("LoRa_mode",{"type":"text","readonly":True,"title":"TX mode"},"")
        sctl("LoRa_wor",{"type":"value","readonly":True,"units":"ms","title":"WOR period"},"")
        sctl("LoRa_version",{"type":"text","readonly":True,"title":"Version"},"")
        sctl("LoRa_raw",{"type":"text","readonly":True,"title":"raw (9 bytes)"},"")
        sctl("LoRa_default",{"type":"text","readonly":True,"title":"LoRa default"},LORA_DEFAULT_RAW)
        sctl("LoRa_read",{"type":"pushbutton","title":"Read"}); sctl("LoRa_apply",{"type":"pushbutton","title":"Write"})
        sctl("LoRa_status",{"type":"text","readonly":True,"title":"Result"},"")   # исход последнего Read/Write словами
        self.mqtt.subscribe("/devices/%s/controls/+/on"%sd)

        # ---- PWM8A04 Setup (RS485-1): начальная настройка модулей по проводу ----
        # Свежий модуль приходит с заводским адресом 1. Читаем регистр адреса (253) по
        # выбранному адресу, потом задаём новый. Регистры частот и скважностей читаются
        # заодно — по ним видно, что на адресе действительно PWM8A04, а не что-то другое.
        pd="pwm_setup"
        self.setname(pd,"PWM8A04 Setup (RS485-1)")
        po=[0]
        def pctl(name,meta,val=None):
            po[0]+=1; m=dict(meta,order=po[0])
            if isinstance(m.get("title"),str): m["title"]={"en":m["title"],"ru":m["title"]}
            self.pub_ctrl_meta(pd,name,m)
            if val is not None: self.mqtt.publish("/devices/%s/controls/%s"%(pd,name),str(val),retain=True)
        pctl("address",{"type":"value","readonly":False,"min":1,"max":247,"title":"Address to talk to"},
             getattr(self,"pwm_addr",1))
        pctl("read",{"type":"pushbutton","title":"Read"})
        pctl("found_address",{"type":"value","readonly":True,"title":"Address in module (reg 253)"},"")
        pctl("baud_code",{"type":"value","readonly":True,"title":"Baud code (reg 254)"},"")
        pctl("freq",{"type":"text","readonly":True,"title":"Frequency ch1/2/3"},"")
        pctl("duty",{"type":"text","readonly":True,"title":"Duty ch1/2/3"},"")
        pctl("new_address",{"type":"value","readonly":False,"min":1,"max":247,"title":"New address"},"")
        pctl("write",{"type":"pushbutton","title":"Write address"})
        pctl("status",{"type":"text","readonly":True,"title":"Result"},"")
        self.mqtt.subscribe("/devices/%s/controls/+/on"%pd)
        # ---- charging stations (chargerN dashboards) ----
        if self.chargerbus is not None:
            for i,ch in enumerate(self.chargerbus.chargers):
                cd=self.chargerbus.dev(i); co=[0]
                self.setname(cd,ch.get("name") or "Charger %d"%(i+1))
                def cctl(name,meta,val=None,_d=cd,_o=co):
                    _o[0]+=1; m=dict(meta,order=_o[0])
                    if isinstance(m.get("title"),str): m["title"]={"en":m["title"],"ru":m["title"]}
                    self.pub_ctrl_meta(_d,name,m)
                    if val is not None: self.mqtt.publish("/devices/%s/controls/%s"%(_d,name),str(val),retain=True)
                cctl("transmitter",{"type":"switch","readonly":False,"title":"Transmitter"})
                cctl("magnets",{"type":"switch","readonly":False,"title":"Hold magnets"})
                cctl("transmitter_current",{"type":"value","readonly":True,"units":"A","title":"Transmitter current"})
                cctl("charge_link",{"type":"value","readonly":True,"units":"%","title":"Coupling (charge vs setpoint)"})
                self.mqtt.subscribe("/devices/%s/controls/+/on"%cd)
        # remove dashboards of absent modules (clear retained topics)
        # Один номер корабля на двух включённых каналах = два разных радиолинка на один
        # адрес. 24.08 boat1 и boat4 оба стояли на 9 (boat1 был выключен), и это стоило
        # времени в разборе: приходилось доказывать, что говорим с тем бортом.
        seen={}
        for nm,ch in self.channels.items():
            if not ch.enabled: continue
            seen.setdefault(ch.lora["address"],[]).append(nm)
        for a,who in seen.items():
            if len(who)>1:
                print("ВНИМАНИЕ: номер корабля %d стоит сразу на каналах %s — это два разных "
                      "радиолинка на один адрес, диагностика станет неоднозначной"
                      %(a,", ".join(sorted(who))),flush=True)
        for dev in getattr(self,"absent",[]):
            self.mqtt.publish("/devices/%s/meta"%dev,"",retain=True)
            self.mqtt.publish("/devices/%s/meta/name"%dev,"",retain=True)
            for cname in BOAT_CONTROLS: self.clear_ctrl(dev,cname)
    def clear_device(self,dev,controls):   # wipe a device's retained topics so homeui drops the dashboard
        self.mqtt.publish("/devices/%s/meta"%dev,"",retain=True)
        self.mqtt.publish("/devices/%s/meta/name"%dev,"",retain=True)
        for c in controls: self.clear_ctrl(dev,c)
    def shutdown(self,*_):   # on stop (SIGTERM from systemctl): collapse boatN + ship_setup dashboards
        try:
            if self.mqtt is not None:
                for ch in self.channels.values(): self.clear_device(ch.dev,BOAT_CONTROLS)
                for n in SHIP_NUMBERS: self.clear_device("ship%d"%n,SHIP_CONTROLS)
                self.clear_device("ship_setup",SETUP_CONTROLS)
                self.clear_device("pwm_setup",PWM_SETUP_CONTROLS)
                if self.chargerbus is not None:
                    for i in range(len(self.chargerbus.chargers)): self.clear_device(self.chargerbus.dev(i),CHARGER_CONTROLS)
                time.sleep(0.6)   # let the retained clears flush before we exit
        except Exception as e: print("shutdown clear err",e,flush=True)
        os._exit(0)
    def on_connect(self,c,u,f,rc,props=None):
        # Падение внутри on_connect paho проглатывает (в журнале ни строчки), а брокер после этого
        # роняет соединение -> драйвер уходит в бесконечный цикл переподключений и молчит. Логируем сами.
        # ВАЖНО: публикация в устройство, которого нет в ACL порта 1883 (/etc/mosquitto/acl/ship.conf),
        # для MQTT 3.1.1 = отключение клиента брокером. Новое устройство -> сначала строка в ACL.
        now=time.monotonic()
        self._conn_at=[t for t in getattr(self,"_conn_at",[]) if now-t<60.0]+[now]
        if len(self._conn_at)>=4:   # брокер рвёт клиента молча, драйвер только переподключается
            devs=[ch.dev for ch in self.channels.values()]+["ship%d"%x for x in SHIP_NUMBERS]+                 ["ship_setup","ship_diag","ship_bus","pwm_setup"]
            if self.chargerbus is not None:
                devs+= [self.chargerbus.dev(i) for i in range(len(self.chargerbus.chargers))]
            print("ВНИМАНИЕ: %d переподключений к брокеру за минуту. Обычная причина — устройство, "
                  "которого нет в /etc/mosquitto/acl/ship.conf: публикация в тему вне ACL для MQTT 3.1.1 "
                  "означает отключение клиента (в журнале mosquitto это «Quota exceeded»). Нужны строки "
                  "topic readwrite /devices/<имя>/# для: %s"%(len(self._conn_at),", ".join(devs)),flush=True)
        try:
            self.declare()
            print("declare: точки %s, корабли %s"%(sorted(self.channels),SHIP_NUMBERS),flush=True)
        except Exception as e:
            import traceback; traceback.print_exc()
            print("declare err: %s"%e,flush=True)
    def on_message(self,c,u,msg):
        p=msg.topic.split("/"); dev=p[2]; ctrl=p[4]; val=msg.payload.decode(errors="ignore").strip()
        if dev=="ship_setup": self.setupq.put((ctrl,val)); return
        if dev=="pwm_setup": self.setupq.put(("pwm:"+ctrl,val)); return   # та же очередь: RS485-1 один, доступ к нему последовательный
        if dev.startswith("charger") and self.chargerbus is not None: self.chargerbus.q.put((dev,ctrl,val)); return
        if dev.startswith("ship") and dev[4:].isdigit():
            # команда с вкладки борта -> на ту точку, где борт сейчас стоит
            n=int(dev[4:])
            if n not in SHIP_NUMSET: return
            if ctrl in SHIP_CTL: self.shipq.put((n,ctrl,val)); return   # привязка к точке / занять точку
            if ctrl not in SHIP_CMD: return   # enabled и номер борта — свойства ТОЧКИ, с вкладки корабля их не меняют
            ch=self.channel_for_ship(n)
            if ch is None: print("[ship%d] команда %s=%s пропущена: борт не привязан ни к одной радиоточке"%(n,ctrl,val),flush=True); return
            ch.q.put((ctrl,val)); return
        for ch in self.channels.values():
            if ch.dev==dev: ch.q.put((ctrl,val)); return
    # ---- Ship Setup (RS485-1) handlers ----
    def setup_worker(self):
        while True:
            ctrl,val=self.setupq.get()
            try: self.handle_setup(ctrl,val)
            except Exception as e: print("setup err",ctrl,e,flush=True)
    def handle_setup(self,ctrl,val):
        if ctrl.startswith("pwm:"): return self.pwm_setup_op(ctrl[4:],val)
        sp=lambda c,v: self.mqtt.publish("/devices/ship_setup/controls/%s"%c,str(v),retain=True)
        if ctrl=="ship_number":
            try: self.setup_number=int(float(val))
            except Exception: self.setup_number=0
            sp("ship_number",self.setup_number); sp("LoRa_address",self.setup_number)
        elif ctrl=="LoRa_channel":
            try: self.setup_channel=int(float(val))
            except Exception: self.setup_channel=0
            sp("LoRa_channel",self.setup_channel); sp("LoRa_freq",round(FREQ_BASE+self.setup_channel,3)); sp("LoRa_grkch",grkch(self.setup_channel))
        elif ctrl=="LoRa_read": self.setup_op(False)           # read connected ship modem -> show all params
        elif ctrl=="LoRa_apply": self.setup_op(True)           # write number+channel (and full dump) to the connected modem
    # ---- PWM8A04 Setup (RS485-1): начальная настройка модулей по проводу ----
    def _mb(self,ser,slave,func,reg,val_or_n):
        """Одна Modbus-транзакция по проводу. func 3 = чтение val_or_n регистров, 6 = запись значения."""
        f=bytes([slave,func,(reg>>8)&0xFF,reg&0xFF,(val_or_n>>8)&0xFF,val_or_n&0xFF]); f+=crc16(f)
        ser.reset_input_buffer(); ser.write(f); ser.flush(); time.sleep(0.15)
        need=(5+2*val_or_n) if func==3 else 8
        r=ser.read(need+4)
        if func==6: return r if (len(r)>=8 and r[0]==slave and r[1]==6) else None
        if len(r)<3+2*val_or_n or r[0]!=slave or r[1]!=3: return None
        d=r[3:3+2*val_or_n]
        return [(d[i]<<8)|d[i+1] for i in range(0,len(d),2)]

    def pwm_setup_op(self,ctrl,val):
        """Свежий PWM8A04 приходит с заводским адресом 1. Регистр 253 — адрес, 254 — код
        скорости (3 = 9600). Адрес применяется сразу, и модуль при этом перезагружается,
        теряя ранее выставленные частоты и скважности — драйвер их всё равно ставит при init."""
        pp=lambda c,v: self.mqtt.publish("/devices/pwm_setup/controls/%s"%c,str(v),retain=True)
        def st(v):
            print("[pwm_setup] %s"%v,flush=True); pp("status",v)
        try: iv=int(float(val))
        except Exception: iv=0
        if ctrl=="address":
            self.pwm_addr=max(1,min(247,iv)); pp("address",self.pwm_addr); return
        if ctrl=="new_address":
            self.pwm_new=max(1,min(247,iv)); pp("new_address",self.pwm_new); return
        a=getattr(self,"pwm_addr",1)
        if ctrl=="read":
            st("читаю адрес %d..."%a)
            try:
                ser=serial.Serial(RS485,9600,8,"N",1,timeout=0.8)
                try:
                    adr=self._mb(ser,a,3,PWM_ADDR_REG,1); bd=self._mb(ser,a,3,PWM_BAUD_REG,1)
                    fq=self._mb(ser,a,3,0,3);             dt=self._mb(ser,a,3,112,3)
                finally: ser.close()
            except Exception as e: st("ERR %s"%e); return
            if adr is None:
                for c in ("found_address","baud_code","freq","duty"): pp(c,"")
                st("ERR адрес %d молчит. Модуль под питанием? Тот ли адрес? Голова в режиме сетап?"%a); return
            pp("found_address",adr[0]); pp("baud_code",bd[0] if bd else "")
            pp("freq"," / ".join(map(str,fq)) if fq else "")
            pp("duty"," / ".join(map(str,dt)) if dt else "")
            st("OK адрес=%d, скорость=%s, частоты=%s, скважности=%s"
               %(adr[0], ("%d (9600)"%bd[0] if bd and bd[0]==3 else (str(bd[0]) if bd else "?")),
                 fq if fq else "-", dt if dt else "-"))
            return
        if ctrl=="write":
            nw=getattr(self,"pwm_new",0)
            if not 1<=nw<=247: st("ERR сначала задай новый адрес (1..247)"); return
            if nw==a: st("ERR новый адрес совпадает с текущим"); return
            st("пишу адрес %d -> %d..."%(a,nw))
            try:
                ser=serial.Serial(RS485,9600,8,"N",1,timeout=0.8)
                try:
                    echo=self._mb(ser,a,6,PWM_ADDR_REG,nw)
                    time.sleep(1.0)                          # адрес применяется сразу, модуль перезагружается
                    chk=self._mb(ser,nw,3,PWM_ADDR_REG,1)    # проверяем УЖЕ на новом адресе
                    old=self._mb(ser,a,3,PWM_ADDR_REG,1)     # старый должен замолчать
                finally: ser.close()
            except Exception as e: st("ERR %s"%e); return
            if chk and chk[0]==nw:
                self.pwm_addr=nw; pp("address",nw); pp("found_address",nw)
                st("OK адрес %d принят%s (эхо %s)"%(nw,
                   ", старый замолчал" if old is None else ", но СТАРЫЙ ЕЩЁ ОТВЕЧАЕТ — на адресе %d был не один модуль"%a,
                   echo.hex() if echo else "нет"))
            else:
                st("ERR на новом адресе %d ответа нет (эхо %s). Модуль мог не принять запись."
                   %(nw, echo.hex() if echo else "нет"))
            return

    def setup_op(self,write):   # write=False -> read connected ship modem; write=True -> program number+channel (full dump) then read back
        sp=lambda c,v: self.mqtt.publish("/devices/ship_setup/controls/%s"%c,str(v),retain=True)
        def st(v):   # раньше исход уходил только в журнал: оператор жал Read и не видел НИЧЕГО
            print("[ship_setup] %s"%v,flush=True); sp("LoRa_status",v)
        st("запись..." if write else "чтение...")
        try:
            ser=serial.Serial(RS485,9600,8,"N",1,timeout=0.8)
            if write:
                # write OUR defaults for everything except address+channel (which come from the fields)
                air=AIR_CODE.get(("%g"%SETUP_DEFAULTS["air_rate"]),7); pw=PWR_CODE.get(str(int(SETUP_DEFAULTS["power"])),0)
                # reg5 = REG5_TXMODE БЕЗ бита packet-byte (0x80): на БОРТУ его включать нельзя — приёмный модем
                # борта допишет байт RSSI к нашему Modbus-ЗАПРОСУ и побьёт локальную шину pwm/УПС. packet-byte только на берегу (lora_op).
                msg=bytes([0xC0,0x00,0x08,(self.setup_number>>8)&0xFF,self.setup_number&0xFF,SPED_BASE|air,OPTION_BASE|pw,self.setup_channel&0xFF,REG5_TXMODE,0x00,0x00])
                ser.reset_input_buffer(); ser.write(msg); ser.flush(); time.sleep(0.5); ser.read(64)
            ser.reset_input_buffer(); ser.write(bytes([0xC1,0x00,0x09])); ser.flush(); time.sleep(0.4); r=ser.read(64)
            ser.close()
            if len(r)>=12 and r[0]==0xC1:
                b=r[3:12]; d=decode_e220(b); addr=d["address"]; ch=d["channel"]
                self.setup_number=addr; self.setup_channel=ch
                if d["air_rate"]!="?": self.setup_air=float(d["air_rate"])
                if d["power"]!="?": self.setup_power=int(d["power"])
                sp("ship_number",addr); sp("LoRa_address",addr); sp("LoRa_channel",ch); sp("LoRa_freq",round(FREQ_BASE+ch,3)); sp("LoRa_grkch",grkch(ch))
                sp("LoRa_air_rate",d["air_rate"]); sp("LoRa_power",d["power"]); sp("LoRa_lbt",d["lbt"])
                sp("LoRa_uart",d["uart"]); sp("LoRa_subpacket",d["subpacket"]); sp("LoRa_rssi_ambient",d["rssi_ambient"])
                sp("LoRa_rssi_byte",d["rssi_byte"]); sp("LoRa_mode",d["mode"]); sp("LoRa_wor",d["wor"]); sp("LoRa_version",d["version"]); sp("LoRa_raw",b.hex())
                st("OK №%d ch=%d %s air=%s power=%s LBT=%s ver=%s raw=%s"%(addr,ch,grkch(ch),d["air_rate"],d["power"],d["lbt"],d["version"],b.hex()))
            else:
                st("ERR нет ответа (модем корабля в режиме CONFIG?)")
        except Exception as e: st("ERR %s"%e)
    def start(self):
        # detect connected MOD modules — make dashboards only for present ones (before MQTT, pub() is a no-op while mqtt=None)
        present={}; self.absent=[]
        for n,ch in self.channels.items():
            if ch.probe(): present[n]=ch; print("module %s: present"%n,flush=True)
            else: self.absent.append(ch.dev); print("module %s: absent -> no dashboard"%n,flush=True)
        self.channels=present   # dashboards only for modules that actually responded (no fallback)
        if not present: print("no MOD modules responded — only Ship Setup will be shown",flush=True)
        self.setup_mqtt(); time.sleep(1.0)
        signal.signal(signal.SIGTERM,self.shutdown); signal.signal(signal.SIGINT,self.shutdown)   # clear dashboards on stop
        for ch in self.channels.values(): ch.start()
        threading.Thread(target=self.setup_worker,daemon=True).start()
        threading.Thread(target=self.ship_worker,daemon=True).start()
        if self.chargerbus is not None: self.chargerbus.start(); print("charger bus: %d station(s)"%len(self.chargerbus.chargers),flush=True)
        self.save()   # привязки бортов к точкам могли вывестись из адресов при старте -> закрепляем их в файле состояния
        while True: time.sleep(1)

if __name__=="__main__": Driver().start()
