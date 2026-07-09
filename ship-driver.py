#!/usr/bin/env python3
# ship-driver.py — custom multi-channel ship-bus driver.
# 4 equivalent LoRa transmitters (MOD1-4 = /dev/ttyMOD1..4). Each channel independently owns its
# port and runs the SAME ship logic against one active ship (Modbus addrs identical on every ship:
# UPS=10, pwm=11/12/13; ships are told apart by LoRa address -> change the channel modem address to
# switch ship). Per channel: Modbus RTU + inline DFPlayer + mode state machine + poll schedule +
# simple charge-thermal. Tunables live in /etc/ship-driver.conf (defaults below if absent).
# boatN MQTT device = operational API + visualisation (driven by external software); LoRa config
# lives in the conf, only LoRa_address is live on boatN (writes the modem immediately, applying the
# conf channel/air/power + the new address). Wired ship pre-config stays on the "Ship Setup" dashboard.
import time, threading, queue, json, os, re, signal, socket
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
RS485="/dev/ttyRS485-1"   # Ship Setup dashboard: wired ship LoRa-modem config (ship modem in config mode by its switch)
STATE_FILE="/etc/ship-driver-state.json"   # persist per-channel enabled across reboot
CONF_FILE="/etc/ship-driver.conf"          # tunable settings (see DEFAULTS)

PWM_SLAVES=[11,12,13]; FREQ_REG={1:0,2:1,3:2}; DUTY_REG={1:112,2:113,3:114}
ALL_CH=[(s,c) for s in PWM_SLAVES for c in (1,2,3)]
UPS=10; UPS_VIN=2; UPS_CUR=5; UPS_CHG=8; UPS_TEMP=9; UPS_CHG_SETPOINT=18   # UPS_VIN=2: input voltage (x0.001)
# motor/light -> (pwm slave, channel) wiring is loaded from the conf "wiring" section below

SEARCH="SEARCH"; SAIL="SAILING"; CHARGE="CHARGING"; IDLE="IDLE"; SERVICE="SERVICE"; OFF="OFF"

# ---- tunable defaults (overridden per-key by /etc/ship-driver.conf) ----
DEFAULTS={
 "main":{
 "baud":9600, "resp_timeout_s":0.8,
 "charge":{"full_ma":2000,"low_ma":600,"t_limit_c":50.0,"t_restore_c":48.0},
 "init":{"freq":400,"motor":40,"light":0},
 "limits":{"motor_min":40,"motor_max":80,"mp3_track_max":15},
 "rates":{
   "CHARGING":{"current":5,"temp":10,"charge":60,"motors":60,"lights":60},
   "SAILING": {"current":5,"temp":300,"charge":300,"motors":60,"lights":60},
   "IDLE":    {"current":5,"temp":300,"charge":300,"motors":300,"lights":300},
   "sail_timeout_s":30.0, "offline_fails":2,
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
   "list":[],   # per ship: {"address":N,"channel":C,"motors":{...},"lights":{...}}
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
CHG_FULL=M["charge"]["full_ma"]; CHG_LOW=M["charge"]["low_ma"]
T_LIMIT=M["charge"]["t_limit_c"]; T_RESTORE=M["charge"]["t_restore_c"]
INIT_FREQ=M["init"]["freq"]; INIT_MOTOR=M["init"]["motor"]; INIT_LIGHT=M["init"]["light"]
MOTOR_MIN=M["limits"]["motor_min"]; MOTOR_MAX=M["limits"]["motor_max"]
MP3_TRACK_MAX=M["limits"]["mp3_track_max"]; MP3_VOL_MAX=30   # max volume hardcoded
RATES={CHARGE:M["rates"]["CHARGING"], SAIL:M["rates"]["SAILING"], IDLE:M["rates"]["IDLE"]}
SAIL_TIMEOUT=M["rates"]["sail_timeout_s"]; OFFLINE_FAILS=M["rates"]["offline_fails"]
SEARCH_PERIOD=M["rates"]["search_period"]; SERVICE_PERIOD=M["rates"]["service_period"]
FREQ_BASE=850.125; SPED_BASE=0x60; OPTION_BASE=0x60   # band base + E220 SPED/OPTION base bytes (UART 9600, subpkt128, RSSI) — fixed
REG5_TXMODE=0x03   # E220 reg 0x05: transparent, LBT OFF, WOR=3 (match working .6 modems; some modules ship with LBT on)
REG_TAIL=[0x00,0x00]   # regs 0x06,0x07 (CRYPT high/low) — .6 reference; written so the full dump matches
LORA_DEFAULT_RAW="xxxx6760xx03000010"   # .6 reference 9-byte dump (x = variable addr/channel; 67/60 SPED/OPTION; 03 reg5; 0000 crypt; 10 version)
LORA_PLAN=C["lora"]   # {mod1..4: {channel,air_rate,address,power}} (top-level)
SETUP_DEFAULTS={"channel":14,"air_rate":62.5,"address":3,"power":22}   # Ship Setup dashboard defaults (hardcoded)
GRKCH_CHANNELS={14,16,17,19}   # ГКРЧ-allowed LoRa channels
def grkch(ch):
    try: return "✓ in band (GKRCh)" if int(ch) in GRKCH_CHANNELS else "⚠ out of band"
    except Exception: return "?"
ADDR_MAX=65535   # ship_number (= LoRa address) control max (hardcoded)
ENABLED_AT_START=set(n for n,v in M["enabled_at_start"].items() if v)
def parse_wiring(w):   # -> (motors[(name,slave,ch)], motor_map{name:(slave,ch)}, lights[(name,slave,ch)] in conf order)
    motors=[(n,m["slave"],m["channel"]) for n,m in w["motors"].items()]
    lights=[(n,l["slave"],l["channel"]) for n,l in w["lights"].items()]
    return motors,{n:(s,c) for n,s,c in motors},lights
SD=C["ships"]; SHIP_DEFAULT=SD["default"]; SHIP_LIST=SD.get("list",[])
DEFAULT_WIRING=parse_wiring(SHIP_DEFAULT)                       # fallback wiring (address not in list)
DEFAULT_AIR_RATE=SHIP_DEFAULT["air_rate"]; DEFAULT_POWER=SHIP_DEFAULT["power"]   # shared for all ships
SHIP_WIRING={int(s["address"]):parse_wiring(s) for s in SHIP_LIST}              # LoRa address -> wiring
SHIP_RADIO={int(s["address"]):{"channel":s["channel"],"air_rate":DEFAULT_AIR_RATE,"power":DEFAULT_POWER} for s in SHIP_LIST}  # number -> radio for Ship Setup write
def wiring_for(addr):
    try: return SHIP_WIRING.get(int(addr),DEFAULT_WIRING)
    except Exception: return DEFAULT_WIRING
# ---- charging stations (Modbus-RTU-over-TCP via a serial-gateway) ----
CHG_LIST=C.get("chargers",[])
CHG_PERIOD=3.0            # charger bus poll period, s
MAI_VOLT_SCALE=1e-6       # WB-MAI6 input-voltage raw (s32) -> volts
MAI_IN0=0x0700           # WB-MAI6 fw2.4: IN n voltage input reg = MAI_IN0 + 2*(n-1), s32
MRM_COIL0=0; MRM_STATE0=96   # WB-MRM2-mini: K ch -> coil (ch-1); real contact state -> discrete 96+(ch-1)
CHARGER_CONTROLS=["transmitter","magnets","transmitter_current"]
# control set is the SAME on every ship -> names/count fixed (from default), only the register mapping varies per ship
MOTOR_NAMES=[n for n,_,_ in DEFAULT_WIRING[0]]
LIGHT_NAMES=[n for n,_,_ in DEFAULT_WIRING[2]]
NLIGHTS=len(LIGHT_NAMES)
_MT={"front_right":"Front Right","back_right":"Back Right","front_left":"Front Left","back_left":"Back Left"}
MOTOR_TITLE={n:_MT.get(n,n.replace("_"," ").title()) for n in MOTOR_NAMES}
_LT={"nav_lights":"Navigation lights","morse_lamp":"Morse signal lamp","deck_lights":"Deck lights","cabin_light1":"Cabin light 1","cabin_light2":"Cabin light 2"}
LIGHT_TITLE={n:_LT.get(n,n.replace("_"," ").title()) for n in LIGHT_NAMES}   # dashboard titles (nautical, English)
BOAT_CONTROLS=["enabled","mode","battery_current","battery_temperature","charge_level","input_voltage"]+MOTOR_NAMES+LIGHT_NAMES+["mp3_track","mp3_volume","ship_number"]
BOAT_EXTRA=[c for c in BOAT_CONTROLS if c not in ("enabled","mode","ship_number")]   # shown only while polling (online); removed in SEARCH/OFF
SETUP_CONTROLS=["ship_number","LoRa_address","LoRa_channel","LoRa_freq","LoRa_grkch","LoRa_air_rate","LoRa_power","LoRa_lbt","LoRa_uart","LoRa_subpacket","LoRa_rssi_ambient","LoRa_rssi_byte","LoRa_mode","LoRa_wor","LoRa_version","LoRa_raw","LoRa_default","LoRa_read","LoRa_apply"]   # ship_setup dashboard controls (for teardown on shutdown)

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
        self.chg_setpoint=CHG_FULL; self.tele={}
        self.motor={n:0 for n in MOTOR_NAMES}; self.light={n:0 for n in LIGHT_NAMES}
        self.due={}
        self.lora=dict(LORA_PLAN[name])   # {channel,air_rate,address,power} from conf; refreshed by reading the modem at start
        self.apply_wiring()               # pick motor/light register map for this ship (by LoRa address)
    def apply_wiring(self):
        self.motors,self.motor_map,self.lights=wiring_for(self.lora["address"])
        self.light_map={n:(s,c) for n,s,c in self.lights}

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
        self.ser.reset_input_buffer(); self.ser.write(req); self.ser.flush()
        end=time.monotonic()+RESP_TO; buf=b""
        while time.monotonic()<end and len(buf)<n:
            c=self.ser.read(n-len(buf))
            if c: buf+=c
        return buf
    def read_regs(self,slave,func,addr,n):
        req=bytes([slave,func,(addr>>8)&0xFF,addr&0xFF,(n>>8)&0xFF,n&0xFF]); req+=crc16(req)
        r=self._txn(req,5+2*n)
        if len(r)>=5+2*n and r[0]==slave and r[1]==func and r[2]==2*n and crc16(r[:3+2*n])==r[3+2*n:5+2*n]:
            return [ (r[3+2*i]<<8)|r[4+2*i] for i in range(n) ]
        return None
    def write_reg(self,slave,addr,val):
        val&=0xFFFF; req=bytes([slave,6,(addr>>8)&0xFF,addr&0xFF,(val>>8)&0xFF,val&0xFF]); req+=crc16(req)
        r=self._txn(req,8); return len(r)>=8 and r[0]==slave and r[1]==6
    def send_mp3(self,fr):
        self.ser.reset_input_buffer(); self.ser.write(fr); self.ser.flush(); time.sleep(0.25)

    # ---- MQTT helpers ----
    def pub(self,ctrl,val):
        if self.drv.mqtt is not None: self.drv.mqtt.publish("/devices/%s/controls/%s"%(self.dev,ctrl),str(val),retain=True)
    def puberr(self,ctrl,err):   # WB convention: /controls/<c>/meta/error = "r" (read error) -> homeui greys/colours it; "" = ok
        if self.drv.mqtt is not None: self.drv.mqtt.publish("/devices/%s/controls/%s/meta/error"%(self.dev,ctrl),err,retain=True)

    # ---- command handling (this thread) ----
    def handle(self,ctrl,val):
        try: fv=float(val)
        except Exception: fv=0.0
        iv=int(fv); is_cmd=True
        if ctrl=="enabled":
            self.enabled=(val in ("1","true","on")); is_cmd=False
            if not self.enabled: self.online=False; self.set_mode(OFF); self.close()
        elif ctrl in self.motor_map and self.online:
            s,c=self.motor_map[ctrl]; self.motor[ctrl]=max(MOTOR_MIN,min(MOTOR_MAX,iv)); self.write_reg(s,DUTY_REG[c],self.motor[ctrl]); self.pub(ctrl,self.motor[ctrl])
        elif ctrl in self.light_map and self.online:
            s,c=self.light_map[ctrl]; self.light[ctrl]=max(0,min(100,iv)); self.write_reg(s,DUTY_REG[c],self.light[ctrl]); self.pub(ctrl,self.light[ctrl])
        elif ctrl=="mp3_track" and self.online:
            is_cmd=False; iv=max(0,min(MP3_TRACK_MAX,iv)); self.send_mp3(mp3_frame(MP3["stop"]) if iv<=0 else mp3_frame(MP3["play"],iv)); self.pub("mp3_track",iv)
        elif ctrl=="mp3_volume" and self.online:
            is_cmd=False; v=max(0,min(MP3_VOL_MAX,iv)); self.send_mp3(mp3_frame(MP3["vol"],v)); self.pub("mp3_volume",v)
        elif ctrl=="ship_number":
            # ship number = LoRa address. Persist FIRST (survives reboot even if the slow modem write is interrupted), then write modem.
            is_cmd=False; self.lora["address"]=iv; self.apply_wiring(); self.pub(ctrl,iv); self.drv.save(); self.lora_op(self.lora)
        else: is_cmd=False
        if ctrl=="enabled": self.drv.save()
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
                des=bytes([(int(target["address"])>>8)&0xFF,int(target["address"])&0xFF,SPED_BASE|air,OPTION_BASE|pw,int(target["channel"])&0xFF,REG5_TXMODE]+REG_TAIL)
                print("[%s] modem raw=%s want=%s"%(self.name,b.hex(),des.hex()),flush=True)
                if bytes(b[0:len(des)])!=des:   # any byte differs -> write the whole dump
                    print("[%s] writing config to modem (all bytes)"%self.name,flush=True)
                    self.ser.reset_input_buffer(); self.ser.write(bytes([0xC0,0x00,len(des)])+des); self.ser.flush(); time.sleep(0.5); self.ser.read(64)
                    self.ser.reset_input_buffer(); self.ser.write(bytes([0xC1,0x00,0x08])); self.ser.flush(); time.sleep(0.4); r2=self.ser.read(64)
                    if len(r2)>=11 and r2[0]==0xC1: b=r2[3:11]; print("[%s] modem after write raw=%s"%(self.name,b.hex()),flush=True)
                self.lora={"channel":b[4],"air_rate":float(AIR_NAME.get(b[2]&7,"62.5")),
                           "address":(b[0]<<8)|b[1],"power":int(PWR_NAME.get(b[3]&3,"22"))}
                self.apply_wiring(); self.pub("ship_number",self.lora["address"])
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
        for s,c in ALL_CH: self.write_reg(s,DUTY_REG[c],0)          # 1) power (duty) off on every channel first
        for s,c in ALL_CH: self.write_reg(s,FREQ_REG[c],INIT_FREQ)  # 2) then pwm frequency = 400
        for n,s,c in self.motors: self.motor[n]=INIT_MOTOR; self.write_reg(s,DUTY_REG[c],INIT_MOTOR)  # 3) then motors to idle (40)
        for n,s,c in self.lights: self.light[n]=INIT_LIGHT; self.write_reg(s,DUTY_REG[c],INIT_LIGHT)
        for n,s,c in self.motors: self.pub(n,self.motor[n])
        for n,s,c in self.lights: self.pub(n,self.light[n])
    def poll_current(self):
        r=self.read_regs(UPS,4,UPS_VIN,4)   # one block: regs 2..5 = Vin, Vout, Vbat, Ibat (input voltage rides along at the 5 s current rate)
        if r is None: self.puberr("battery_current","r"); self.puberr("input_voltage","r"); return False
        self.tele["current"]=s16(r[3])*0.001; self.pub("battery_current",round(self.tele["current"],3)); self.puberr("battery_current","")
        self.pub("input_voltage",round(r[0]*0.001,2)); self.puberr("input_voltage","")
        return True
    def pwm_alive(self):   # ship reachable via pwm even when UPS is off — probe each pwm8a04 frequency register
        for s in PWM_SLAVES:
            if self.read_regs(s,3,FREQ_REG[1],1) is not None: return True
        return False
    def poll_temp(self):
        r=self.read_regs(UPS,4,UPS_TEMP,1)
        if r is None: self.puberr("battery_temperature","r"); return False
        self.tele["temp"]=s16(r[0])*0.01; self.pub("battery_temperature",round(self.tele["temp"],2)); self.puberr("battery_temperature",""); return True
    def poll_charge(self):
        r=self.read_regs(UPS,4,UPS_CHG,1)
        if r is None: self.puberr("charge_level","r"); return False
        self.pub("charge_level",round(r[0]*0.01,1)); self.puberr("charge_level",""); return True
    def poll_motors(self):
        ok=True
        for n,s,c in self.motors:
            r=self.read_regs(s,3,DUTY_REG[c],1)
            if r is None: ok=False; self.puberr(n,"r")
            else: self.pub(n,r[0]); self.puberr(n,"")
        return ok
    def poll_lights(self):
        ok=True
        for n,s,c in self.lights:
            r=self.read_regs(s,3,DUTY_REG[c],1)
            if r is None: ok=False; self.puberr(n,"r")
            else: self.pub(n,r[0]); self.puberr(n,"")
        return ok
    GROUPS={"current":"poll_current","temp":"poll_temp","charge":"poll_charge","motors":"poll_motors","lights":"poll_lights"}
    def thermal(self):
        t=self.tele.get("temp")
        if t is None: return
        new=self.chg_setpoint
        if t>T_LIMIT: new=CHG_LOW
        elif t<T_RESTORE: new=CHG_FULL
        if new!=self.chg_setpoint:
            self.chg_setpoint=new; self.write_reg(UPS,UPS_CHG_SETPOINT,new)
    def decide(self):
        if not self.online: return SEARCH
        if time.monotonic()-self.last_cmd < SAIL_TIMEOUT: return SAIL
        return CHARGE if self.tele.get("current",0)>0 else IDLE
    def set_mode(self,m):
        if m!=self.mode:
            self.mode=m; self.pub("mode",m)
            want_full=(m not in (SEARCH,OFF))   # polling -> show full dashboard; not polling -> only enabled+mode
            if want_full!=self.declared_full: self.drv.boat_controls(self,want_full)
            if m==CHARGE:
                self.chg_setpoint=CHG_FULL; self.write_reg(UPS,UPS_CHG_SETPOINT,CHG_FULL)
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
                    self.fails=0; self.online=True; self.due={}   # due={} -> re-poll all groups at once on (re)connect
                    self.drv.boat_controls(self,True)             # restore full dashboard before init/poll fills values
                    self.init_ship(); self.last_cmd=0.0; self.set_mode(self.decide())
                else: time.sleep(SEARCH_PERIOD)
                continue
            self.set_mode(self.decide())
            did=False
            for g,per in RATES[self.mode].items():
                if now>=self.due.get(g,0):
                    ok=getattr(self,self.GROUPS[g])(); self.due[g]=now+per; did=True
                    if g=="current":
                        if ok or self.pwm_alive(): self.fails=0   # UPS may be off; ship still alive if any pwm8a04 answers
                        else:
                            self.fails+=1
                            if self.fails>=OFFLINE_FAILS: self.online=False; self.set_mode(SEARCH)
            if self.mode==CHARGE: self.thermal()
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
                    for ctrl in ("transmitter","magnets"):
                        out=ch.get(ctrl)
                        if not out: continue
                        st=self.relay_state(ch,out)
                        if st is None: self.puberr(dev,ctrl,"r")
                        else: self.pub(dev,ctrl,1 if st else 0); self.puberr(dev,ctrl,"")
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
        self.setup_number=int(SETUP_DEFAULTS["address"]); self.setup_channel=int(SETUP_DEFAULTS["channel"])   # Ship Setup editable: number + channel
        self.setup_air=float(SETUP_DEFAULTS["air_rate"]); self.setup_power=int(SETUP_DEFAULTS["power"])        # preserved from last read, used on write
        self.setupq=queue.Queue()
        self.chargerbus=ChargerBus(self,[dict(c) for c in CHG_LIST]) if CHG_LIST else None
    def load(self):
        try: return json.load(open(STATE_FILE))
        except Exception: return {}
    def save(self):
        try: json.dump({n:{"enabled":c.enabled,"ship_number":c.lora["address"]} for n,c in self.channels.items()},open(STATE_FILE,"w"))
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
    def boat_controls(self,ch,full):
        # full=True: publish the whole dashboard; full=False: keep only enabled+mode, remove the rest
        # (the "extra" controls are telemetry/commands that only make sense while the channel polls a live ship).
        if self.mqtt is None: return
        d=ch.dev; o=[0]
        def ctl(name,meta,val=None):
            o[0]+=1; m=dict(meta,order=o[0])
            if isinstance(m.get("title"),str): m["title"]={"en":m["title"],"ru":m["title"]}   # homeui needs title as {lang:...} object
            self.mqtt.publish("/devices/%s/controls/%s/meta"%(d,name),json.dumps(m),retain=True)
            if val is not None: self.mqtt.publish("/devices/%s/controls/%s"%(d,name),str(val),retain=True)
        ctl("enabled",{"type":"switch","readonly":False,"title":"Enabled"},1 if ch.enabled else 0)
        ctl("mode",{"type":"text","readonly":True,"title":"Mode"})
        ctl("ship_number",{"type":"value","readonly":False,"min":0,"max":ADDR_MAX,"title":"Ship number"},ch.lora["address"])   # always visible (set ship even while searching)
        if full:
            for nm,u,t in (("battery_current","A","Battery current"),("battery_temperature","°C","Battery temperature"),("charge_level","%","Charge level"),("input_voltage","V","Input voltage")):
                ctl(nm,{"type":"value","readonly":True,"units":u,"title":t})
            for n2 in MOTOR_NAMES: ctl(n2,{"type":"range","readonly":False,"min":MOTOR_MIN,"max":MOTOR_MAX,"title":MOTOR_TITLE[n2]})
            for n in LIGHT_NAMES: ctl(n,{"type":"range","readonly":False,"min":0,"max":100,"title":LIGHT_TITLE.get(n,n)})
            ctl("mp3_track",{"type":"range","readonly":False,"min":0,"max":MP3_TRACK_MAX,"title":"Audio track"})
            ctl("mp3_volume",{"type":"range","readonly":False,"min":0,"max":MP3_VOL_MAX,"title":"Volume"})
        else:
            for c in BOAT_EXTRA:   # remove control: clear its meta, error flag and value
                for sub in ("/meta","/meta/error",""):
                    self.mqtt.publish("/devices/%s/controls/%s%s"%(d,c,sub),"",retain=True)
        ch.declared_full=full
    def declare(self):
        for n,ch in self.channels.items():
            self.setname(ch.dev,"boat%s (channel %s)"%(n[-1],ch.lora["channel"]))
            self.boat_controls(ch, ch.online and ch.enabled)   # collapsed until the channel actually polls a ship
            self.mqtt.subscribe("/devices/%s/controls/+/on"%ch.dev)
        # ---- Ship Setup dashboard (RS485-1 wired config) — unchanged ----
        sd="ship_setup"
        self.setname(sd,"Ship Setup (RS485-1)")
        so=[0]
        def sctl(name,meta,val=None):
            so[0]+=1; m=dict(meta,order=so[0])
            if isinstance(m.get("title"),str): m["title"]={"en":m["title"],"ru":m["title"]}   # homeui needs title as {lang:...} object
            self.mqtt.publish("/devices/%s/controls/%s/meta"%(sd,name),json.dumps(m),retain=True)
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
        self.mqtt.subscribe("/devices/%s/controls/+/on"%sd)
        # ---- charging stations (chargerN dashboards) ----
        if self.chargerbus is not None:
            for i,ch in enumerate(self.chargerbus.chargers):
                cd=self.chargerbus.dev(i); co=[0]
                self.setname(cd,ch.get("name") or "Charger %d"%(i+1))
                def cctl(name,meta,val=None,_d=cd,_o=co):
                    _o[0]+=1; m=dict(meta,order=_o[0])
                    if isinstance(m.get("title"),str): m["title"]={"en":m["title"],"ru":m["title"]}
                    self.mqtt.publish("/devices/%s/controls/%s/meta"%(_d,name),json.dumps(m),retain=True)
                    if val is not None: self.mqtt.publish("/devices/%s/controls/%s"%(_d,name),str(val),retain=True)
                cctl("transmitter",{"type":"switch","readonly":False,"title":"Transmitter"})
                cctl("magnets",{"type":"switch","readonly":False,"title":"Hold magnets"})
                cctl("transmitter_current",{"type":"value","readonly":True,"units":"A","title":"Transmitter current"})
                self.mqtt.subscribe("/devices/%s/controls/+/on"%cd)
        # remove dashboards of absent modules (clear retained topics)
        for dev in getattr(self,"absent",[]):
            self.mqtt.publish("/devices/%s/meta"%dev,"",retain=True)
            self.mqtt.publish("/devices/%s/meta/name"%dev,"",retain=True)
            for cname in BOAT_CONTROLS:
                self.mqtt.publish("/devices/%s/controls/%s/meta"%(dev,cname),"",retain=True)
                self.mqtt.publish("/devices/%s/controls/%s"%(dev,cname),"",retain=True)
    def clear_device(self,dev,controls):   # wipe a device's retained topics so homeui drops the dashboard
        self.mqtt.publish("/devices/%s/meta"%dev,"",retain=True)
        self.mqtt.publish("/devices/%s/meta/name"%dev,"",retain=True)
        for c in controls:
            self.mqtt.publish("/devices/%s/controls/%s/meta"%(dev,c),"",retain=True)
            self.mqtt.publish("/devices/%s/controls/%s/meta/error"%(dev,c),"",retain=True)
            self.mqtt.publish("/devices/%s/controls/%s"%(dev,c),"",retain=True)
    def shutdown(self,*_):   # on stop (SIGTERM from systemctl): collapse boatN + ship_setup dashboards
        try:
            if self.mqtt is not None:
                for ch in self.channels.values(): self.clear_device(ch.dev,BOAT_CONTROLS)
                self.clear_device("ship_setup",SETUP_CONTROLS)
                if self.chargerbus is not None:
                    for i in range(len(self.chargerbus.chargers)): self.clear_device(self.chargerbus.dev(i),CHARGER_CONTROLS)
                time.sleep(0.6)   # let the retained clears flush before we exit
        except Exception as e: print("shutdown clear err",e,flush=True)
        os._exit(0)
    def on_connect(self,c,u,f,rc,props=None): self.declare()
    def on_message(self,c,u,msg):
        p=msg.topic.split("/"); dev=p[2]; ctrl=p[4]; val=msg.payload.decode(errors="ignore").strip()
        if dev=="ship_setup": self.setupq.put((ctrl,val)); return
        if dev.startswith("charger") and self.chargerbus is not None: self.chargerbus.q.put((dev,ctrl,val)); return
        for ch in self.channels.values():
            if ch.dev==dev: ch.q.put((ctrl,val)); return
    # ---- Ship Setup (RS485-1) handlers ----
    def setup_worker(self):
        while True:
            ctrl,val=self.setupq.get()
            try: self.handle_setup(ctrl,val)
            except Exception as e: print("setup err",ctrl,e,flush=True)
    def handle_setup(self,ctrl,val):
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
    def setup_op(self,write):   # write=False -> read connected ship modem; write=True -> program number+channel (full dump) then read back
        st=lambda v: print("[ship_setup] %s"%v,flush=True)
        sp=lambda c,v: self.mqtt.publish("/devices/ship_setup/controls/%s"%c,str(v),retain=True)
        st("запись..." if write else "чтение...")
        try:
            ser=serial.Serial(RS485,9600,8,"N",1,timeout=0.8)
            if write:
                # write OUR defaults for everything except address+channel (which come from the fields)
                air=AIR_CODE.get(("%g"%SETUP_DEFAULTS["air_rate"]),7); pw=PWR_CODE.get(str(int(SETUP_DEFAULTS["power"])),0)
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
        if self.chargerbus is not None: self.chargerbus.start(); print("charger bus: %d station(s)"%len(self.chargerbus.chargers),flush=True)
        while True: time.sleep(1)

if __name__=="__main__": Driver().start()
