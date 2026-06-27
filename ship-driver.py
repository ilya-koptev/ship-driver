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
import time, threading, queue, json, os, re
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
UPS=10; UPS_CUR=5; UPS_CHG=8; UPS_TEMP=9; UPS_CHG_SETPOINT=18
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
     "lights":{"light1":{"slave":11,"channel":3},"light2":{"slave":12,"channel":3},"light3":{"slave":13,"channel":2},
               "light4":{"slave":13,"channel":1},"light5":{"slave":13,"channel":3}}},
   "list":[],   # per ship: {"address":N,"channel":C,"motors":{...},"lights":{...}}
 },
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
LORA_PLAN=C["lora"]   # {mod1..4: {channel,air_rate,address,power}} (top-level)
SETUP_DEFAULTS={"channel":14,"air_rate":62.5,"address":3,"power":22}   # Ship Setup dashboard defaults (hardcoded)
ADDR_MAX=65535   # ship_number (= LoRa address) control max (hardcoded)
ENABLED_AT_START=set(n for n,v in M["enabled_at_start"].items() if v)
def parse_wiring(w):   # -> (motors[(name,slave,ch)], motor_map{name:(slave,ch)}, lights[(slave,ch)] in light1..N order)
    motors=[(n,m["slave"],m["channel"]) for n,m in w["motors"].items()]
    lights=[(w["lights"][k]["slave"],w["lights"][k]["channel"]) for k in sorted(w["lights"],key=lambda x:int(x[5:]))]
    return motors,{n:(s,c) for n,s,c in motors},lights
SD=C["ships"]; SHIP_DEFAULT=SD["default"]; SHIP_LIST=SD.get("list",[])
DEFAULT_WIRING=parse_wiring(SHIP_DEFAULT)                       # fallback wiring (address not in list)
DEFAULT_AIR_RATE=SHIP_DEFAULT["air_rate"]; DEFAULT_POWER=SHIP_DEFAULT["power"]   # shared for all ships
SHIP_WIRING={int(s["address"]):parse_wiring(s) for s in SHIP_LIST}              # LoRa address -> wiring
SHIP_RADIO={int(s["address"]):{"channel":s["channel"],"air_rate":DEFAULT_AIR_RATE,"power":DEFAULT_POWER} for s in SHIP_LIST}  # number -> radio for Ship Setup write
def wiring_for(addr):
    try: return SHIP_WIRING.get(int(addr),DEFAULT_WIRING)
    except Exception: return DEFAULT_WIRING
# control set is the SAME on every ship -> names/count fixed (from default), only the register mapping varies per ship
MOTOR_NAMES=[n for n,_,_ in DEFAULT_WIRING[0]]
NLIGHTS=len(DEFAULT_WIRING[2])
_MT={"front_right":"Front Right","back_right":"Back Right","front_left":"Front Left","back_left":"Back Left"}
MOTOR_TITLE={n:_MT.get(n,n.replace("_"," ").title()) for n in MOTOR_NAMES}
LIGHT_TITLE={4:"Ходовые огни",5:"Внутренняя подсветка"}   # dashboard titles (others default to "Light N")
BOAT_CONTROLS=["enabled","mode","battery_current","battery_temperature","charge_level"]+MOTOR_NAMES+["light%d"%i for i in range(1,NLIGHTS+1)]+["mp3_track","mp3_volume","ship_number"]

MP3={"play":0x08,"vol":0x06,"pause":0x0E,"resume":0x0D,"stop":0x16,"next":0x01,"prev":0x02}
def mp3_frame(cmd,param=0): return bytes([0x7E,0xFF,0x06,cmd,0x00,(param>>8)&0xFF,param&0xFF,0xEF])
AIR_CODE={"2.4":2,"4.8":3,"9.6":4,"19.2":5,"38.4":6,"62.5":7}
AIR_NAME={0:"2.4",1:"2.4",2:"2.4",3:"4.8",4:"9.6",5:"19.2",6:"38.4",7:"62.5"}
PWR_CODE={"22":0,"17":1,"13":2,"10":3}; PWR_NAME={0:"22",1:"17",2:"13",3:"10"}

def crc16(d):
    c=0xFFFF
    for b in d:
        c^=b
        for _ in range(8): c=(c>>1)^0xA001 if c&1 else c>>1
    return bytes([c&0xFF,(c>>8)&0xFF])
def s16(v): return v-65536 if v>=32768 else v
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
        self.last_cmd=0.0; self.lora_read=False
        self.chg_setpoint=CHG_FULL; self.tele={}
        self.motor={n:0 for n in MOTOR_NAMES}; self.light={i:0 for i in range(1,NLIGHTS+1)}
        self.due={}
        self.lora=dict(LORA_PLAN[name])   # {channel,air_rate,address,power} from conf; refreshed by reading the modem at start
        self.apply_wiring()               # pick motor/light register map for this ship (by LoRa address)
    def apply_wiring(self):
        self.motors,self.motor_map,self.lights=wiring_for(self.lora["address"])

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
        elif ctrl.startswith("light") and self.online and ctrl[5:].isdigit() and 0<int(ctrl[5:])<=len(self.lights):
            i=int(ctrl[5:]); s,c=self.lights[i-1]; self.light[i]=max(0,min(100,iv)); self.write_reg(s,DUTY_REG[c],self.light[i]); self.pub(ctrl,self.light[i])
        elif ctrl=="mp3_track" and self.online:
            is_cmd=False; iv=max(0,min(MP3_TRACK_MAX,iv)); self.send_mp3(mp3_frame(MP3["stop"]) if iv<=0 else mp3_frame(MP3["play"],iv)); self.pub("mp3_track",iv)
        elif ctrl=="mp3_volume" and self.online:
            is_cmd=False; v=max(0,min(MP3_VOL_MAX,iv)); self.send_mp3(mp3_frame(MP3["vol"],v)); self.pub("mp3_volume",v)
        elif ctrl=="ship_number":
            is_cmd=False; self.lora["address"]=iv; self.pub(ctrl,iv); self.lora_op(self.lora)  # ship number = LoRa address; write modem immediately (ship switch)
        else: is_cmd=False
        if ctrl in ("enabled","ship_number"): self.drv.save()
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
        for s,c in ALL_CH: self.write_reg(s,FREQ_REG[c],INIT_FREQ)
        for n,s,c in self.motors: self.motor[n]=INIT_MOTOR; self.write_reg(s,DUTY_REG[c],INIT_MOTOR)
        for i,(s,c) in enumerate(self.lights,1): self.light[i]=INIT_LIGHT; self.write_reg(s,DUTY_REG[c],INIT_LIGHT)
        for n,s,c in self.motors: self.pub(n,self.motor[n])
        for i in range(1,len(self.lights)+1): self.pub("light%d"%i,self.light[i])
    def poll_current(self):
        r=self.read_regs(UPS,4,UPS_CUR,1)
        if r is None: return False
        self.tele["current"]=s16(r[0])*0.001; self.pub("battery_current",round(self.tele["current"],3)); return True
    def poll_temp(self):
        r=self.read_regs(UPS,4,UPS_TEMP,1)
        if r is None: return False
        self.tele["temp"]=s16(r[0])*0.01; self.pub("battery_temperature",round(self.tele["temp"],2)); return True
    def poll_charge(self):
        r=self.read_regs(UPS,4,UPS_CHG,1)
        if r is None: return False
        self.pub("charge_level",round(r[0]*0.01,1)); return True
    def poll_motors(self):
        ok=True
        for n,s,c in self.motors:
            r=self.read_regs(s,3,DUTY_REG[c],1)
            if r is None: ok=False
            else: self.pub(n,r[0])
        return ok
    def poll_lights(self):
        ok=True
        for i,(s,c) in enumerate(self.lights,1):
            r=self.read_regs(s,3,DUTY_REG[c],1)
            if r is None: ok=False
            else: self.pub("light%d"%i,r[0])
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
                if self.poll_current(): self.fails=0; self.online=True; self.init_ship(); self.last_cmd=0.0; self.set_mode(self.decide())
                else: time.sleep(SEARCH_PERIOD)
                continue
            self.set_mode(self.decide())
            did=False
            for g,per in RATES[self.mode].items():
                if now>=self.due.get(g,0):
                    ok=getattr(self,self.GROUPS[g])(); self.due[g]=now+per; did=True
                    if g=="current":
                        if ok: self.fails=0
                        else:
                            self.fails+=1
                            if self.fails>=OFFLINE_FAILS: self.online=False; self.set_mode(SEARCH)
            if self.mode==CHARGE: self.thermal()
            if not did: time.sleep(0.2)

class Driver:
    def __init__(self):
        st=self.load()
        self.channels={}
        for n,(tty,g) in CHANNELS.items():
            en=st.get(n,{}).get("enabled", n in ENABLED_AT_START)
            self.channels[n]=Channel(self,n,tty,g,en)
        self.mqtt=None
        self.setup_number=int(SETUP_DEFAULTS["address"])   # Ship Setup: only the ship number (= LoRa address) is editable
        self.setupq=queue.Queue()
    def load(self):
        try: return json.load(open(STATE_FILE))
        except Exception: return {}
    def save(self):
        try: json.dump({n:{"enabled":c.enabled} for n,c in self.channels.items()},open(STATE_FILE,"w"))
        except Exception as e: print("state save err",e,flush=True)
    def setup_mqtt(self):
        try:
            from paho.mqtt.client import CallbackAPIVersion
            c=mqtt.Client(CallbackAPIVersion.VERSION1)   # paho-mqtt 2.x (WB8/trixie)
        except ImportError:
            c=mqtt.Client()                               # paho-mqtt 1.x (older Debian / WB7)
        c.on_connect=self.on_connect; c.on_message=self.on_message
        c.connect("localhost",1883,60); self.mqtt=c; c.loop_start()
    def declare(self):
        for n,ch in self.channels.items():
            d=ch.dev; o=[0]
            self.mqtt.publish("/devices/%s/meta/name"%d,"Boat %s (%s)"%(n[-1],n.upper()),retain=True)
            def ctl(name,meta,val=None,d=d,o=o):
                o[0]+=1; m=dict(meta,order=o[0])
                self.mqtt.publish("/devices/%s/controls/%s/meta"%(d,name),json.dumps(m),retain=True)
                if val is not None: self.mqtt.publish("/devices/%s/controls/%s"%(d,name),str(val),retain=True)
            ctl("enabled",{"type":"switch","readonly":False},1 if ch.enabled else 0)
            ctl("mode",{"type":"text","readonly":True})
            for nm,u in (("battery_current","A"),("battery_temperature","°C"),("charge_level","%")):
                ctl(nm,{"type":"value","readonly":True,"units":u})
            for n2 in MOTOR_NAMES: ctl(n2,{"type":"range","readonly":False,"min":MOTOR_MIN,"max":MOTOR_MAX,"title":MOTOR_TITLE[n2]})
            for i in range(1,NLIGHTS+1): ctl("light%d"%i,{"type":"range","readonly":False,"min":0,"max":100,"title":LIGHT_TITLE.get(i,"Light %d"%i)})
            ctl("mp3_track",{"type":"range","readonly":False,"min":0,"max":MP3_TRACK_MAX})
            ctl("mp3_volume",{"type":"range","readonly":False,"min":0,"max":MP3_VOL_MAX})
            ctl("ship_number",{"type":"value","readonly":False,"min":0,"max":ADDR_MAX,"title":"Номер корабля"},ch.lora["address"])
            self.mqtt.subscribe("/devices/%s/controls/+/on"%d)
        # ---- Ship Setup dashboard (RS485-1 wired config) — unchanged ----
        sd="ship_setup"
        self.mqtt.publish("/devices/%s/meta/name"%sd,"Ship Setup (RS485-1)",retain=True)
        so=[0]
        def sctl(name,meta,val=None):
            so[0]+=1; m=dict(meta,order=so[0])
            self.mqtt.publish("/devices/%s/controls/%s/meta"%(sd,name),json.dumps(m),retain=True)
            if val is not None: self.mqtt.publish("/devices/%s/controls/%s"%(sd,name),str(val),retain=True)
        sctl("ship_number",{"type":"value","readonly":False,"min":0,"max":ADDR_MAX,"title":"Номер корабля"},self.setup_number)
        sctl("LoRa_address",{"type":"value","readonly":True,"title":"LoRa адрес"},self.setup_number)
        sctl("LoRa_channel",{"type":"value","readonly":True},"")
        sctl("LoRa_air_rate",{"type":"value","readonly":True,"units":"kbps"},"")
        sctl("LoRa_freq",{"type":"value","readonly":True,"units":"MHz"},"")
        sctl("LoRa_read",{"type":"pushbutton","title":"Считать"}); sctl("LoRa_apply",{"type":"pushbutton","title":"Записать"})
        sctl("LoRa_status",{"type":"text","readonly":True})
        self.mqtt.subscribe("/devices/%s/controls/+/on"%sd)
        # remove dashboards of absent modules (clear retained topics)
        for dev in getattr(self,"absent",[]):
            self.mqtt.publish("/devices/%s/meta/name"%dev,"",retain=True)
            for cname in BOAT_CONTROLS:
                self.mqtt.publish("/devices/%s/controls/%s/meta"%(dev,cname),"",retain=True)
                self.mqtt.publish("/devices/%s/controls/%s"%(dev,cname),"",retain=True)
    def on_connect(self,c,u,f,rc,props=None): self.declare()
    def on_message(self,c,u,msg):
        p=msg.topic.split("/"); dev=p[2]; ctrl=p[4]; val=msg.payload.decode(errors="ignore").strip()
        if dev=="ship_setup": self.setupq.put((ctrl,val)); return
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
        elif ctrl=="LoRa_read": self.setup_op(None)            # read connected ship's modem, show data
        elif ctrl=="LoRa_apply": self.setup_op(self.setup_number)  # write: pull this ship's radio from conf, program the modem
    def setup_op(self,num):   # num=None -> read only; else write ship #num's radio (from conf "ships") to the connected modem
        st=lambda v: self.mqtt.publish("/devices/ship_setup/controls/LoRa_status",v,retain=True)
        sp=lambda c,v: self.mqtt.publish("/devices/ship_setup/controls/%s"%c,str(v),retain=True)
        radio=None
        if num is not None:
            radio=SHIP_RADIO.get(int(num))
            if radio is None: st("ERR корабль №%s не настроен в конфиге"%num); return
        st("запись..." if num is not None else "чтение...")
        try:
            ser=serial.Serial(RS485,9600,8,"N",1,timeout=0.8)
            if radio is not None:
                air=AIR_CODE.get(("%g"%radio["air_rate"]),7); pw=PWR_CODE.get(str(int(radio["power"])),0)
                msg=bytes([0xC0,0x00,0x06,(int(num)>>8)&0xFF,int(num)&0xFF,SPED_BASE|air,OPTION_BASE|pw,int(radio["channel"])&0xFF,REG5_TXMODE])
                ser.reset_input_buffer(); ser.write(msg); ser.flush(); time.sleep(0.5); ser.read(64)
            ser.reset_input_buffer(); ser.write(bytes([0xC1,0x00,0x08])); ser.flush(); time.sleep(0.4); r=ser.read(64)
            ser.close()
            if len(r)>=11 and r[0]==0xC1:
                c=r[3:11]; addr=(c[0]<<8)|c[1]; air_s=AIR_NAME.get(c[2]&7,"?")
                sp("ship_number",addr); sp("LoRa_address",addr); sp("LoRa_channel",c[4]); sp("LoRa_air_rate",air_s); sp("LoRa_freq",round(FREQ_BASE+c[4],3))
                self.setup_number=addr
                st("OK №%d ch=%d freq=%.3f air=%s power=%s reg5=0x%02x raw=%s"%(addr,c[4],FREQ_BASE+c[4],air_s,PWR_NAME.get(c[3]&3,"?"),c[5],c.hex()))
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
        for ch in self.channels.values(): ch.start()
        threading.Thread(target=self.setup_worker,daemon=True).start()
        while True: time.sleep(1)

if __name__=="__main__": Driver().start()
