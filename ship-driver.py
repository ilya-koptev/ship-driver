#!/usr/bin/env python3
# ship-driver.py — custom multi-channel ship-bus driver.
# 4 equivalent LoRa transmitters (MOD1-4 = /dev/ttyMOD1..4). Each channel independently owns its
# port and runs the SAME ship logic against one active ship (Modbus addrs identical on every ship:
# UPS=10, pwm=11/12/13; ships are told apart by LoRa address -> reconfigure the channel's modem to
# switch ship). Per channel: Modbus RTU + inline DFPlayer + mode state machine + poll schedule +
# simple charge-thermal + the channel's own LoRa modem config. Mirrors to MQTT (WB convention).
import time, threading, queue, json, os
import serial
import paho.mqtt.client as mqtt
from paho.mqtt.client import CallbackAPIVersion

BAUD=9600; RESP_TO=0.8
# channels: name -> (tty, config-gpio). MOD1-4 are equivalent.
CHANNELS={"mod1":("/dev/ttyMOD1",519),"mod2":("/dev/ttyMOD2",522),
          "mod3":("/dev/ttyMOD3",271),"mod4":("/dev/ttyMOD4",106)}
ENABLED_AT_START={"mod1","mod2","mod3","mod4"}   # default (no saved state): all channels active -> SEARCH until a ship appears
LORA_PLAN={"mod1":(14,3),"mod2":(16,23),"mod3":(17,43),"mod4":(19,63)}  # (channel, address) defaults
STATE_FILE="/etc/ship-driver-state.json"   # persist per-channel enabled + modem config across reboot
RS485="/dev/ttyRS485-1"   # Ship Setup dashboard: wired ship LoRa-modem config (ship modem in config mode by its switch)

PWM_SLAVES=[11,12,13]; FREQ_REG={1:0,2:1,3:2}; DUTY_REG={1:112,2:113,3:114}
MOTORS=[("back_left",12,1),("front_left",12,2),("back_right",11,1),("front_right",11,2)]  # after pwm addr rotation 11->13,12->11,13->12 (same physical outputs)
MOTOR_MAP={n:(s,c) for n,s,c in MOTORS}
MOTOR_TITLE={"front_right":"Front Right","back_right":"Back Right","front_left":"Front Left","back_left":"Back Left"}
LIGHTS=[(11,3),(12,3),(13,2),(13,1),(13,3)]  # light1..5 renamed 2026-06-25 (1->5,2->1,3->4,4->3,5->2)
ALL_CH=[(s,c) for s in PWM_SLAVES for c in (1,2,3)]
INIT_FREQ=400; INIT_MOTOR=40; INIT_LIGHT=0
MOTOR_MIN=40; MOTOR_MAX=80   # motor slider range (40=idle, 80=max)
UPS=10; UPS_CUR=5; UPS_CHG=8; UPS_TEMP=9; UPS_CHG_SETPOINT=18
CHG_FULL=2000; CHG_LOW=300; T_LIMIT=50.0; T_RESTORE=48.0
SAIL_TIMEOUT=30.0; OFFLINE_FAILS=2

SEARCH="SEARCH"; SAIL="SAILING"; CHARGE="CHARGING"; IDLE="IDLE"; SERVICE="SERVICE"; OFF="OFF"
RATES={
 CHARGE: {"current":5,"temp":10,"charge":60,"motors":60,"lights":60,"freq":60},
 SAIL:   {"current":5,"temp":300,"charge":300,"motors":60,"lights":60,"freq":300},
 IDLE:   {"current":5,"temp":300,"charge":300,"motors":300,"lights":300,"freq":300},
}
SEARCH_PERIOD=0.05; SERVICE_PERIOD=1.0   # search paced by the read timeout itself (~0.8s/probe), no extra sleep

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
        self.mode=OFF; self.online=False; self.fails=0
        self.last_cmd=0.0
        self.chg_setpoint=CHG_FULL; self.tele={}
        self.motor={n:0 for n,_,_ in MOTORS}; self.light={k:0 for k in LIGHTS}
        self.due={}
        ch,addr=LORA_PLAN[name]; self.lora={"channel":ch,"air_rate":62.5,"address":addr,"power":22}

    # ---- serial / modbus ----
    def open(self):
        if self.ser is None:
            self.ser=serial.Serial(self.tty,BAUD,8,"N",1,timeout=RESP_TO)
            gpio_set(self.gpio,0)   # ensure TRANSPARENT (relay) mode for normal operation — for ALL channels
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
    def pub(self,ctrl,val): self.drv.mqtt.publish("/devices/%s/controls/%s"%(self.dev,ctrl),str(val),retain=True)

    # ---- command handling (this thread) ----
    def handle(self,ctrl,val):
        try: fv=float(val)
        except Exception: fv=0.0
        iv=int(fv); is_cmd=True
        if ctrl=="enabled":
            self.enabled=(val in ("1","true","on")); is_cmd=False
            if not self.enabled: self.online=False; self.set_mode(OFF); self.close()
        elif ctrl in MOTOR_MAP and self.online:
            s,c=MOTOR_MAP[ctrl]; self.motor[ctrl]=max(MOTOR_MIN,min(MOTOR_MAX,iv)); self.write_reg(s,DUTY_REG[c],self.motor[ctrl]); self.pub(ctrl,self.motor[ctrl])
        elif ctrl.startswith("light") and self.online:
            i=int(ctrl[5:])-1; k=LIGHTS[i]; self.light[k]=max(0,min(100,iv)); self.write_reg(k[0],DUTY_REG[k[1]],self.light[k]); self.pub(ctrl,self.light[k])
        elif ctrl=="freq" and self.online:
            f=max(0,min(20000,iv))
            for s,c in ALL_CH: self.write_reg(s,FREQ_REG[c],f)
            self.pub("freq",f)
        elif ctrl=="mp3_track" and self.online:
            is_cmd=False; iv=max(0,min(15,iv)); self.send_mp3(mp3_frame(MP3["stop"]) if iv<=0 else mp3_frame(MP3["play"],iv)); self.pub("mp3_track",iv)
        elif ctrl=="mp3_volume" and self.online:
            is_cmd=False; v=max(0,min(30,iv)); self.send_mp3(mp3_frame(MP3["vol"],v)); self.pub("mp3_volume",v)
        # ---- LoRa modem config ----
        elif ctrl=="LoRa_channel": is_cmd=False; self.lora["channel"]=iv; self.pub(ctrl,iv); self.pub("LoRa_freq",round(850.125+iv,3))
        elif ctrl=="LoRa_address": is_cmd=False; self.lora["address"]=iv; self.pub(ctrl,iv); self.lora_op(self.lora)  # apply immediately (ship switch)
        elif ctrl=="LoRa_air_rate": is_cmd=False; self.lora["air_rate"]=fv; self.pub(ctrl,fv)
        elif ctrl=="LoRa_power": is_cmd=False; self.lora["power"]=iv; self.pub(ctrl,iv)
        elif ctrl=="LoRa_read": is_cmd=False; self.lora_op(None)
        elif ctrl=="LoRa_apply": is_cmd=False; self.lora_op(self.lora)
        else: is_cmd=False
        if ctrl in ("enabled","LoRa_channel","LoRa_address","LoRa_air_rate","LoRa_power","LoRa_apply"): self.drv.save()
        if is_cmd: self.last_cmd=time.monotonic()
    def drain(self):
        while True:
            try: ctrl,val=self.q.get_nowait()
            except queue.Empty: break
            try: self.handle(ctrl,val)
            except Exception as e: print("[%s] cmd err %s %s"%(self.name,ctrl,e),flush=True)

    # ---- LoRa config of THIS channel's modem ----
    def lora_op(self,write):
        self.pub("LoRa_status","applying..." if write else "reading...")
        was_open=self.ser is not None
        try:
            self.open()
            gpio_set(self.gpio,1); time.sleep(0.4)
            if write:
                air=AIR_CODE.get(("%g"%write["air_rate"]),7); pw=PWR_CODE.get(str(int(write["power"])),0)
                msg=bytes([0xC0,0x00,0x05,(int(write["address"])>>8)&0xFF,int(write["address"])&0xFF,0x60|air,0x60|pw,int(write["channel"])&0xFF])
                self.ser.reset_input_buffer(); self.ser.write(msg); self.ser.flush(); time.sleep(0.5); self.ser.read(64)
            self.ser.reset_input_buffer(); self.ser.write(bytes([0xC1,0x00,0x08])); self.ser.flush(); time.sleep(0.4); r=self.ser.read(64)
            if len(r)>=11 and r[0]==0xC1:
                c=r[3:11]; addr=(c[0]<<8)|c[1]
                self.pub("LoRa_freq",round(850.125+c[4],3))
                self.pub("LoRa_status","OK ch=%d freq=%.3f air=%s addr=%d power=%s"%(c[4],850.125+c[4],AIR_NAME.get(c[2]&7,"?"),addr,PWR_NAME.get(c[3]&3,"?")))
            else: self.pub("LoRa_status","ERR no response")
        except Exception as e: self.pub("LoRa_status","ERR %s"%e)
        finally:
            gpio_set(self.gpio,0)
            if not was_open: self.close()

    # ---- ship logic ----
    def init_ship(self):
        for s,c in ALL_CH: self.write_reg(s,FREQ_REG[c],INIT_FREQ)
        for n,s,c in MOTORS: self.motor[n]=INIT_MOTOR; self.write_reg(s,DUTY_REG[c],INIT_MOTOR)
        for s,c in LIGHTS: self.light[(s,c)]=INIT_LIGHT; self.write_reg(s,DUTY_REG[c],INIT_LIGHT)
        self.pub("freq",INIT_FREQ)
        for n,s,c in MOTORS: self.pub(n,self.motor[n])
        for i,k in enumerate(LIGHTS,1): self.pub("light%d"%i,self.light[k])
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
        for n,s,c in MOTORS:
            r=self.read_regs(s,3,DUTY_REG[c],1)
            if r is None: ok=False
            else: self.pub(n,r[0])
        return ok
    def poll_lights(self):
        ok=True
        for i,(s,c) in enumerate(LIGHTS,1):
            r=self.read_regs(s,3,DUTY_REG[c],1)
            if r is None: ok=False
            else: self.pub("light%d"%i,r[0])
        return ok
    def poll_freq(self):
        r=self.read_regs(11,3,FREQ_REG[1],1)
        if r is None: return False
        self.pub("freq",r[0]); return True
    GROUPS={"current":"poll_current","temp":"poll_temp","charge":"poll_charge","motors":"poll_motors","lights":"poll_lights","freq":"poll_freq"}
    def thermal(self):
        t=self.tele.get("temp")
        if t is None: return
        new=self.chg_setpoint
        if t>T_LIMIT: new=CHG_LOW
        elif t<T_RESTORE: new=CHG_FULL
        if new!=self.chg_setpoint:
            self.chg_setpoint=new; self.write_reg(UPS,UPS_CHG_SETPOINT,new); self.pub("charge_setpoint",new)
    def decide(self):
        if not self.online: return SEARCH
        if time.monotonic()-self.last_cmd < SAIL_TIMEOUT: return SAIL
        return CHARGE if self.tele.get("current",0)>0 else IDLE
    def set_mode(self,m):
        if m!=self.mode:
            self.mode=m; self.pub("mode",m)
            if m==CHARGE:
                self.chg_setpoint=CHG_FULL; self.write_reg(UPS,UPS_CHG_SETPOINT,CHG_FULL); self.pub("charge_setpoint",CHG_FULL)

    def run(self):
        while True:
            self.drain()
            if not self.enabled:
                self.set_mode(OFF); time.sleep(0.5); continue
            if self.drv.service:                          # global Ship-Setup mode: pause radio (free air/wire)
                self.set_mode(SERVICE); time.sleep(0.5); continue
            self.open()
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
            ch=Channel(self,n,tty,g,en)
            if isinstance(st.get(n,{}).get("lora"),dict): ch.lora.update(st[n]["lora"])
            self.channels[n]=ch
        self.mqtt=None
        self.service=False                       # global Ship-Setup mode (pauses radio channels)
        self.setup_lora={"channel":14,"air_rate":62.5,"address":3,"power":22}
        self.setupq=queue.Queue()
    def load(self):
        try: return json.load(open(STATE_FILE))
        except Exception: return {}
    def save(self):
        try: json.dump({n:{"enabled":c.enabled,"lora":c.lora} for n,c in self.channels.items()},open(STATE_FILE,"w"))
        except Exception as e: print("state save err",e,flush=True)
    def setup_mqtt(self):
        c=mqtt.Client(CallbackAPIVersion.VERSION1); c.on_connect=self.on_connect; c.on_message=self.on_message
        c.connect("localhost",1883,60); self.mqtt=c; c.loop_start()
    def declare(self):
        for n,ch in self.channels.items():
            d=ch.dev; P=lambda t,v: self.mqtt.publish("/devices/%s/%s"%(d,t),v,retain=True); o=[0]
            P("meta/name","Boat %s (%s)"%(n[-1],n.upper()))
            def ctl(name,meta,val=None,d=d,o=o):
                o[0]+=1; m=dict(meta,order=o[0])
                self.mqtt.publish("/devices/%s/controls/%s/meta"%(d,name),json.dumps(m),retain=True)
                if val is not None: self.mqtt.publish("/devices/%s/controls/%s"%(d,name),str(val),retain=True)
            ctl("enabled",{"type":"switch","readonly":False},1 if ch.enabled else 0)
            ctl("mode",{"type":"text","readonly":True})
            for nm,u in (("battery_current","A"),("battery_temperature","°C"),("charge_level","%"),("charge_setpoint","mA")):
                ctl(nm,{"type":"value","readonly":True,"units":u})
            for n,_,_ in MOTORS: ctl(n,{"type":"range","readonly":False,"min":MOTOR_MIN,"max":MOTOR_MAX,"title":MOTOR_TITLE[n]})
            for i in range(1,6): ctl("light%d"%i,{"type":"range","readonly":False,"min":0,"max":100})
            ctl("freq",{"type":"value","readonly":False,"min":0,"max":20000,"units":"Hz"})
            ctl("mp3_track",{"type":"range","readonly":False,"min":0,"max":15})
            ctl("mp3_volume",{"type":"range","readonly":False,"min":0,"max":30})
            # LoRa modem config (this channel's transmitter)
            ctl("LoRa_channel",{"type":"value","readonly":False,"min":0,"max":83},ch.lora["channel"])
            ctl("LoRa_freq",{"type":"value","readonly":True,"units":"MHz"},round(850.125+ch.lora["channel"],3))
            ctl("LoRa_air_rate",{"type":"value","readonly":False,"min":0,"max":100,"units":"kbps"},ch.lora["air_rate"])
            ctl("LoRa_address",{"type":"value","readonly":False,"min":0,"max":65535},ch.lora["address"])
            ctl("LoRa_power",{"type":"value","readonly":False,"min":0,"max":22,"units":"dBm"},ch.lora["power"])
            ctl("LoRa_read",{"type":"pushbutton"}); ctl("LoRa_apply",{"type":"pushbutton"})
            ctl("LoRa_status",{"type":"text","readonly":True})
            self.mqtt.subscribe("/devices/%s/controls/+/on"%d)
        # ---- Ship Setup dashboard (RS485-1 wired config) ----
        sd="ship_setup"
        self.mqtt.publish("/devices/%s/meta/name"%sd,"Ship Setup (RS485-1)",retain=True)
        so=[0]
        def sctl(name,meta,val=None):
            so[0]+=1; m=dict(meta,order=so[0])
            self.mqtt.publish("/devices/%s/controls/%s/meta"%(sd,name),json.dumps(m),retain=True)
            if val is not None: self.mqtt.publish("/devices/%s/controls/%s"%(sd,name),str(val),retain=True)
        sctl("service",{"type":"switch","readonly":False},1 if self.service else 0)
        sctl("LoRa_channel",{"type":"value","readonly":False,"min":0,"max":83},self.setup_lora["channel"])
        sctl("LoRa_freq",{"type":"value","readonly":True,"units":"MHz"},round(850.125+self.setup_lora["channel"],3))
        sctl("LoRa_air_rate",{"type":"value","readonly":False,"min":0,"max":100,"units":"kbps"},self.setup_lora["air_rate"])
        sctl("LoRa_address",{"type":"value","readonly":False,"min":0,"max":65535},self.setup_lora["address"])
        sctl("LoRa_power",{"type":"value","readonly":False,"min":0,"max":22,"units":"dBm"},self.setup_lora["power"])
        sctl("LoRa_read",{"type":"pushbutton"}); sctl("LoRa_apply",{"type":"pushbutton"})
        sctl("LoRa_status",{"type":"text","readonly":True})
        self.mqtt.subscribe("/devices/%s/controls/+/on"%sd)
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
        if ctrl=="service": self.service=(val in ("1","true","on")); sp("service",1 if self.service else 0)
        elif ctrl=="LoRa_channel": self.setup_lora["channel"]=int(float(val)); sp(ctrl,self.setup_lora["channel"]); sp("LoRa_freq",round(850.125+self.setup_lora["channel"],3))
        elif ctrl=="LoRa_address": self.setup_lora["address"]=int(float(val)); sp(ctrl,self.setup_lora["address"])
        elif ctrl=="LoRa_air_rate": self.setup_lora["air_rate"]=float(val); sp(ctrl,self.setup_lora["air_rate"])
        elif ctrl=="LoRa_power": self.setup_lora["power"]=int(float(val)); sp(ctrl,self.setup_lora["power"])
        elif ctrl=="LoRa_read": self.setup_op(None)
        elif ctrl=="LoRa_apply": self.setup_op(self.setup_lora)
    def setup_op(self,write):
        st=lambda v: self.mqtt.publish("/devices/ship_setup/controls/LoRa_status",v,retain=True)
        st("applying..." if write else "reading...")
        try:
            ser=serial.Serial(RS485,9600,8,"N",1,timeout=0.8)
            if write:
                air=AIR_CODE.get(("%g"%write["air_rate"]),7); pw=PWR_CODE.get(str(int(write["power"])),0)
                msg=bytes([0xC0,0x00,0x05,(int(write["address"])>>8)&0xFF,int(write["address"])&0xFF,0x60|air,0x60|pw,int(write["channel"])&0xFF])
                ser.reset_input_buffer(); ser.write(msg); ser.flush(); time.sleep(0.5); ser.read(64)
            ser.reset_input_buffer(); ser.write(bytes([0xC1,0x00,0x08])); ser.flush(); time.sleep(0.4); r=ser.read(64)
            ser.close()
            if len(r)>=11 and r[0]==0xC1:
                c=r[3:11]; addr=(c[0]<<8)|c[1]
                self.mqtt.publish("/devices/ship_setup/controls/LoRa_freq",str(round(850.125+c[4],3)),retain=True)
                st("OK ch=%d freq=%.3f air=%s addr=%d power=%s"%(c[4],850.125+c[4],AIR_NAME.get(c[2]&7,"?"),addr,PWR_NAME.get(c[3]&3,"?")))
            else:
                st("ERR no response (ship modem in CONFIG mode?)")
        except Exception as e: st("ERR %s"%e)
    def start(self):
        self.setup_mqtt(); time.sleep(1.0)
        for ch in self.channels.values(): ch.start()
        threading.Thread(target=self.setup_worker,daemon=True).start()
        while True: time.sleep(1)

if __name__=="__main__": Driver().start()
