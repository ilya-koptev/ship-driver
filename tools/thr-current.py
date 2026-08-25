# -*- coding: utf-8 -*-
"""Обороты по току АКБ: ступенчатый проход газа с замером battery_current.

Ток — объективная замена глазам: пока вал стоит, ESC потребляет только на себя,
и как только мотор пошёл, ток заметно растёт. Так находится точка страгивания
БОРТОВЫХ моторов (на стенде она 1047 мкс, у борта своя) и видно, монотонно ли
кривая газа отдаёт мощность.

По одному мотору за прогон — иначе ток не отнести к конкретному движителю.

    python3 thr-current.py --motor back_left
    python3 thr-current.py --motor back_left --divisions 1,2,3,4,5,6,7,8 --hold 5

Драйвер должен РАБОТАТЬ: команды идут через него по MQTT.
"""
import argparse, io, subprocess, sys, time

# Параметры кривой берём из КОНФИГА ДРАЙВЕРА, а не помним числами: пол шкалы
# подбирается по замеру и меняется, а скрипт с зашитыми числами начинает врать
# в столбце «импульс» — так и вышло 25.08 после сдвига пола 1047 -> 1080.
CONF = "/etc/ship-driver.conf"


def _limits():
    import json
    try:
        txt = io.open(CONF, encoding="utf-8").read()
        nl = chr(10)
        clean = nl.join("" if l.lstrip().startswith("//") else l for l in txt.split(nl))
        return json.loads(clean)["main"]["limits"]
    except Exception as e:
        print("конфиг не прочитан (%s), беру значения по умолчанию" % e)
        return {}


_L = _limits()
IDLE = float(_L.get("pulse_idle_us", 1000.0))
START = float(_L.get("pulse_start_us", 1047.0))
TOP = float(_L.get("pulse_top_us", 2000.0))
N = float(_L.get("throttle_max", 100))
G = float(_L.get("throttle_gamma", 1.5))
DUTY = float(_L.get("motor_duty", 60))
MOTORS = ("back_left", "back_right", "front_left", "front_right")


def us(t):
    if t <= 0:
        return 0.0
    if t < 2:
        return IDLE
    return START + (TOP - START) * (((min(t, N) - 2.0) / (N - 2.0)) ** G)


def hz(u):
    return 0 if u <= 0 else int(round(DUTY * 1e4 / u))


def pub(dev, ctrl, val):
    subprocess.call(["mosquitto_pub", "-t", "/devices/%s/controls/%s/on" % (dev, ctrl),
                     "-m", str(val)])


def sub(dev, ctrl, wait=4):
    try:
        out = subprocess.check_output(
            ["mosquitto_sub", "-t", "/devices/%s/controls/%s" % (dev, ctrl),
             "-C", "1", "-W", str(wait)], stderr=subprocess.DEVNULL)
        return float(out.decode().strip())
    except Exception:
        return None


def samples(dev, ctrl, n, wait=4):
    """n свежих значений подряд: контрол публикуется по кругу опроса, так что
    подписка отдаёт именно новые, а не retained."""
    out = []
    for _ in range(n):
        v = sub(dev, ctrl, wait)
        if v is not None:
            out.append(v)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="boat4")
    ap.add_argument("--motor", default="back_left", choices=MOTORS)
    ap.add_argument("--divisions", default="1,2,3,4,6,8,10,12,15,20,25,30")
    ap.add_argument("--hold", type=float, default=4.0)
    ap.add_argument("--n", type=int, default=2, help="сколько значений тока на ступень")
    a = ap.parse_args()

    divs = [float(x) for x in a.divisions.split(",") if x.strip()]

    print("борт %s, мотор %s. Остальные моторы не трогаю — они в том состоянии, что были."
          % (a.device, a.motor))
    v0 = sub(a.device, "battery_voltage")
    c0 = sub(a.device, "charge_level")
    print("до замера: напряжение %s В, заряд %s %%\n" % (v0, c0))

    base = samples(a.device, "battery_current", 3)
    print("покой (как есть): ток %s -> среднее %.3f А\n"
          % (", ".join("%.3f" % x for x in base), sum(base) / len(base) if base else 0.0))
    b = sum(base) / len(base) if base else 0.0

    print("  деление  импульс    частота   ток, А            прирост к покою")
    try:
        for d in divs:
            pub(a.device, a.motor, d)
            time.sleep(a.hold)
            s = samples(a.device, "battery_current", a.n)
            if not s:
                print("  %-8g %-10.1f %-9d нет значений" % (d, us(d), hz(us(d))))
                continue
            m = sum(s) / len(s)
            print("  %-8g %-10.1f %-9d %-17s %+.3f"
                  % (d, us(d), hz(us(d)), ", ".join("%.3f" % x for x in s), m - b))
            sys.stdout.flush()
    finally:
        pub(a.device, a.motor, 1)
        print("\nвернул %s в холостой (деление 1)" % a.motor)
    v1 = sub(a.device, "battery_voltage")
    c1 = sub(a.device, "charge_level")
    print("после замера: напряжение %s В, заряд %s %%" % (v1, c1))


if __name__ == "__main__":
    main()
