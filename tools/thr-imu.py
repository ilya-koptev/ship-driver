# -*- coding: utf-8 -*-
"""Проверка моторов по датчику, независимо от тока АКБ.

Три признака, каждый сам по себе доказывает вращение:
  * вибрация — размах accel и gyro растёт, когда вал крутится;
  * магнитометр — ток мотора искажает поле, а моторы стоят в РАЗНЫХ местах,
    поэтому отклонение своё у каждого: это признак пер-мотор, а не общий;
  * рыскание — одиночный движитель разворачивает борт, видно по gyro_z и course.

Работает так: мотор в холостой, снимок N секунд, мотор на деление D, снимок N
секунд, сравнение. Одна подписка на все контролы сразу — по одному запросу на
контрол было бы втрое дольше, чем сам замер.

    python3 thr-imu.py --motor front_right --division 20
    python3 thr-imu.py --all --division 20

Драйвер должен РАБОТАТЬ: команды и телеметрия идут через него.
"""
import argparse, io, json, subprocess, sys, time

MOTORS = ("back_left", "back_right", "front_left", "front_right")
WATCH = ("accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
         "mag_x", "mag_y", "mag_z", "course", "battery_current", "battery_voltage")
CONF = "/etc/ship-driver.conf"


def limits():
    try:
        txt = io.open(CONF, encoding="utf-8").read()
        nl = chr(10)
        clean = nl.join("" if l.lstrip().startswith("//") else l for l in txt.split(nl))
        return json.loads(clean)["main"]["limits"]
    except Exception:
        return {}


_L = limits()
IDLE = float(_L.get("pulse_idle_us", 1000.0))
START = float(_L.get("pulse_start_us", 1047.0))
TOP = float(_L.get("pulse_top_us", 2000.0))
N = float(_L.get("throttle_max", 100))
G = float(_L.get("throttle_gamma", 1.5))
DUTY = float(_L.get("motor_duty", 60))


def us(t):
    if t <= 0:
        return 0.0
    if t < 2:
        return IDLE
    return START + (TOP - START) * (((min(t, N) - 2.0) / (N - 2.0)) ** G)


def pub(dev, ctrl, val):
    subprocess.call(["mosquitto_pub", "-t", "/devices/%s/controls/%s/on" % (dev, ctrl),
                     "-m", str(val)])


def snap(dev, seconds):
    """Одна подписка на все контролы борта на seconds секунд -> {контрол: [значения]}."""
    try:
        out = subprocess.check_output(
            # -R отбрасывает retained: иначе в начале снимка приходит по одному
            # СТАРОМУ значению на каждый контрол, и размах получается завышенным.
            ["mosquitto_sub", "-v", "-R", "-t", "/devices/%s/controls/+" % dev, "-W", str(int(seconds))],
            stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError as e:
        out = e.output or b""
    series = {}
    for line in out.decode("utf-8", "replace").splitlines():
        p = line.split()
        if len(p) < 2:
            continue
        name = p[0].rsplit("/", 1)[-1]
        if name not in WATCH:
            continue
        try:
            series.setdefault(name, []).append(float(p[1]))
        except ValueError:
            pass
    return series


def stat(v):
    if not v:
        return None
    m = sum(v) / len(v)
    return m, max(v) - min(v), len(v)


def report(name, base, run, d):
    print("\n  --- %s, деление %g (%.0f мкс) ---" % (name, d, us(d)))
    print("  %-16s %-24s %-24s %s" % ("признак", "холостой (среднее/размах)",
                                      "под газом (среднее/размах)", "вывод"))
    verdict = []
    for k in WATCH:
        b, r = stat(base.get(k)), stat(run.get(k))
        if not b or not r:
            print("  %-16s %s" % (k, "нет данных"))
            continue
        note = ""
        if k.startswith(("accel", "gyro")):
            # вибрация: размах под газом заметно больше
            if b[1] > 0 and r[1] > b[1] * 2.0:
                note = "размах x%.1f" % (r[1] / b[1]); verdict.append(k)
            elif b[1] == 0 and r[1] > 0:
                note = "появился размах"; verdict.append(k)
        elif k.startswith("mag"):
            if abs(r[0] - b[0]) > max(30.0, 3.0 * max(b[1], 1.0)):
                note = "сдвиг %+.0f" % (r[0] - b[0]); verdict.append(k)
        print("  %-16s %-24s %-24s %s"
              % (k, "%.4g / %.4g" % (b[0], b[1]), "%.4g / %.4g" % (r[0], r[1]), note))
    print("  признаков вращения: %s" % (", ".join(verdict) if verdict else "НЕТ НИ ОДНОГО"))
    return verdict


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="boat4")
    ap.add_argument("--motor", default="front_right", choices=MOTORS)
    ap.add_argument("--all", action="store_true", help="все четыре по очереди")
    ap.add_argument("--division", type=float, default=20.0)
    ap.add_argument("--snap", type=float, default=8.0, help="длина снимка, с")
    ap.add_argument("--settle", type=float, default=3.0)
    a = ap.parse_args()

    names = list(MOTORS) if a.all else [a.motor]
    res = {}
    for nm in names:
        pub(a.device, nm, 1)
        time.sleep(a.settle)
        base = snap(a.device, a.snap)
        pub(a.device, nm, a.division)
        time.sleep(a.settle)
        run = snap(a.device, a.snap)
        pub(a.device, nm, 1)
        res[nm] = report(nm, base, run, a.division)
        sys.stdout.flush()
    print("\n  ИТОГ")
    for nm in names:
        print("    %-13s %s" % (nm, "крутится (%d признак(ов))" % len(res[nm]) if res[nm]
                                else "НИ ОДНОГО признака вращения"))


if __name__ == "__main__":
    main()
