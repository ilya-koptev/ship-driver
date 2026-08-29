# -*- coding: utf-8 -*-
"""Обороты по току АКБ: ступенчатый проход газа с замером battery_current.

Ток — объективная замена глазам: пока вал стоит, ESC потребляет только на себя,
и как только мотор пошёл, ток заметно растёт. Так находится точка страгивания
БОРТОВЫХ моторов (на стенде она своя, винтов там нет) и видно, монотонно ли
кривая газа отдаёт мощность.

Два правила, на которых первая версия обожглась:
  * базовая линия снимается ВНУТРИ прогона, после принудительного холостого и
    выдержки. Иначе она захватывает выбег предыдущего мотора, и весь столбец
    «прирост» уезжает — так вышло 25.08, пришлось пересчитывать вручную;
  * кривая читается ИЗ КОНФИГА драйвера, а не помнится числами: пол шкалы
    подбирается замером и меняется, а скрипт с зашитым полом начинает врать.

По одному мотору за прогон — иначе ток не отнести к конкретному движителю.
С --all прогоняются все четыре по очереди и печатается сводная таблица.

    python3 thr-current.py --motor back_left
    python3 thr-current.py --all
    python3 thr-current.py --all --divisions 1,2,5,10,20,30 --hold 5

Драйвер должен РАБОТАТЬ: команды идут через него по MQTT.
"""
import argparse, io, json, subprocess, sys, time

MOTORS = ("back_left", "back_right", "front_left", "front_right")
CONF = "/etc/ship-driver.conf"


def limits():
    try:
        txt = io.open(CONF, encoding="utf-8").read()
        nl = chr(10)
        clean = nl.join("" if l.lstrip().startswith("//") else l for l in txt.split(nl))
        return json.loads(clean)["main"]["limits"]
    except Exception as e:
        print("конфиг не прочитан (%s), беру значения по умолчанию" % e)
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


def hz(u):
    return 0 if u <= 0 else int(round(DUTY * 1e4 / u))


def pub(dev, ctrl, val):
    subprocess.call(["mosquitto_pub", "-t", "/devices/%s/controls/%s/on" % (dev, ctrl),
                     "-m", str(val)])


def one(dev, ctrl, wait=8):
    """Одно СВЕЖЕЕ значение контрола.

    Ключ -R обязателен: без него `mosquitto_sub -C 1` отдаёт сохранённое (retained)
    сообщение мгновенно, и «три отсчёта подряд» оказываются одним и тем же последним
    значением, повторённым трижды. Проверено 25.08 на стенде: без -R ответ приходит
    за 20-26 мс и не меняется, с -R за 10 с не приходит ничего, пока контрол не
    опубликуют заново. Заявлять усреднение по retained-значению нельзя."""
    try:
        out = subprocess.check_output(
            ["mosquitto_sub", "-t", "/devices/%s/controls/%s" % (dev, ctrl),
             "-R", "-C", "1", "-W", str(wait)], stderr=subprocess.DEVNULL)
        return float(out.decode().strip())
    except Exception:
        return None


def cur(dev, n):
    """n свежих значений тока: контрол публикуется по кругу опроса, значит
    подписка отдаёт новые, а не retained. Медиана — чтобы один выброс не решал."""
    v = []
    for _ in range(n):
        x = one(dev, "battery_current")
        if x is not None:
            v.append(x)
    if not v:
        return None, []
    s = sorted(v)
    return s[len(s) // 2], v


def sweep(dev, motor, divs, hold, n, settle):
    print("\n### %s" % motor)
    pub(dev, motor, 1)
    print("  холостой + выдержка %.0f с (чтобы не поймать выбег предыдущего мотора)" % settle)
    time.sleep(settle)
    base, bv = cur(dev, 3)
    if base is None:
        print("  ток не читается — прогон отменён"); return {}
    print("  базовая линия: %s -> %.3f А" % (", ".join("%.3f" % x for x in bv), base))
    print("  деление  импульс    частота   ток, А    лишний ток")
    out = {}
    try:
        for d in divs:
            pub(dev, motor, d)
            time.sleep(hold)
            m, v = cur(dev, n)
            if m is None:
                print("  %-8g %-10.1f %-9d нет значений" % (d, us(d), hz(us(d)))); continue
            extra = abs(m) - abs(base)
            out[d] = extra
            print("  %-8g %-10.1f %-9d %-9.3f %+.3f" % (d, us(d), hz(us(d)), m, extra))
            sys.stdout.flush()
    finally:
        pub(dev, motor, 1)
    return out


PIDFILE = "/run/thr-sweep.pid"


def running_pid():
    """PID работающего прогона, иначе None. Нужен потому, что 25.08 прогон
    остался незамеченным: команда считалась отменённой, а на контроллере уже
    шла, гоняла борт по воде и перекрывала попытки поставить моторы в холостой."""
    try:
        pid = int(io.open(PIDFILE).read().strip())
    except Exception:
        return None
    try:
        io.open("/proc/%d/cmdline" % pid).read()
        return pid
    except Exception:
        return None


def park(dev):
    for m in MOTORS:
        pub(dev, m, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="boat4")
    ap.add_argument("--stop", action="store_true",
                    help="убить работающий прогон и поставить все моторы в холостой")
    ap.add_argument("--force", action="store_true",
                    help="запустить, даже если другой прогон помечен работающим")
    ap.add_argument("--motor", default="back_left", choices=MOTORS)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--divisions", default="1,2,5,10,15,20,25,30,40")
    ap.add_argument("--hold", type=float, default=4.0)
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--settle", type=float, default=6.0)
    a = ap.parse_args()

    if a.stop:
        pid = running_pid()
        if pid:
            import signal, os
            os.kill(pid, signal.SIGTERM)
            print("прогон %d остановлен" % pid)
        else:
            print("работающего прогона нет")
        park(a.device)
        print("все моторы в холостой")
        return

    pid = running_pid()
    if pid and not a.force:
        print("УЖЕ ИДЁТ прогон (pid %d). Останови его: thr-current.py --stop" % pid)
        sys.exit(1)
    io.open(PIDFILE, "w").write(str(__import__("os").getpid()))

    divs = [float(x) for x in a.divisions.split(",") if x.strip()]
    names = list(MOTORS) if a.all else [a.motor]

    print("борт %s. Кривая из конфига: холостой %.0f, страгивание %.0f, верх %.0f, "
          "делений %g, gamma %g, скважность %g %%"
          % (a.device, IDLE, START, TOP, N, G, DUTY))
    print("напряжение %s В, заряд %s %%" % (one(a.device, "battery_voltage"),
                                            one(a.device, "charge_level")))

    res = {}
    try:
        for nm in names:
            res[nm] = sweep(a.device, nm, divs, a.hold, a.n, a.settle)
    finally:
        park(a.device)
        try: __import__("os").unlink(PIDFILE)
        except Exception: pass

    print("\n\n### СВОДНО: лишний ток над холостым, А")
    print("  деление  импульс    частота   " + "  ".join("%-12s" % m for m in names))
    for d in divs:
        row = "  %-8g %-10.1f %-9d " % (d, us(d), hz(us(d)))
        row += "  ".join("%-12s" % ("%.3f" % res[m][d] if d in res.get(m, {}) else "—")
                         for m in names)
        print(row)
    print("\nнапряжение %s В, заряд %s %%" % (one(a.device, "battery_voltage"),
                                              one(a.device, "charge_level")))


if __name__ == "__main__":
    main()
