# -*- coding: utf-8 -*-
"""Замер точки страгивания мотора: с какого импульса вал начинает вращаться.

Зачем: низ шкалы газа сейчас лежит внутри мёртвой зоны, потому что где она
кончается — неизвестно. Прежняя оценка «скважность 42» = 1050 мкс имеет
неопределённость в целое деление той шкалы, то есть 25 мкс. После замера низ
шкалы можно сдвинуть на точку страгивания и вернуть себе десяток делений.

Как считать: кандидаты подаются по одному, между ними вал возвращается на
холостой. Смотришь на вал и называешь номер ПЕРВОГО кандидата, где он пошёл.
Сигнал не снимается ни на миг, поэтому ESC остаётся взведённым всё время.

Скважность держится постоянной (60 %), импульс задаётся частотой:
импульс = скважность * 1e4 / f. У холостого 1 Гц это 1.67 мкс.

    python3 thr-start.py                      # 9 кандидатов, 1002..1068 мкс
    python3 thr-start.py --from 1050 --to 1068 --steps 10   # уточняющий проход
    python3 thr-start.py --list               # только расписание, ничего не подавать

Драйвер должен быть остановлен: порт занят им.
"""
import argparse, sys, time

try:
    import serial
except ImportError:
    print("нет модуля serial"); sys.exit(1)

DUTY = 60                      # скважность моторных каналов постоянна
FREQ_REG = {1: 0, 2: 1, 3: 2}
DUTY_REG = {1: 112, 2: 113, 3: 114}
IDLE_US = 1000.0               # холостой: возврат между кандидатами
MOTORS = {"back_left": (12, 1), "front_left": (12, 2),
          "back_right": (11, 1), "front_right": (11, 2)}


def crc16(b):
    c = 0xFFFF
    for ch in b:
        c ^= ch
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return bytes([c & 0xFF, c >> 8])


class Bus(object):
    def __init__(self, tty, baud, timeout):
        self.ser = serial.Serial(tty, baud, 8, "N", 1, timeout=timeout)

    def wr(self, slave, reg, val):
        f = bytes([slave, 6, reg >> 8, reg & 0xFF, (val >> 8) & 0xFF, val & 0xFF])
        self.ser.reset_input_buffer()
        self.ser.write(f + crc16(f)); self.ser.flush()
        return len(self.ser.read(9)) >= 8

    def rd(self, slave, reg, n):
        f = bytes([slave, 3, reg >> 8, reg & 0xFF, 0, n])
        self.ser.reset_input_buffer()
        self.ser.write(f + crc16(f)); self.ser.flush()
        r = self.ser.read(5 + 2 * n + 1)
        if len(r) < 3 + 2 * n or r[0] != slave:
            return None
        d = r[3:3 + 2 * n]
        return [(d[i] << 8) | d[i + 1] for i in range(0, len(d), 2)]

    def close(self):
        self.ser.close()


def hz(us):
    """Импульс -> ближайшая целая частота, и что при ней получится на самом деле."""
    f = int(round(DUTY * 1e4 / us))
    return f, DUTY * 1e4 / f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tty", default="/dev/ttyMOD3")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--timeout", type=float, default=0.4)
    ap.add_argument("--motor", default="back_left", choices=sorted(MOTORS))
    ap.add_argument("--from", dest="lo", type=float, default=1002.0, help="импульс первого кандидата, мкс")
    ap.add_argument("--to", dest="hi", type=float, default=1068.0, help="импульс последнего кандидата, мкс")
    ap.add_argument("--steps", type=int, default=9, help="сколько кандидатов")
    ap.add_argument("--hold", type=float, default=3.0, help="держать кандидата, с")
    ap.add_argument("--rest", type=float, default=2.0, help="держать холостой между кандидатами, с")
    ap.add_argument("--arm", type=float, default=4.0, help="пауза на арминг ESC, с")
    ap.add_argument("--list", action="store_true", help="только расписание")
    a = ap.parse_args()

    n = max(2, a.steps)
    cands = []
    for i in range(n):
        us = a.lo + (a.hi - a.lo) * i / float(n - 1)
        f, real = hz(us)
        if cands and cands[-1][1] == f:
            continue                     # та же частота -> тот же импульс, кандидат лишний
        cands.append((len(cands) + 1, f, real))

    print("мотор: %s (модуль %d, канал %d), скважность %d %%"
          % (a.motor, MOTORS[a.motor][0], MOTORS[a.motor][1], DUTY))
    print("между кандидатами возврат на холостой %.0f мкс на %.1f с; сигнал не снимается\n" % (IDLE_US, a.rest))
    print("  №  частота   импульс")
    for i, f, real in cands:
        print("  %-2d %4d Гц  %7.1f мкс" % (i, f, real))
    step = (cands[-1][2] - cands[0][2]) / float(len(cands) - 1) if len(cands) > 1 else 0.0
    print("\n  шаг между кандидатами ~%.1f мкс, всего %d, время прогона ~%.0f с"
          % (step, len(cands), a.arm + len(cands) * (a.hold + a.rest)))
    if a.list:
        return

    slave, ch = MOTORS[a.motor]
    b = Bus(a.tty, a.baud, a.timeout)
    idle_f, _ = hz(IDLE_US)

    def pulse(f):
        b.wr(slave, FREQ_REG[ch], f)

    try:
        print("\nснимаю выход, ставлю холостой, жду арминг %.1f с" % a.arm)
        b.wr(slave, DUTY_REG[ch], 0)
        pulse(idle_f)
        b.wr(slave, DUTY_REG[ch], DUTY)
        time.sleep(a.arm)
        got = b.rd(slave, FREQ_REG[ch], 1)
        print("проверка: в модуле %s Гц, скважность %s\n"
              % (got[0] if got else "?", (b.rd(slave, DUTY_REG[ch], 1) or ["?"])[0]))
        for i, f, real in cands:
            pulse(f)
            print("  кандидат %-2d  %4d Гц = %7.1f мкс   %.1f с" % (i, f, real, a.hold), flush=True)
            time.sleep(a.hold)
            pulse(idle_f)
            time.sleep(a.rest)
        print("\nвозвращаю холостой")
        pulse(idle_f)
    finally:
        # покой как у драйвера: сначала снять выход, потом частота холостого
        b.wr(slave, DUTY_REG[ch], 0)
        pulse(idle_f)
        b.wr(slave, DUTY_REG[ch], DUTY)
        b.close()
    print("готово. Назови номер ПЕРВОГО кандидата, на котором вал пошёл.")


if __name__ == "__main__":
    main()
