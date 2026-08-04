# ship-driver — MQTT API

**Брокер:** `192.168.69.105:1883`  
**Активный канал:** `boat4` (LoRa CH19, корабль №9)  
**Контроллер:** WB8 (wirenboard-AKEYTVSO)

---

## Устройства MQTT

| Устройство | Топик | Назначение |
|---|---|---|
| `boat1..4` | `/devices/boatN/…` | Оперативный API канала + визуализация |
| `ship_setup` | `/devices/ship_setup/…` | Преднастройка LoRa-модема корабля по проводу (RS485-1) |
| `charger1..N` | `/devices/chargerN/…` | Беспроводные зарядные станции (реле передатчика/магнитов + ток) |

Дашборд `boatN` появляется **только для тех MOD-слотов, где при старте нашёлся LoRa-модем**. На этом контроллере активен только `boat4`.

---

## Доступ

Внешние клиенты (те, что управляют кораблём) подключаются к брокеру `192.168.69.105:1883`. На этом порту видны **только топики корабля** — `/devices/boatN/#`, `/devices/chargerN/#` и `/devices/ship_setup|ship_diag|ship_bus/#` (и на чтение, и на запись). Всё остальное на контроллере (`/devices/system`, `/devices/network`, `wb-*` и т. п.) с этого порта закрыто.

Сам набор топиков и контролов при этом **не менялся** — управление и телеметрия ровно те же, что и раньше; поменялась только область видимости на внешнем порту.

---

## Контролы boat4

### Чтение (subscribe)

```bash
# подписаться на всю телеметрию
mosquitto_sub -h 192.168.69.105 -t '/devices/boat4/controls/+' -v

# один контрол
mosquitto_sub -h 192.168.69.105 -t '/devices/boat4/controls/mode' -C 1
```

| Контрол | Топик | ro/rw | Значение |
|---|---|---|---|
| `mode` | `/devices/boat4/controls/mode` | ro | `SEARCH` / `SAILING` / `CHARGING` / `IDLE` / `OFF` |
| `enabled` | `/devices/boat4/controls/enabled` | rw | `0` / `1` |
| `ship_number` | `/devices/boat4/controls/ship_number` | rw | `0–65535` — LoRa-адрес корабля, пишется в модем немедленно |
| `battery_current` | `/devices/boat4/controls/battery_current` | ro | A, float · разряд отрицательный · период 5 с в IDLE |
| `battery_voltage` | `/devices/boat4/controls/battery_voltage` | ro | В · напряжение АКБ (~8 В) · вместе с током |
| `battery_temperature` | `/devices/boat4/controls/battery_temperature` | ro | °C · 5 мин в IDLE, 10 с в CHARGING |
| `charge_level` | `/devices/boat4/controls/charge_level` | ro | % заряда |
| `input_voltage` | `/devices/boat4/controls/input_voltage` | ro | В · входное напряжение ИБП · период 5 с (читается вместе с током) |
| `rssi` | `/devices/boat4/controls/rssi` | ro | дБм · RSSI LoRa‑линка активного борта (E220 packet‑byte) |
| `comms_errors` | `/devices/boat4/controls/comms_errors` | ro | сырых промахов чтения за скользящие 5 мин |
| `link_quality` | `/devices/boat4/controls/link_quality` | ro | % успешных чтений за 5 мин (100 = идеал) |
| `read_failures` | `/devices/boat4/controls/read_failures` | ro | отказов чтения **после всех повторов** за 5 мин = реально потерянные данные |
| `err_timeout` | `/devices/boat4/controls/err_timeout` | ro | отказов «не пришло ни байта» за 5 мин |
| `err_frame` | `/devices/boat4/controls/err_frame` | ro | отказов «обрывок / чужой кадр / битый CRC» за 5 мин |
| `retry_fixed` | `/devices/boat4/controls/retry_fixed` | ro | промахов, вылеченных повтором, за 5 мин |
| `course` | `/devices/boat4/controls/course` | ro | ° · курс борта (−180…180) с датчика WT901C485; публикуется только если датчик есть на этом борту |
| `lat_p95` | `/devices/boat4/controls/lat_p95` | ro | мс · латентность чтения, 95‑й перцентиль |
| `back_left` / `front_left` / `back_right` / `front_right` | `/devices/boat4/controls/{name}` | rw | `40–80` (40 = холостой ход) |
| `nav_lights` / `morse_lamp` / `deck_lights` / `cabin_light1` / `cabin_light2` | `/devices/boat4/controls/{name}` | rw | `0–100` % |
| `mp3_track` | `/devices/boat4/controls/mp3_track` | rw | `0` = стоп, `1–15` = трек |
| `mp3_volume` | `/devices/boat4/controls/mp3_volume` | rw | `0–30` |

> **Важно:** контролы моторов, света и батареи видны на дашборде, только пока канал онлайн (режимы SAILING / CHARGING / IDLE). В SEARCH и OFF они прячутся и возвращаются, когда борт снова выходит на связь.

### Запись (publish → `/on`)

Любая команда записи публикуется в топик `/devices/boat4/controls/<ctrl>/on`.

```bash
# мотор вперёд-вправо на 60 %
mosquitto_pub -h 192.168.69.105 \
  -t '/devices/boat4/controls/front_right/on' \
  -m 60

# ходовые огни на 100 %
mosquitto_pub -h 192.168.69.105 \
  -t '/devices/boat4/controls/nav_lights/on' \
  -m 100

# включить трек 3 на громкости 20
mosquitto_pub -h 192.168.69.105 -t '/devices/boat4/controls/mp3_volume/on' -m 20
mosquitto_pub -h 192.168.69.105 -t '/devices/boat4/controls/mp3_track/on'   -m 3

# переключить канал на корабль №15 (пишет адрес в LoRa-модем немедленно)
mosquitto_pub -h 192.168.69.105 \
  -t '/devices/boat4/controls/ship_number/on' \
  -m 15

# остановить все моторы (холостой ход)
for m in front_left front_right back_left back_right; do
  mosquitto_pub -h 192.168.69.105 \
    -t "/devices/boat4/controls/${m}/on" \
    -m 40
done

# выключить канал (порт закрывается, LoRa переходит в config-mode)
mosquitto_pub -h 192.168.69.105 \
  -t '/devices/boat4/controls/enabled/on' \
  -m 0
```

> Команды моторов и света срабатывают только в режимах **SAILING / CHARGING / IDLE**; в SEARCH и OFF они просто игнорируются.  
> А вот `mp3_track` и `mp3_volume` режим в SAILING **не** переключают — так задумано.

---

## Режимы (mode state machine)

| Режим | Когда | Дашборд |
|---|---|---|
| **SEARCH** | нет связи, зондирует ~0.8 с | свёрнут |
| **SAILING** | команда < 30 с назад | полный |
| **CHARGING** | 30 с без команд + ток > 0 | полный |
| **IDLE** | 30 с без команд + ток ≤ 0 | полный |
| **OFF** | `enabled = 0` | свёрнут |

**Что происходит при выходе в онлайн:** все DUTY → 0, затем FREQ → 400 Гц, моторы → 40, свет → 0.  
**В режиме OFF:** GPIO MODn=1 переводит LoRa в config-mode. Состояние сохраняется в `/etc/ship-driver-state.json`.

### Матрица периодов опроса

| Параметр | SEARCH | SAILING | IDLE | CHARGING |
|---|---|---|---|---|
| `battery_current` | ~0.8 с | 5 с | **5 с** ✓ | 5 с |
| `battery_temperature` | — | 5 мин | 5 мин | 10 с |
| `charge_level` | — | 5 мин | 5 мин | 1 мин |
| `motors` | — | 1 мин | 5 мин | 1 мин |
| `lights` | — | 1 мин | 5 мин | 1 мин |

✓ подтверждено live: `13:53:24 → :29 → :34 → :40` (интервалы 5–6 с с учётом Modbus over LoRa overhead)

---

## Зарядные станции — `chargerN`

Опрашиваются по Modbus RTU через **TCP-шлюз** (EBYTE serial server, `host:port` задаётся в конфиге)
— отдельно от корабельной LoRa-шины. Управление — публикацией в `/on`, период опроса примерно 3 с.

| Контрол | Топик | ro/rw | Значение |
|---|---|---|---|
| `transmitter` | `/devices/chargerN/controls/transmitter` | rw | `0`/`1` — передатчик XKT-801 |
| `magnets` | `/devices/chargerN/controls/magnets` | rw | `0`/`1` — магниты фиксации |
| `transmitter_current` | `/devices/chargerN/controls/transmitter_current` | ro | ток передатчика, А (WB-MAI6 = падение на шунте / сопротивление) |

```bash
# включить передатчик и магниты на станции 1
mosquitto_pub -h 192.168.69.105 -t '/devices/charger1/controls/transmitter/on' -m 1
mosquitto_pub -h 192.168.69.105 -t '/devices/charger1/controls/magnets/on'     -m 1

# ток передатчика
mosquitto_sub -h 192.168.69.105 -t '/devices/charger1/controls/transmitter_current' -C 1
```

> Реле WB-MRM2-mini здесь нормально-замкнутые, поэтому в конфиге зарядки для каждого выхода включён
> `invert` — чтобы `1` означало «включено». При старте драйвер только показывает фактическое
> состояние реле, ничего не переключая.
