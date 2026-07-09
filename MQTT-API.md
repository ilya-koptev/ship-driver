# ship-driver — MQTT API

**Брокер:** `192.168.69.105:1883`  
**Активный канал:** `boat4` (LoRa CH19, корабль №9)  
**Контроллер:** WB 7.3.4, s/n AJUF3LS5

---

## Устройства MQTT

| Устройство | Топик | Назначение |
|---|---|---|
| `boat1..4` | `/devices/boatN/…` | Оперативный API канала + визуализация |
| `ship_setup` | `/devices/ship_setup/…` | Преднастройка LoRa-модема корабля по проводу (RS485-1) |
| `charger1..N` | `/devices/chargerN/…` | Беспроводные зарядные станции (реле передатчика/магнитов + ток) |

Дашборды `boatN` создаются **только для тех MOD-слотов, где обнаружен LoRa-модем** (зондирование при старте). На текущем контроллере активен только `boat4`.

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
| `battery_current` | `/devices/boat4/controls/battery_current` | ro | A, float · сейчас −0.144 A · период 5 с в IDLE |
| `battery_temperature` | `/devices/boat4/controls/battery_temperature` | ro | °C · сейчас 26.9 °C · 5 мин в IDLE, 10 с в CHARGING |
| `charge_level` | `/devices/boat4/controls/charge_level` | ro | % · сейчас 47.8 % · 5 мин в IDLE |
| `input_voltage` | `/devices/boat4/controls/input_voltage` | ro | В · входное напряжение ИБП · период 5 с (читается вместе с током) |
| `back_left` / `front_left` / `back_right` / `front_right` | `/devices/boat4/controls/{name}` | rw | `40–80` (40 = холостой ход) |
| `nav_lights` / `morse_lamp` / `deck_lights` / `cabin_light1` / `cabin_light2` | `/devices/boat4/controls/{name}` | rw | `0–100` % |
| `mp3_track` | `/devices/boat4/controls/mp3_track` | rw | `0` = стоп, `1–15` = трек |
| `mp3_volume` | `/devices/boat4/controls/mp3_volume` | rw | `0–30` |

> **Важно:** контролы моторов, света и батареи видны на дашборде только пока канал онлайн (режимы SAILING / CHARGING / IDLE). В SEARCH и OFF они убираются и возвращаются при выходе в онлайн.

### Запись (publish → `/on`)

Все команды записи публикуются в топик `/devices/boat4/controls/<ctrl>/on`.

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

> Команды моторов/света работают только в режимах **SAILING / CHARGING / IDLE**. В SEARCH/OFF они игнорируются.  
> Команды `mp3_track` и `mp3_volume` **не** переключают режим в SAILING — намеренно.

---

## Режимы (mode state machine)

| Режим | Когда | Дашборд |
|---|---|---|
| **SEARCH** | нет связи, зондирует ~0.8 с | свёрнут |
| **SAILING** | команда < 30 с назад | полный |
| **CHARGING** | 30 с без команд + ток > 0 | полный |
| **IDLE** | 30 с без команд + ток ≤ 0 | полный |
| **OFF** | `enabled = 0` | свёрнут |

**Инициализация при выходе в онлайн:** все DUTY → 0, FREQ → 400 Гц, моторы → 40, свет → 0.  
**В OFF:** GPIO MODn=1 → LoRa в config-mode. Состояние сохраняется в `/etc/ship-driver-state.json`.

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

Опрашиваются по Modbus RTU через **TCP-шлюз** (EBYTE serial server, `host:port` в конфиге),
независимо от корабельной LoRa-шины. Управление — публикацией в `/on`. Период опроса ~3 с.

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

> Реле WB-MRM2-mini в NC-исполнении — в конфиге зарядки для каждого выхода стоит `invert`, чтобы
> `1` = включено. При старте драйвер только отражает фактическое состояние реле, не переключая его.
