# ship-driver

Драйвер корабельной шины для контроллеров Wiren Board: четыре канала LoRa
(`/dev/ttyMOD1..4`), устройства `boat1..4` и зарядные станции `charger1..N` в MQTT.
На корабельной шине заменяет собой `wb-mqtt-serial`.

## Установка

MOD-слоты с LoRa-модулями сперва перевести в тип **LoRa** в конфигурации Wiren Board,
иначе `/dev/ttyMODn` не откроется.

```sh
echo "deb [trusted=yes] https://ilya-koptev.github.io/ship-driver ./" | sudo tee /etc/apt/sources.list.d/ship-driver.list
sudo apt update && sudo apt install ship-driver
```

Кладётся `/usr/bin/ship-driver.py`, юнит `ship-driver.service`, конфиг
`/etc/ship-driver.conf` (dpkg conffile) и схема для веб-редактора.

## Обновление

```sh
sudo apt update && sudo apt upgrade
```

Если dpkg спросит про `/etc/ship-driver.conf` — оставлять свой, иначе затрётся список
кораблей и зарядок. Из скрипта: `-o Dpkg::Options::="--force-confold"`.

## Управление

MQTT: чтение `/devices/<устройство>/controls/<контрол>`, команда — тот же топик с `/on`.

## Релиз

1. Правки в `ship-driver.py` / схеме / конфиге.
2. Поднять `VERSION` — apt обновляет только на бóльшую.
3. `git push` в `main`: Action собирает `.deb`, пересобирает apt-репозиторий и
   публикует на GitHub Pages.
4. На контроллере `apt update && apt upgrade`.

Для перепубликации делать новый коммит, а не «Re-run jobs» — иначе дубли артефактов.

**Репозиторий должен оставаться публичным:** с него по https ставятся контроллеры.

## Файлы

| Файл | Что |
|---|---|
| `ship-driver.py` | сам драйвер |
| `ship-driver.conf`, `ship-driver.schema.json` | конфиг и схема веб-редактора |
| `packaging/` | сборка пакета, правила `wb-rules`, сбор логов |
| `Ship PCB/` | плата корабля и её BOM |
| [`BUILD.md`](BUILD.md) | где лежит спецификация платы |

Описание работы драйвера, режимов и железа ведётся отдельно и в репозиторий
не выносится.
