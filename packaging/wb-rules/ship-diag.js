// ship-diag.js — дашборд "Сбор логов корабля".
// Выбираешь корабль (Все / номер из настроенных) + период в часах, жмёшь "Собрать логи" —
// драйвер собирает журнал + телеметрию из wb-mqtt-db + конфиг в /var/www/ship-logs-*.tar.gz,
// и в "Результат" показывает ссылку для скачивания из браузера.
// Ставится пакетом ship-driver вместе с /usr/bin/ship-collect-logs.

defineVirtualDevice("ship_diag", {
    title: { en: "Ship logs", ru: "Сбор логов корабля" },
    cells: {
        ship:    { type: "value", value: 0, readonly: false, order: 1, title: { ru: "Корабль", en: "Ship" } },
        hours:   { type: "value", value: 6, readonly: false, min: 1, max: 720, order: 2, title: { ru: "Период, часов", en: "Hours" } },
        collect: { type: "pushbutton", order: 3, title: { ru: "Собрать логи", en: "Collect logs" } },
        status:  { type: "text", value: "", readonly: true, order: 4, title: { ru: "Результат", en: "Result" } }
    }
});

// Список кораблей для выпадающего меню: 0 = Все, далее номера из /etc/ship-driver.conf (ships.list[].address).
runShellCommand(
    "python3 -c \"import json;print(' '.join(str(s.get('address')) for s in json.load(open('/etc/ship-driver.conf')).get('ships',{}).get('list',[]) if s.get('address') is not None))\"",
    { captureOutput: true, exitCallback: function (code, out) {
        var en = { "0": { en: "All", ru: "Все" } };
        ("" + out).trim().split(/\s+/).forEach(function (s) { if (s) en[s] = { en: "#" + s, ru: "№" + s }; });
        var meta = { type: "value", readonly: false, order: 1, title: { ru: "Корабль", en: "Ship" }, enum: en };
        publish("/devices/ship_diag/controls/ship/meta", JSON.stringify(meta), 2, true);
    } });

defineRule("ship_diag_collect", {
    whenChanged: "ship_diag/collect",
    then: function () {
        var v = dev["ship_diag"]["ship"];
        var ship = (!v || v === 0 || v === "0") ? "all" : ("" + v);
        var hours = dev["ship_diag"]["hours"] || 6;
        dev["ship_diag"]["status"] = "Собираю (" + (ship === "all" ? "все" : "№" + ship) + ", " + hours + " ч)…";
        runShellCommand("/usr/bin/ship-collect-logs '" + ship + "' '" + hours + "' 2>&1",
            { captureOutput: true, exitCallback: function (code, out) {
                var lines = ("" + out).trim().split("\n");
                var path = lines.length ? lines[lines.length - 1] : "";
                if (code === 0 && path.indexOf("/var/www/") === 0) {
                    dev["ship_diag"]["status"] = "Готово — скачать: " + path.replace("/var/www", "");
                } else {
                    dev["ship_diag"]["status"] = "Ошибка (код " + code + "): " + (lines.slice(-1)[0] || "");
                }
            } });
    }
});
