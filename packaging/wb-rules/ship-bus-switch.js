// ship-bus-switch.js — dashboard toggle between ship-driver and wb-mqtt-serial on the ship bus.
//
// ship-driver and wb-mqtt-serial Conflict (systemd) — only one may own the RS485/MOD bus at a time.
// This virtual switch flips them via systemctl. It lives in wb-rules (outside both services) so it
// can start either one — including starting ship-driver back up while wb-mqtt-serial holds the bus.
//
// Logic is idempotent: it acts only when the actual service state differs from the requested one,
// so a wb-rules engine reload (which re-runs this file) never restarts the services on its own.

function shipActiveAsync(cb) {
    runShellCommand("systemctl is-active ship-driver", {
        captureOutput: true,
        exitCallback: function (code, out) {
            cb(("" + out).replace(/\s/g, "") === "active");
        }
    });
}

function updateActive() {
    shipActiveAsync(function (ship) {
        dev["ship_bus/active"] = ship ? "ship-driver" : "wb-mqtt-serial";
    });
}

defineVirtualDevice("ship_bus", {
    title: { en: "Ship bus driver", ru: "Драйвер шины корабля" },
    cells: {
        ship_driver: {
            type: "switch",
            title: { en: "Ship driver (off = wb-mqtt-serial)", ru: "Ship driver (выкл = wb-mqtt-serial)" },
            value: true,
            order: 1
        },
        active: {
            type: "text",
            title: { en: "Active service", ru: "Активный сервис" },
            value: "",
            readonly: true,
            order: 2
        }
    }
});

defineRule("ship_bus_switch", {
    whenChanged: "ship_bus/ship_driver",
    then: function (newValue) {
        shipActiveAsync(function (shipActive) {
            if (newValue === shipActive) { updateActive(); return; }   // already in requested state
            if (newValue) {
                runShellCommand("systemctl stop wb-mqtt-serial; systemctl start ship-driver");
            } else {
                runShellCommand("systemctl stop ship-driver; systemctl start wb-mqtt-serial");
            }
            setTimeout(updateActive, 3000);
        });
    }
});

// On startup sync the switch to the real service state (idempotent rule => safe if this triggers it).
setTimeout(function () {
    shipActiveAsync(function (shipActive) {
        dev["ship_bus/ship_driver"] = shipActive;
        updateActive();
    });
}, 2000);
