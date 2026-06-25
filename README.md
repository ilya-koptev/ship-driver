# ship-driver

Custom ship-bus **LoRa driver for Wiren Board**. Owns the 4 MOD LoRa transmitters
(`/dev/ttyMOD1..4`), talks Modbus to the boat's `pwm8a04` + `WB-UPS-v3`, injects DFPlayer
frames inline, runs a per-channel mode state machine, configures the LoRa modems, and
mirrors everything to MQTT (devices `boat1..4` + `Ship Setup`).

Full driver documentation: **`README.html`** (open in a browser).

This repo also ships an **APT repository** (built by GitHub Actions, hosted on GitHub Pages)
so the driver installs and updates with `apt`.

---

## Install on the controller (one-time)

```sh
echo "deb [trusted=yes] https://ilya-koptev.github.io/ship-driver ./" | sudo tee /etc/apt/sources.list.d/ship-driver.list
sudo apt update
sudo apt install ship-driver
```

That installs the driver to `/usr/bin/ship-driver.py`, the unit to
`/lib/systemd/system/ship-driver.service`, pulls deps (`python3-serial`, `python3-paho-mqtt`,
`mosquitto`), and enables + starts the service. If a manual install existed
(`/usr/local/bin/ship-driver.py` + `/etc/systemd/system/ship-driver.service`), the package
removes it during install.

## Update

```sh
sudo apt update && sudo apt upgrade
```

## Manage

```sh
systemctl status|restart|stop|start ship-driver
journalctl -u ship-driver -f
```

---

## Releasing a new version (maintainer)

1. Edit the driver (`ship-driver.py`) / unit / docs.
2. **Bump `VERSION`** (e.g. `1.0.1`) — apt only upgrades to a higher version.
3. `git commit` + `git push` to `main`.
4. The GitHub Action **build-and-publish-apt** builds `ship-driver_<VERSION>_all.deb`,
   regenerates the apt repo, and deploys it to GitHub Pages.
5. On the controller: `sudo apt update && sudo apt upgrade`.

## Repo setup (one-time, on GitHub)

- Repository must be **public** (for GitHub Pages + apt over https without auth).
- Settings → **Pages** → Source = **GitHub Actions**.
- The workflow needs Pages write permission (already declared in the workflow).

The apt repo is currently **unsigned** (`[trusted=yes]`). To add GPG signing later, sign
`Release` in the workflow and ship the public key.

## License

[MIT](LICENSE) © 2026 Ilya Koptev
