# Setup Guide

## Required Components

- Raspberry Pi 5 (16 GB)
- Micro SD card (32 GB+)
- Power supply for Pi (27W / 5.1V 5A)
- Luxonis OAK-D-SR-POE camera
- PoE injector (12V barrel jack input)
- 12V power supply for PoE injector
- M12/RJ45 Ethernet Cable
- SparkFun NEO-M9N GPS module
- USB SSD
- USB Wi-Fi adapter (BrosTrend AX900)
- Camera hood mount

## 1. Raspberry Pi OS Installation

Open Raspberry Pi Imager on your computer.

1. **Device:** Raspberry Pi 5
2. **OS:** Raspberry Pi OS (other) → Raspberry Pi OS Lite (64-bit)
3. **Storage:** Select your micro SD card
4. **Edit Settings:**
   - Hostname: `road-profiler`
   - Username: `nline`, Password: `Potheads`
   - Wi-Fi: enter your network SSID and password
   - Locale: leave default
   - Enable SSH with password authentication
   - Skip Raspberry Pi Connect
5. **Write** the image to the SD card

Insert SD card into Pi and boot.

## 2. System Packages

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git curl build-essential python3-dev libusb-1.0-0-dev cmake exfatprogs
```

## 3. Python Environment

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
echo 'source $HOME/.local/bin/env' >> ~/.bashrc
uv python install 3.12
```

## 4. SSD Setup

Plug in the SSD and find the device name:

```bash
lsblk
```

Format as exFAT (replace `sdX1` with your actual partition — e.g. `sda1`):

> **WARNING:** This erases everything on the SSD.

```bash
sudo mkfs.exfat -n road-data /dev/sdX1
```

Create mount point and auto-mount on boot:

```bash
sudo mkdir -p /mnt/ssd
echo 'LABEL=road-data /mnt/ssd exfat defaults,noatime,uid=1000,gid=1000 0 0' | sudo tee -a /etc/fstab
sudo mount -a
```

Reload systemctl:

```bash
systemctl daemon-reload
```

Create session data directory:

```bash
sudo mkdir -p /mnt/ssd/sessions
```

Verify:

```bash
df -h /mnt/ssd
```

## 5. Camera Setup

```bash
sudo nmcli connection add type ethernet ifname eth0 \
  con-name oak-poe \
  ipv4.method manual \
  ipv4.addresses 169.254.1.100/16 \
  ipv6.method disabled \
  connection.autoconnect yes

sudo nmcli connection up oak-poe
```

Verify camera is reachable (plug in PoE injector + camera first):

```bash
ping -c 3 169.254.1.222
```

## 6. GPS Setup

Plug in the NEO-M9N via USB, then check vendor/product ID:

```bash
lsusb | grep -i u-blox
```

Add user to dialout group:

```bash
sudo usermod -aG dialout $USER
```

Create symlink rule (adjust `idVendor`/`idProduct` if different from `lsusb` output):

```bash
sudo tee /etc/udev/rules.d/99-gps.rules <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666", GROUP="dialout"
EOF

sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify:

```bash
ls -la /dev/gps
```

## 7. WiFi AP Setup

### BrosTrend AX900 (needs driver install)

```bash
sh -c 'wget linux.brostrend.com/install -O /tmp/install && yes | bash /tmp/install'
```

### Lock the default Wi-Fi connection to wlan0

The Wi-Fi network configured during OS installation (via netplan) may claim wlan1 on boot, blocking the AP. Lock it to wlan0:

```bash
# Find the netplan Wi-Fi connection name
nmcli -t -f NAME,TYPE connection show | grep wireless

# Lock it to wlan0 (replace YOUR_WIFI_CONNECTION with the name from above)
sudo nmcli connection modify "YOUR_WIFI_CONNECTION" connection.interface-name wlan0
```

### Create the AP profile (works with either adapter)

```bash
sudo nmcli connection add type wifi ifname wlan1 \
  con-name "road-profiler-ap" \
  wifi.mode ap \
  wifi.ssid "ROAD-PROFILER" \
  wifi.band bg \
  wifi.channel 6 \
  ipv4.method shared \
  ipv4.addresses 192.168.4.1/24 \
  connection.autoconnect-priority 100

sudo nmcli connection up road-profiler-ap
```

Verify: connect phone to "ROAD-PROFILER".

## 8. Project Setup

From your development machine, copy the project to the Pi:

```bash
scp -r road-profiler/ nline@road-profiler.local:~/road-profiler
```

Then on the Pi:

```bash
cd ~/road-profiler
```

Create project venv with Python 3.12:

```bash
uv venv --python 3.12
source .venv/bin/activate
```

Install dependencies:

```bash
uv pip install .
```

## 9. Service Setup

```bash
sudo tee /etc/systemd/system/road-profiler.service <<EOF
[Unit]
Description=nLine Road Profiler
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=/home/$USER/road-profiler
ExecStart=/home/$USER/road-profiler/.venv/bin/python -m src.main
Restart=on-failure
RestartSec=5
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable road-profiler.service
```

To start/stop/check manually:

```bash
sudo systemctl start road-profiler
sudo systemctl stop road-profiler
sudo systemctl status road-profiler
journalctl -u road-profiler -f   # live logs
```

## 10. Reboot

```bash
sudo reboot now
```
