#!/usr/bin/env bash
# Runs on the Pi as root: once on first boot (from cloud-init), and again from just deploy.
# Installs ROS 2 Jazzy, hardware access (GPIO, I2C, SPI) and Docker. Safe to re-run.
# Usage: provision.sh <user>
set -euo pipefail
user="${1:?usage: provision.sh <user>}"
export DEBIAN_FRONTEND=noninteractive
log() { echo "[robbie provision] $*"; }

log "ROS 2 apt source"
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q "install ok installed"; then
    apt-get update
    apt-get install -y curl software-properties-common
    add-apt-repository -y universe
    # Latest release, not pinned: the package exists so ROS can rotate its signing key.
    ver="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
    deb="$(mktemp --suffix=.deb)"
    curl -fL -o "$deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.noble_all.deb"
    dpkg -i "$deb"
    rm -f "$deb"
fi
apt-get update

log "Packages"
apt-get install -y \
    ros-jazzy-ros-base ros-dev-tools \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro \
    build-essential cmake git \
    gpiod libgpiod-dev i2c-tools libi2c-dev \
    docker.io docker-compose-v2

log "rosdep"
[ -e /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
sudo -u "$user" rosdep update

log "Hardware access"
for g in gpio i2c spi; do getent group "$g" >/dev/null || groupadd --system "$g"; done
cat > /etc/udev/rules.d/99-robbie-hw.rules <<'EOF'
# robbie_ros: let the gpio, i2c and spi groups use the hardware without root.
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0660"
SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"
EOF
udevadm control --reload-rules
udevadm trigger
usermod -aG gpio,i2c,spi,docker,dialout "$user"

log "Shell"
bashrc="/home/$user/.bashrc"
grep -q "/opt/ros/jazzy/setup.bash" "$bashrc" 2>/dev/null || echo "source /opt/ros/jazzy/setup.bash" >> "$bashrc"

mkdir -p /var/lib/robbie
date -Iseconds > /var/lib/robbie/provisioned
log "Done. Log in again for the new groups to apply."
