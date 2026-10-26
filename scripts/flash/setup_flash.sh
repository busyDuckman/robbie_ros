#!/usr/bin/env bash
# just setup-flash (runs in the dev container): builds a ready-to-flash SD card image for the Pi 4 at .output/robbie_ros.img
#
#   - Downloads the official Ubuntu 24.04 Server (arm64+raspi) image, checks its SHA256, and caches it
#     in .third_party/ubuntu/.
#   - Writes cloud-init files (user, SSH key, hostname, WiFi) and robbie's config.txt lines into the
#     image's boot partition. Uses mtools, so no root and no loop mounts.
#   - On first boot the Pi runs provision.sh, which installs ROS 2 Jazzy and the hardware access.
#
# Settings come from .env (see .env.example); anything missing is asked for and saved.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../setup/common.sh
. "$here/../setup/common.sh"

release_url="https://cdimage.ubuntu.com/releases/24.04/release"
image_pattern="preinstalled-server-arm64+raspi.img.xz"
# Ubuntu CD Image Automatic Signing Key (2012), from https://ubuntu.com/tutorials/how-to-verify-ubuntu
ubuntu_cdimage_key="843938DF228D22F7B3742BC0D94AA3F0EFE21092"
cache_dir="$repo_root/.third_party/ubuntu"
out_dir="$repo_root/.output"
out_img="$out_dir/robbie_ros.img"
env_file="$repo_root/.env"

step "Tools"
for tool in curl gpg xz sfdisk mcopy mtype mmd sha256sum openssl python3; do
    command -v "$tool" >/dev/null || die "$tool is missing. Run this through just setup-flash (it runs in the dev container)."
done
export MTOOLS_SKIP_CHECK=1

step "Settings (.env)"
# shellcheck disable=SC1090
[ -f "$env_file" ] && . "$env_file"

read_secret() {  # read_secret VAR "question": hidden input that echoes * per character
    local c s=""
    printf '%s: ' "$2"
    while IFS= read -r -s -n1 c; do
        if [ -z "$c" ]; then
            break                                           # Enter
        elif [ "$c" = $'\x7f' ] || [ "$c" = $'\b' ]; then   # Backspace
            [ -n "$s" ] && { s="${s%?}"; printf '\b \b'; }
        else
            s+="$c"; printf '*'
        fi
    done
    echo
    printf -v "$1" '%s' "$s"
}

prompt_setting() {  # prompt_setting NAME "question" [default] [-s]
    local name="$1" question="$2" default="${3:-}" secret="${4:-}" value
    [ -n "${!name:-}" ] && return 0
    if [ "$secret" = "-s" ]; then
        read_secret value "$question"
    else
        read -r -p "$question${default:+ [$default]}: " value
    fi
    value="${value:-$default}"
    [ -n "$value" ] || die "$name is required."
    save_setting "$name" "$value"
}

prompt_setting ROBBIE_HOSTNAME "Pi hostname" "robbie-ros"
prompt_setting ROBBIE_USER "Pi user name" "robbie"
# Offer the host's saved WiFi networks (just passes them in, one per line) to save typing odd SSIDs.
if [ -z "${ROBBIE_WIFI_SSID:-}" ] && [ -n "${ROBBIE_WIFI_KNOWN:-}" ]; then
    mapfile -t known <<< "$ROBBIE_WIFI_KNOWN"
    echo "WiFi networks saved on this computer:"
    for i in "${!known[@]}"; do printf '  %d) %s\n' "$((i + 1))" "${known[$i]}"; done
    echo "  0) type another name"
    read -r -p "Network for the Pi to join (2.4 or 5 GHz) [0]: " choice
    if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le "${#known[@]}" ]; then
        save_setting ROBBIE_WIFI_SSID "${known[$((choice - 1))]}"
    fi
fi
prompt_setting ROBBIE_WIFI_SSID "WiFi network for the Pi to join (SSID, 2.4 or 5 GHz)"
prompt_setting ROBBIE_WIFI_PASSWORD "Password for that WiFi network" "" -s
prompt_setting ROBBIE_WIFI_COUNTRY "WiFi country code (e.g. AU, GB, US)"

if [ -z "${ROBBIE_SSH_PUBKEY:-}" ]; then
    # just finds a key on the host (host_ssh_key.sh) and passes it in; the container can't see ~/.ssh.
    [ -n "${ROBBIE_SSH_PUBKEY_DEFAULT:-}" ] || die "No SSH public key found on the host (id_ed25519, id_ecdsa or id_rsa in ~/.ssh). Create one with: ssh-keygen -t ed25519"
    save_setting ROBBIE_SSH_PUBKEY "$ROBBIE_SSH_PUBKEY_DEFAULT"
fi

if [ -z "${ROBBIE_PASSWORD_HASH:-}" ]; then
    echo "Pick a password for $ROBBIE_USER. It's for the console only; SSH uses your key."
    read_secret pw1 "Console password"
    read_secret pw2 "Same again"
    [ -n "$pw1" ] || die "The console password can't be empty."
    [ "$pw1" = "$pw2" ] || die "The passwords don't match."
    save_setting ROBBIE_PASSWORD_HASH "$(printf '%s\n' "$pw1" | openssl passwd -6 -stdin)"
    unset pw1 pw2
fi
echo "Using $ROBBIE_USER@$ROBBIE_HOSTNAME.local on WiFi '$ROBBIE_WIFI_SSID' ($ROBBIE_WIFI_COUNTRY)."

step "Ubuntu 24.04 Server image"
mkdir -p "$cache_dir" "$out_dir"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

# SHA256SUMS must be signed by Ubuntu's CD image key, so a tampered mirror can't pass the checksum.
curl -fsSL -o "$work/SHA256SUMS" "$release_url/SHA256SUMS"
curl -fsSL -o "$work/SHA256SUMS.gpg" "$release_url/SHA256SUMS.gpg"
export GNUPGHOME="$work/gnupg"
mkdir -m 700 "$GNUPGHOME"
gpg --batch --quiet --keyserver hkps://keyserver.ubuntu.com --recv-keys "$ubuntu_cdimage_key"
gpg --batch --status-fd 1 --verify "$work/SHA256SUMS.gpg" "$work/SHA256SUMS" 2>/dev/null \
    | grep -q "^\[GNUPG:\] VALIDSIG $ubuntu_cdimage_key " \
    || die "SHA256SUMS isn't signed by the Ubuntu CD image key. Not using it."
echo "SHA256SUMS signature OK."

# The list holds several point releases (24.04.3, .4, .5 ...); take the newest.
line="$(grep -F "$image_pattern" "$work/SHA256SUMS" | sort -V -k2 | tail -n1)"
[ -n "$line" ] || die "No $image_pattern listed at $release_url/SHA256SUMS"
expected_sha="${line%% *}"
image_name="${line##* }"; image_name="${image_name#\*}"
cached="$cache_dir/$image_name"

if [ -f "$cached" ] && echo "$expected_sha  $cached" | sha256sum -c --status; then
    echo "Using cached $image_name"
else
    echo "Downloading $image_name"
    curl -fL --progress-bar -o "$cached.part" "$release_url/$image_name"
    mv "$cached.part" "$cached"
    echo "$expected_sha  $cached" | sha256sum -c - || { rm -f "$cached"; die "Checksum mismatch, deleted the download."; }
fi

step "Unpacking"
tmp_img="$out_img.part"
xz -dcvT0 "$cached" > "$tmp_img"

# Byte offset of the first (FAT boot) partition, for mtools' img@@offset syntax.
offset="$(sfdisk -J "$tmp_img" | python3 -c '
import json, sys
t = json.load(sys.stdin)["partitiontable"]
print(t["partitions"][0]["start"] * t.get("sectorsize", 512))')"
boot="$tmp_img@@$offset"
mtype -i "$boot" ::user-data >/dev/null 2>&1 || die "No user-data on the boot partition. Is this a raspi image?"

step "Boot files"

yq() { local s="${1//\'/\'\'}"; printf "'%s'" "$s"; }  # single-quoted YAML string

cat > "$work/user-data" <<EOF
#cloud-config
# Written by just setup-flash (robbie_ros).
hostname: $(yq "$ROBBIE_HOSTNAME")
manage_etc_hosts: true
users:
  - name: $(yq "$ROBBIE_USER")
    groups: [adm, sudo, dialout, video, plugdev]
    shell: /bin/bash
    sudo: ALL=(ALL) NOPASSWD:ALL
    lock_passwd: false
    passwd: $(yq "$ROBBIE_PASSWORD_HASH")
    ssh_authorized_keys:
      - $(yq "$ROBBIE_SSH_PUBKEY")
ssh_pwauth: false
package_update: true
packages:
  - avahi-daemon
runcmd:
  - [bash, /boot/firmware/robbie/provision.sh, $(yq "$ROBBIE_USER")]
EOF

cat > "$work/network-config" <<EOF
# Written by just setup-flash (robbie_ros).
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      regulatory-domain: $(yq "$ROBBIE_WIFI_COUNTRY")
      access-points:
        $(yq "$ROBBIE_WIFI_SSID"):
          password: $(yq "$ROBBIE_WIFI_PASSWORD")
EOF

mtype -i "$boot" ::config.txt > "$work/config.txt"
if ! grep -q "robbie_ros" "$work/config.txt"; then
    cat >> "$work/config.txt" <<'EOF'

# --- robbie_ros (added by just setup-flash) ---
[all]
dtparam=spi=on
dtoverlay=spi1-1cs
dtparam=i2c_arm=on
# Extra I2C buses for the AS5600s, one sensor per bus.
# Pins aren't decided yet (q_and_a.md Q9). Check for clashes with the eyes before enabling.
#dtoverlay=i2c3
#dtoverlay=i2c4
#dtoverlay=i2c6
EOF
fi

mcopy -o -i "$boot" "$work/user-data" ::user-data
mcopy -o -i "$boot" "$work/network-config" ::network-config
mcopy -o -i "$boot" "$work/config.txt" ::config.txt
mmd -i "$boot" ::robbie 2>/dev/null || true  # already there on a re-run
mcopy -o -i "$boot" "$here/provision.sh" ::robbie/provision.sh
mcopy -o -i "$boot" "$repo_root/pi/justfile" ::robbie/justfile

mv "$tmp_img" "$out_img"

step "Done"
echo "Image: $out_img"
echo "Flash it with Raspberry Pi Imager ('Use custom', no OS customisation) or dd."
echo "First boot installs ROS 2 and takes a while. Then: ssh $ROBBIE_USER@$ROBBIE_HOSTNAME.local"
echo "Progress on the Pi: tail -f /var/log/cloud-init-output.log"
