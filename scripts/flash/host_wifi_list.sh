#!/usr/bin/env bash
# Runs on the host (not in the container): prints the WiFi networks this computer has saved, one per line.
# just setup-flash passes the list to the flash script as a menu. Prints nothing if it can't tell.
# Note: a saved profile's name is usually, but not always, its SSID.

if command -v netsh.exe >/dev/null 2>&1; then
    # WSL: Windows' saved profiles. Doesn't need Location services, unlike scanning.
    netsh.exe wlan show profiles 2>/dev/null | tr -d '\r' \
        | awk '/^User profiles/ { on = 1; next } on && / : / { sub(/^[^:]*: /, ""); print }'
elif command -v nmcli >/dev/null 2>&1; then
    # Linux with NetworkManager.
    nmcli -t -f NAME,TYPE connection show 2>/dev/null | awk -F: '$2 == "802-11-wireless" { print $1 }'
fi
exit 0
