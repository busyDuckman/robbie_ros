#!/usr/bin/env bash
# Runs on the host (not in the container): prints the first SSH public key it finds, for the Pi's
# authorized_keys. Looks in ~/.ssh, then (on WSL) the Windows user's .ssh. Prints nothing if none.

dirs=("$HOME/.ssh")
if command -v cmd.exe >/dev/null 2>&1 && command -v wslpath >/dev/null 2>&1; then
    win_home="$(cd /mnt/c 2>/dev/null && cmd.exe /c 'echo %USERPROFILE%' 2>/dev/null | tr -d '\r')"
    [ -n "$win_home" ] && dirs+=("$(wslpath "$win_home")/.ssh")
fi

for dir in "${dirs[@]}"; do
    for key in id_ed25519.pub id_ecdsa.pub id_rsa.pub; do
        if [ -f "$dir/$key" ]; then
            cat "$dir/$key"
            exit 0
        fi
    done
done
exit 0
