#!/usr/bin/env bash
# WSL only: checks WSL uses mirrored networking, and offers to switch it in %USERPROFILE%\.wslconfig.
# With the default NAT mode, WSL can't resolve the Pi's .local address (mDNS doesn't cross the NAT).
set -euo pipefail
# shellcheck source=common.sh
. "$(dirname "$0")/common.sh"

mode="$(wslinfo --networking-mode 2>/dev/null || echo unknown)"
if [ "$mode" = "mirrored" ]; then
    echo "OK: WSL networking is mirrored"
    exit 0
fi

win_home="$(cd /mnt/c 2>/dev/null && cmd.exe /c 'echo %USERPROFILE%' 2>/dev/null | tr -d '\r')"
[ -n "$win_home" ] || { warn "WSL networking is '$mode', and the Windows home folder couldn't be found to change it."; exit 0; }
cfg="$(wslpath "$win_home")/.wslconfig"

mode_line='^[[:space:]]*networkingMode[[:space:]]*='
if grep -qiE "${mode_line}[[:space:]]*mirrored" "$cfg" 2>/dev/null; then
    if [ "$mode" = "unknown" ]; then
        echo "OK: $cfg sets mirrored networking"
    else
        warn "$cfg sets mirrored networking, but WSL hasn't restarted since. For it to take effect, close WSL terminals and run from Windows: wsl --shutdown"
    fi
    exit 0
fi

warn "WSL networking is '$mode'. WSL won't be able to resolve the Pi's .local address; mirrored mode is needed for that."
ask "Set networkingMode=mirrored in $cfg?" || { echo "Skipped. Use the Pi's IP address from WSL instead."; exit 0; }

if [ ! -f "$cfg" ]; then
    printf '[wsl2]\r\nnetworkingMode=mirrored\r\n' > "$cfg"
else
    cp "$cfg" "$cfg.bak"
    has_line=0
    grep -qiE "$mode_line" "$cfg" && has_line=1
    # Replace the existing networkingMode line; or add one under [wsl2]; or add a [wsl2] section.
    awk -v has_line="$has_line" '
        { sub(/\r$/, "") }
        has_line && tolower($0) ~ /^[[:space:]]*networkingmode[[:space:]]*=/ { print "networkingMode=mirrored\r"; done = 1; next }
        { print $0 "\r" }
        !has_line && !done && tolower($0) ~ /^[[:space:]]*\[wsl2\]/ { print "networkingMode=mirrored\r"; done = 1 }
        END { if (!done) printf "[wsl2]\r\nnetworkingMode=mirrored\r\n" }
    ' "$cfg" > "$cfg.new"
    mv "$cfg.new" "$cfg"
    echo "Backup of the old file: $cfg.bak"
fi
echo "Updated $cfg"
# Stop here, so this isn't lost in the Docker build output that follows.
die "WSL needs a restart for the change to take effect:
  1. Close all WSL terminals (including VS Code's WSL windows).
  2. In PowerShell or cmd, run: wsl --shutdown
  3. Re-run: just setup"
