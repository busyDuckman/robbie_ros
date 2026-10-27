#!/usr/bin/env bash
# just setup: picks the Rerun viewer just up logs the sim to, and saves it to .env:
#   ROBBIE_RERUN      docker (a viewer in the dev container), or host:port
#   ROBBIE_RERUN_EXE  a viewer found here (or on Windows, from WSL); just up starts it if it isn't running
# Looks for a viewer already listening on 9876, rerun on this system, and on WSL, rerun.exe on Windows.
set -euo pipefail
# All in braces, so bash reads the whole script before running it. It otherwise reads as it goes, and
# that read can fail ("No data available") if the file changes meanwhile, or on /mnt/c.
{
# shellcheck source=common.sh
. "$(dirname "$0")/common.sh"

# shellcheck disable=SC1091
[ -f "$repo_root/.env" ] && . "$repo_root/.env"
if [ -n "${ROBBIE_RERUN:-}" ]; then
    echo "Using $ROBBIE_RERUN${ROBBIE_RERUN_EXE:+ ($ROBBIE_RERUN_EXE)}, from .env."
    ask "Choose again?" || exit 0
fi
want="$(rerun_sdk_version)"

# Windows programs get </dev/null: they otherwise share the terminal's input with read below.
version_of() {  # version_of EXE: e.g. 0.40.0, or nothing
    "$1" --version 2>/dev/null </dev/null | grep -o 'rerun-cli [0-9][^ ]*' | head -n1 | cut -d' ' -f2 || true
}

labels=() sinks=() exes=() oks=()
add() {  # add NAME DETAILS SINK EXE VERSION: the label is "NAME (v VERSION) DETAILS"
    local v="(v $5)"
    [ -z "$5" ] && v="(version unknown)"
    [ -n "$5" ] && [ "$5" != "$want" ] && v="(v $5, but robbie_ros needs $want)"
    labels+=("$1 $v $2") sinks+=("$3") exes+=("$4") oks+=("$([ "$5" = "$want" ] && echo 1 || echo 0)")
}
state() {  # state PORT: how just up will use a viewer found on this machine
    if [ -n "$1" ]; then echo "running on port $1"; else echo "(launched as needed)"; fi
}

# On this system. A running viewer's port is from its command line (9876 unless --port says otherwise).
if exe="$(command -v rerun)"; then
    running="$(ps -eo args | grep -E '(^|/)rerun( |$)' | grep -v grep || true)"
    port=""
    if [ -n "$running" ]; then
        port="$(grep -oE -- '--port[ =][0-9]+' <<<"$running" | head -n1 | grep -oE '[0-9]+' || true)"
        port="${port:-9876}"
    fi
    add rerun "on this system, $exe, $(state "$port")" "127.0.0.1:${port:-9876}" "$exe" "$(version_of "$exe")"
fi

# On Windows, from WSL: on the PATH, or where pip, uv, pipx or cargo usually put it.
on_wsl=false
if grep -qi microsoft /proc/version 2>/dev/null && [ -e /proc/sys/fs/binfmt_misc/WSLInterop ]; then
    on_wsl=true
    win() { (cd /mnt/c && cmd.exe /c "$@" 2>/dev/null </dev/null) | tr -d '\r'; }
    win_exe="$(win where rerun.exe | head -n1)"
    [ -n "$win_exe" ] && win_exe="$(wslpath "$win_exe")"
    if [ -z "$win_exe" ]; then
        winpath() { wslpath "$(win echo "%$1%")" 2>/dev/null || echo /nonexistent; }
        appdata="$(winpath APPDATA)" local_appdata="$(winpath LOCALAPPDATA)" home="$(winpath USERPROFILE)"
        for f in "$appdata"/Python/Python3*/Scripts/rerun.exe \
                 "$local_appdata"/Programs/Python/Python3*/Scripts/rerun.exe \
                 "$local_appdata"/Packages/PythonSoftwareFoundation.Python.3*/LocalCache/local-packages/Python3*/Scripts/rerun.exe \
                 "$home"/.local/bin/rerun.exe "$home"/.cargo/bin/rerun.exe; do
            [ -f "$f" ] && { win_exe="$f"; break; }
        done
    fi
    win_ports="$( (cd /mnt/c && powershell.exe -NoProfile -Command 'Get-Process rerun -ErrorAction SilentlyContinue | ForEach-Object { Get-NetTCPConnection -State Listen -OwningProcess $_.Id -ErrorAction SilentlyContinue } | Select-Object -ExpandProperty LocalPort -Unique' 2>/dev/null </dev/null) | tr -d '\r' || true)"
    win_port="$(grep -x 9876 <<<"$win_ports" || head -n1 <<<"$win_ports")"
    if [ -n "$win_exe" ]; then
        add rerun.exe "on Windows, $win_exe, $(state "$win_port")" "127.0.0.1:${win_port:-9876}" "$win_exe" "$(version_of "$win_exe")"
    elif [ -n "$win_port" ]; then
        add rerun.exe "running on Windows, port $win_port" "127.0.0.1:$win_port" "" ""
    fi
    if [ -n "$win_exe$win_port" ] && [ "$(wslinfo --networking-mode 2>/dev/null || echo mirrored)" != mirrored ]; then
        warn "Use mirrored networking if you have rerun.io installed under Windows: the dev container can't reach it otherwise."
    fi
fi

# Anything else already listening on 9876. A running just up's docker viewer is there too; that's not one to pick.
if port_open 127.0.0.1 9876 && [[ " ${sinks[*]} " != *" 127.0.0.1:9876 "* ]] \
        && [ -z "$(docker ps -q --filter name=robbie_ros_sim 2>/dev/null)" ]; then
    add "a viewer" "already listening on 127.0.0.1:9876" "127.0.0.1:9876" "" ""
fi

[ ${#labels[@]} -gt 0 ] || echo "No Rerun viewer found on this system$($on_wsl && echo ", or on Windows")."
echo "The simulation uses Rerun $want or later.:"
echo "Select an option for connecting to Rerun:"
keys=() default=c
for i in "${!labels[@]}"; do
    key=a
    [ ${#labels[@]} -gt 1 ] && key="a$((i + 1))"
    keys+=("$key")
    [ "$default" = c ] && [ "${oks[$i]}" = 1 ] && default="$key"
    printf '  %-4s %s\n' "$key)" "${labels[$i]}"
done
printf '  %-4s %s\n' "b)" "connect via host:port" \
                     "c)" "use rerun hosted in a docker container (software rendering)"

while true; do
    read -r -p "Rerun viewer [$default]: " choice || true
    choice="${choice:-$default}"
    case "$choice" in
        b|B)
            read -r -p "host:port: " hp || true
            if [[ "$hp" =~ ^[^:[:space:]]+:[0-9]+$ ]]; then
                save_setting ROBBIE_RERUN "$hp"; save_setting ROBBIE_RERUN_EXE ""; break
            fi
            echo "Expected host:port, e.g. 192.168.1.20:9876" ;;
        c|C)
            save_setting ROBBIE_RERUN docker; save_setting ROBBIE_RERUN_EXE ""; break ;;
        *)
            for i in "${!keys[@]}"; do
                if [ "$choice" = "${keys[$i]}" ]; then
                    save_setting ROBBIE_RERUN "${sinks[$i]}"; save_setting ROBBIE_RERUN_EXE "${exes[$i]}"; break 2
                fi
            done
            echo "Pick one of: ${keys[*]} b c" ;;
    esac
done
echo "Saved ROBBIE_RERUN=$ROBBIE_RERUN${ROBBIE_RERUN_EXE:+ and ROBBIE_RERUN_EXE=$ROBBIE_RERUN_EXE} to .env"
exit
}
