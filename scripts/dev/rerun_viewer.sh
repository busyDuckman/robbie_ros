#!/usr/bin/env bash
# just up (host side): prints where the sim logs to Rerun (sim.launch.py's rerun argument), from .env.
# Starts the viewer in ROBBIE_RERUN_EXE if it isn't running. Headless, nothing is started: the docker
# viewer, or a ROBBIE_RERUN_EXE one that isn't running, becomes a recording in .output/rerun/.
# Messages go to stderr. Usage: rerun_viewer.sh <headless: true|false>
set -euo pipefail
# shellcheck source=../setup/common.sh
. "$(dirname "$0")/../setup/common.sh"
# shellcheck disable=SC1091
[ -f "$repo_root/.env" ] && . "$repo_root/.env"
headless="$1"
sink="${ROBBIE_RERUN:-docker}" exe="${ROBBIE_RERUN_EXE:-}"
record="/ws/robbie_ros/.output/rerun/sim_$(date +%Y%m%d_%H%M%S).rrd"

if [ "$sink" = docker ]; then
    [ "$headless" = true ] && { echo "$record"; exit 0; }
    if port_open 127.0.0.1 9876; then
        warn "Something already listens on port 9876, so the docker Rerun viewer can't start." >&2
    fi
    echo docker
    exit 0
fi

port="${sink##*:}"
if [ -n "$exe" ] && ! port_open 127.0.0.1 "$port"; then
    [ "$headless" = true ] && { echo "$record"; exit 0; }
    args="--bind 127.0.0.1 --port $port --memory-limit 25%"
    echo "Starting the Rerun viewer: $exe" >&2
    if [[ "$exe" == *.exe ]]; then   # on Windows, from WSL. Start-Process, so it outlives this shell.
        (cd /mnt/c && powershell.exe -NoProfile -Command "Start-Process -FilePath '$(wslpath -w "$exe")' -ArgumentList '$args'" </dev/null) >&2
    else
        # shellcheck disable=SC2086
        setsid "$exe" $args >/dev/null 2>&1 </dev/null &
    fi
    for _ in $(seq 1 30); do
        port_open 127.0.0.1 "$port" && break
        sleep 0.5
    done
    port_open 127.0.0.1 "$port" || warn "Started $exe, but nothing listens on port $port yet." >&2
fi
echo "$sink"
