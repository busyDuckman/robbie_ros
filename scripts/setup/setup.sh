#!/usr/bin/env bash
# just setup (host side): checks the host and the robbie checkout. just then builds the dev container.
# The host needs only Docker and just (ADR 2). Safe to re-run.
set -euo pipefail
# shellcheck source=common.sh
. "$(dirname "$0")/common.sh"

step "Platform"
[ "$(uname -s)" = "Linux" ] || die "Run just from Linux or a WSL 2 shell (ADR 2)."
if grep -qi microsoft /proc/version 2>/dev/null; then
    echo "OK: WSL 2"
    if [[ "$repo_root" == /mnt/* ]]; then
        warn "The repo is on a Windows drive ($repo_root). It will work better in a WSL filesystem, e.g. under /home."
    fi
    bash "$(dirname "$0")/wsl_networking.sh"
else
    echo "OK: Linux"
fi

step "Docker"
command -v docker >/dev/null || die "Docker isn't installed. On WSL 2, install Docker Desktop with WSL integration, or Docker Engine in the distro."
docker info >/dev/null 2>&1 || die "Can't reach the Docker daemon. Start Docker, or add yourself to the docker group (then log in again)."
docker compose version >/dev/null 2>&1 || die "Docker Compose v2 is missing (the 'docker compose' plugin)."
echo "OK: $(docker --version)"

step "robbie repo"
robbie_dir="$(cd "$repo_root/.." && pwd)/robbie"
if [ -d "$robbie_dir/.git" ]; then
    echo "Found $robbie_dir"
elif ask "robbie isn't checked out at $robbie_dir. Clone it now?"; then
    read -r -p "Clone over HTTPS or SSH? [https/ssh]: " proto || true
    case "$proto" in
        ssh|SSH) url=git@github.com:busyDuckman/robbie.git ;;
        *)       url=https://github.com/busyDuckman/robbie.git ;;
    esac
    git clone "$url" "$robbie_dir"
else
    die "robbie is needed at $robbie_dir (robbie_interfaces). The dev container mounts it."
fi

step "Rerun viewer"
bash "$(dirname "$0")/rerun_setup.sh"

step "Dev container"
echo "Building the image (the first build downloads a few GB)..."
