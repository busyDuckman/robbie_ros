# Shared helpers for the setup scripts. Source it, don't run it.

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

step() { printf '\n\033[34m== %s\033[0m\n' "$*"; }
warn() { printf '\033[33mWARNING: %s\033[0m\n' "$*"; }
die()  { printf '\033[31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

# ask "question": returns 0 on yes
ask() {
    local answer
    read -r -p "$1 [y/N]: " answer || true
    case "$answer" in y|Y|yes|YES) return 0 ;; *) return 1 ;; esac
}

# port_open HOST PORT: returns 0 if something is listening there
port_open() {
    timeout 1 bash -c "exec 3<>/dev/tcp/$1/$2" 2>/dev/null
}

# The Rerun version robbie_ros logs with; a viewer must match it.
rerun_sdk_version() {
    sed -n 's/^rerun-sdk==//p' "$repo_root/scripts/setup/requirements.txt" | tr -d '\r'
}

save_setting() {  # save_setting NAME VALUE: sets it now and saves it to .env
    local env_file="$repo_root/.env"
    printf -v "$1" '%s' "$2"
    touch "$env_file"
    grep -v "^$1=" "$env_file" > "$env_file.tmp" || true
    printf '%s=%q\n' "$1" "$2" >> "$env_file.tmp"
    mv "$env_file.tmp" "$env_file"
    chmod 600 "$env_file"   # it can hold the WiFi password
}
