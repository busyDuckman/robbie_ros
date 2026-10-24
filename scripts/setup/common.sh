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
