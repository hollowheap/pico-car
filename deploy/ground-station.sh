#!/usr/bin/env bash
# Run the ground station in containers and put it on the tailnet.
#
#   deploy/ground-station.sh up      build, start, and map to tailnet ports
#   deploy/ground-station.sh down    remove the tailnet mappings and stop
#   deploy/ground-station.sh status  show the containers and the mappings
#
# The dashboard and the bridge each get the first free tailnet port from 8080
# up (tailnet only, never Funnel). A mapping that already points at the
# ground station is reused, so `up` again keeps the same addresses.
set -euo pipefail

DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
COMPOSE=(podman-compose -f "$DIR/compose.yaml")
export BRIDGE_LOCAL_PORT="${BRIDGE_LOCAL_PORT:-5280}"
export DASHBOARD_LOCAL_PORT="${DASHBOARD_LOCAL_PORT:-5281}"

for tool in podman-compose tailscale python3 curl; do
  command -v "$tool" >/dev/null 2>&1 || { echo "missing $tool" >&2; exit 1; }
done

# Print "<tailnet port of local port $1> <first free tailnet port>" (the first
# field is 0 when no mapping points at $1). $2 is a port already taken in this run.
pick_port() {
  tailscale serve status --json 2>/dev/null | python3 -c '
import json, subprocess, sys
local, taken = sys.argv[1], int(sys.argv[2])
raw = sys.stdin.read().strip()
cfg = json.loads(raw) if raw else {}
used = {int(p) for p in cfg.get("TCP", {})} | {taken}
for line in subprocess.run(["ss", "-ltnH"], capture_output=True, text=True).stdout.splitlines():
    f = line.split()
    if len(f) >= 4 and "tailscale" in f[3]:
        used.add(int(f[3].rsplit(":", 1)[1]))
mine = 0
for host, web in cfg.get("Web", {}).items():
    for h in web.get("Handlers", {}).values():
        if h.get("Proxy", "").endswith("127.0.0.1:" + local):
            mine = int(host.rsplit(":", 1)[1])
free = next(p for p in range(8080, 9000) if p not in used)
print(mine, free)
' "$1" "$2"
}

dns_name() {
  tailscale status --self --json | python3 -c 'import json,sys; print(json.load(sys.stdin)["Self"]["DNSName"].rstrip("."))'
}

up() {
  local host bridge_port dash_port mine free
  host=$(dns_name)
  read -r mine free < <(pick_port "$BRIDGE_LOCAL_PORT" 0)
  bridge_port=$(( mine > 0 ? mine : free ))
  read -r mine free < <(pick_port "$DASHBOARD_LOCAL_PORT" "$bridge_port")
  dash_port=$(( mine > 0 ? mine : free ))

  export DASHBOARD_LINK_URL="ws://$host:$bridge_port"
  export DASHBOARD_SERVICE_URL="http://$host:$bridge_port"
  "${COMPOSE[@]}" up -d --build

  tailscale serve --bg --http="$bridge_port" "http://127.0.0.1:$BRIDGE_LOCAL_PORT" >/dev/null
  tailscale serve --bg --http="$dash_port" "http://127.0.0.1:$DASHBOARD_LOCAL_PORT" >/dev/null

  for _ in $(seq 1 30); do
    curl -fsS "http://127.0.0.1:$BRIDGE_LOCAL_PORT/health" >/dev/null 2>&1 && break
    sleep 1
  done
  cat <<INFO
dashboard  http://$host:$dash_port   (tailnet only)
bridge     http://$host:$bridge_port   (tailnet only)
broker     mqtt://<this host's Wi-Fi address>:1883   (for the car)
INFO
}

down() {
  local mine free
  for local in "$BRIDGE_LOCAL_PORT" "$DASHBOARD_LOCAL_PORT"; do
    read -r mine free < <(pick_port "$local" 0)
    if [ "$mine" -gt 0 ]; then
      tailscale serve --http="$mine" off >/dev/null
    fi
  done
  DASHBOARD_LINK_URL=unused DASHBOARD_SERVICE_URL=unused "${COMPOSE[@]}" down
}

case "${1:-}" in
  up) up ;;
  down) down ;;
  status)
    DASHBOARD_LINK_URL=unused DASHBOARD_SERVICE_URL=unused "${COMPOSE[@]}" ps
    tailscale serve status
    ;;
  *) echo "usage: $0 up|down|status" >&2; exit 2 ;;
esac
