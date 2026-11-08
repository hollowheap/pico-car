#!/usr/bin/env bash
# Start and supervise the local ground station: MQTT broker, bridge, and
# dashboard. The supervisor restarts a child that exits. Press Ctrl-C to stop.
set -euo pipefail

DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$DIR"

BROKER_CONFIG="$DIR/host/broker/mosquitto.conf"
BROKER_PORT=1883
BRIDGE_PORT="${BRIDGE_PORT:-8090}"
EXPO_PORT="${EXPO_PORT:-8083}"
EXPO_CLEAR="${EXPO_CLEAR:-0}"
LOG_DIR="${PICO_CAR_LOG_DIR:-$DIR/logs}"
STATE_DIR="${PICO_CAR_STATE_DIR:-$LOG_DIR/state}"
SUPERVISOR_LOG="$LOG_DIR/supervisor.log"

BROKER_PID=""
BRIDGE_PID=""
EXPO_PID=""

for tool in mosquitto npm node curl ip ss setsid; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "missing $tool; run 'nix develop' or install it" >&2
        exit 1
    fi
done

if [ ! -f "$BROKER_CONFIG" ]; then
    echo "missing $BROKER_CONFIG" >&2
    exit 1
fi

mkdir -p "$LOG_DIR" "$STATE_DIR"
export MQTT_URL="mqtt://127.0.0.1:$BROKER_PORT"
export BRIDGE_HTTP_PORT="$BRIDGE_PORT"
export BRIDGE_HISTORY_PATH="$STATE_DIR/history.jsonl"
export EXPO_PUBLIC_ALLOW_INSECURE=1

# Active Wi-Fi address, matching configure.sh. Fall back to the route address.
iface=""
if command -v nmcli >/dev/null 2>&1; then
    iface=$(nmcli -t -f DEVICE,TYPE,STATE dev 2>/dev/null |
            awk -F: '$2=="wifi" && $3=="connected"{print $1; exit}' || true)
fi
HOST_IP=""
if [ -n "$iface" ]; then
    HOST_IP=$(ip -4 -o addr show dev "$iface" 2>/dev/null |
              awk '{print $4}' | cut -d/ -f1 | head -n1 || true)
fi
if [ -z "$HOST_IP" ]; then
    HOST_IP=$(ip -4 route get 1.1.1.1 2>/dev/null |
              awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}' || true)
fi
if [ -z "$HOST_IP" ]; then
    echo "no IPv4 address; connect to Wi-Fi first" >&2
    exit 1
fi

export EXPO_PUBLIC_LINK_URL="ws://$HOST_IP:$BRIDGE_PORT"
export EXPO_PUBLIC_SERVICE_URL="http://$HOST_IP:$BRIDGE_PORT"

log() {
    printf '%s %s\n' "$(date -Is)" "$*" | tee -a "$SUPERVISOR_LOG" >&2
}

port_listening() {
    ss -ltn 2>/dev/null | grep -q ":$1 "
}

bridge_ready() {
    curl -fsS "http://127.0.0.1:$BRIDGE_PORT/health" 2>/dev/null |
        grep -q '"mqtt":"connected"'
}

expo_ready() {
    curl -fsS "http://127.0.0.1:$EXPO_PORT/" >/dev/null 2>&1
}

stop_group() {
    local pid="${1:-}"
    local _
    if [ -z "$pid" ]; then
        return
    fi
    kill -TERM -- "-$pid" 2>/dev/null || true
    for _ in {1..20}; do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
    done
    kill -KILL -- "-$pid" 2>/dev/null || true
}

cleanup() {
    trap - EXIT INT TERM
    stop_group "$EXPO_PID"
    stop_group "$BRIDGE_PID"
    stop_group "$BROKER_PID"
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

start_broker() {
    local _
    stop_group "$BROKER_PID"
    BROKER_PID=""
    setsid mosquitto -c "$BROKER_CONFIG" -v >"$LOG_DIR/broker.log" 2>&1 &
    BROKER_PID=$!
    for _ in {1..40}; do
        if port_listening "$BROKER_PORT" && kill -0 "$BROKER_PID" 2>/dev/null; then
            return 0
        fi
        sleep 0.25
    done
    log "broker failed to start"
    tail -n 20 "$LOG_DIR/broker.log" >&2
    return 1
}

start_bridge() {
    local _
    stop_group "$BRIDGE_PID"
    BRIDGE_PID=""
    setsid bash -c 'cd host/bridge && exec ./node_modules/.bin/tsx src/index.ts' \
        >"$LOG_DIR/bridge.log" 2>&1 &
    BRIDGE_PID=$!
    for _ in {1..40}; do
        if bridge_ready && kill -0 "$BRIDGE_PID" 2>/dev/null; then
            return 0
        fi
        kill -0 "$BRIDGE_PID" 2>/dev/null || break
        sleep 0.25
    done
    log "bridge did not connect"
    tail -n 20 "$LOG_DIR/bridge.log" >&2
    return 1
}

start_expo() {
    local _
    local -a args=(--port "$EXPO_PORT")
    if [ "$EXPO_CLEAR" = "1" ]; then
        args+=(--clear)
    fi
    stop_group "$EXPO_PID"
    EXPO_PID=""
    setsid bash -c 'cd host/dashboard && exec npm run web -- "$@"' _ "${args[@]}" \
        >"$LOG_DIR/expo.log" 2>&1 &
    EXPO_PID=$!
    for _ in {1..120}; do
        if expo_ready && kill -0 "$EXPO_PID" 2>/dev/null; then
            return 0
        fi
        kill -0 "$EXPO_PID" 2>/dev/null || break
        sleep 0.5
    done
    log "dashboard failed to start"
    tail -n 20 "$LOG_DIR/expo.log" >&2
    return 1
}

start_broker
start_bridge
start_expo

cat <<EOF
broker    mqtt://$HOST_IP:$BROKER_PORT
bridge    http://$HOST_IP:$BRIDGE_PORT
dashboard http://$HOST_IP:$EXPO_PORT
logs      $LOG_DIR
EOF

if command -v systemctl >/dev/null 2>&1 &&
    systemctl is-active --quiet firewall.service 2>/dev/null; then
    echo "note: allow TCP $BROKER_PORT $BRIDGE_PORT $EXPO_PORT on wlan0" >&2
fi

while true; do
    sleep 2
    if ! port_listening "$BROKER_PORT"; then
        log "broker lost; restarting"
        start_broker || true
    fi
    if ! bridge_ready; then
        log "bridge lost; restarting"
        start_bridge || true
    fi
    if ! expo_ready; then
        log "dashboard lost; restarting"
        start_expo || true
    fi
done
