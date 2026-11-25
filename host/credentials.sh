#!/usr/bin/env bash
# Create the local MQTT credentials shared by the broker, the car build and the
# bridge. Existing passwords are kept; only missing files are generated.
#
# Output directory (gitignored): host/credentials, or PICO_CAR_CREDENTIALS_DIR.
#   car.env     firmware user and password, read by configure.sh
#   bridge.env  bridge user and password, read by the bridge and compose
#   passwd      broker password file with hashed entries
#   acl         broker topic permissions for the two users
#
# The topic prefix comes from MQTT_TOPIC_PREFIX or PICO_CAR_TOPIC_PREFIX.
# Delete car.env or bridge.env to replace that password; passwd is rebuilt
# with it. Rerun configure.sh to put a new car password into the firmware.
set -euo pipefail
umask 077

HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
OUT=${PICO_CAR_CREDENTIALS_DIR:-$HERE/credentials}
PREFIX=${MQTT_TOPIC_PREFIX:-${PICO_CAR_TOPIC_PREFIX:-car/PICO-CAR}}
SCHEMA=$HERE/../protocol/telemetry.yaml
CAR_USER=car
BRIDGE_USER=bridge

mkdir -p "$OUT"

new_password() {
    head -c 24 /dev/urandom | od -An -tx1 | tr -d ' \n'
}

write_env() {
    local file=$1 user_key=$2 user=$3 pass_key=$4
    if [ ! -s "$OUT/$file" ]; then
        printf '%s=%s\n%s=%s\n' "$user_key" "$user" "$pass_key" "$(new_password)" >"$OUT/$file"
        rm -f "$OUT/passwd"
    fi
}

read_value() {
    sed -n "s/^$2=//p" "$OUT/$1" | head -n 1
}

write_env car.env PICO_CAR_MQTT_USERNAME "$CAR_USER" PICO_CAR_MQTT_PASSWORD
write_env bridge.env MQTT_USERNAME "$BRIDGE_USER" MQTT_PASSWORD

if [ ! -s "$OUT/passwd" ]; then
    command -v mosquitto_passwd >/dev/null || {
        echo "credentials: mosquitto_passwd not found; run inside nix develop" >&2
        exit 1
    }
    plain=$(mktemp "$OUT/passwd.XXXXXX")
    trap 'rm -f "$plain"' EXIT
    printf '%s:%s\n%s:%s\n' \
        "$(read_value car.env PICO_CAR_MQTT_USERNAME)" "$(read_value car.env PICO_CAR_MQTT_PASSWORD)" \
        "$(read_value bridge.env MQTT_USERNAME)" "$(read_value bridge.env MQTT_PASSWORD)" >"$plain"
    mosquitto_passwd -U "$plain"
    chmod 600 "$plain"
    mv "$plain" "$OUT/passwd"
    trap - EXIT
fi

command_topic=$(sed -n '/^topics:/,/^[^ ]/s/^  command: *"\(.*\)"/\1/p' "$SCHEMA")
[ -n "$command_topic" ] || { echo "credentials: no command topic in $SCHEMA" >&2; exit 1; }
{
    echo "user $CAR_USER"
    sed -n '/^topics:/,/^[^ ]/s/^  [a-z_]*: *"\(.*\)"/\1/p' "$SCHEMA" |
        while IFS= read -r suffix; do
            if [ "$suffix" = "$command_topic" ]; then
                echo "topic read $PREFIX/$suffix"
            else
                echo "topic write $PREFIX/$suffix"
            fi
        done
    echo
    echo "user $BRIDGE_USER"
    echo "topic read $PREFIX/#"
    echo "topic write $PREFIX/$command_topic"
} >"$OUT/acl"
chmod 600 "$OUT/acl"
echo "credentials: $OUT"
