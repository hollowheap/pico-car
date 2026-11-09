#!/usr/bin/env bash
set -euo pipefail

# cd to directory containing this script
DIR=$(dirname -- "${BASH_SOURCE[0]}")
cd "$DIR"

git submodule update --init --recursive

# Detect the active Wi-Fi and the broker host unless the caller supplied
# them with -D flags or the environment.
if [ -z "${PICO_CAR_WIFI_SSID:-}" ] && command -v nmcli >/dev/null 2>&1; then
    active=$(nmcli -t -f ACTIVE,SSID,FREQ dev wifi 2>/dev/null |
             sed -n 's/^yes://p' | head -n1 || true)
    ssid=${active%%:*}
    freq=${active##*:}
    freq_mhz=${freq%% *}
    conn=$(nmcli -t -f NAME,DEVICE,TYPE,STATE connection show --active 2>/dev/null |
           awk -F: '$3=="802-11-wireless"{print $1; exit}' || true)
    keymgmt=""
    if [ -n "$conn" ]; then
        keymgmt=$(nmcli -g 802-11-wireless-security.key-mgmt connection show "$conn" 2>/dev/null || true)
    fi

    # The CYW43439 in the Pico W is 2.4 GHz only.  A host on a 5/6 GHz BSS
    # would otherwise bake an unusable SSID into the firmware, unless the same
    # SSID is also broadcast on 2.4 GHz (dual-band SSID).
    low_24=$(nmcli -t -f SSID,FREQ dev wifi 2>/dev/null |
             awk -F: -v want="$ssid" \
                 '$1==want && $2+0 > 0 && $2+0 < 2500 {print "yes"; exit}' || true)
    if [ -n "$ssid" ] && [ -z "$low_24" ] &&
        [ "$freq_mhz" -ge 2500 ] 2>/dev/null; then
        echo "configure: Wi-Fi '$ssid' is $freq; the Pico W CYW43439 is 2.4 GHz only" >&2
        networks=$(nmcli -t -f SSID,FREQ dev wifi 2>/dev/null |
                   awk -F: '$2+0 > 0 && $2+0 < 2500 {print $1}' |
                   sort -u | paste -sd, - || true)
        if [ -n "$networks" ]; then
            echo "configure: 2.4 GHz SSIDs in range: $networks" >&2
        fi
        echo "configure: Wi-Fi join stays off; connect to a 2.4 GHz SSID or pass -D flags" >&2
    elif [ -n "$ssid" ]; then
        case "$keymgmt" in
            wpa-psk|"")
                psk=$(nmcli -s -g 802-11-wireless-security.psk connection show "$conn" 2>/dev/null || true)
                if [ -z "$psk" ] && [ -t 0 ]; then
                    read -r -s -p "Wi-Fi password for '$ssid': " psk
                    echo
                fi
                if [ -n "$psk" ]; then
                    export PICO_CAR_WIFI_SSID="$ssid"
                    export PICO_CAR_WIFI_PASSWORD="$psk"
                    export PICO_CAR_WIFI_AUTH="CYW43_AUTH_WPA2_AES_PSK"
                    echo "configure: Wi-Fi '$ssid' (WPA2-PSK)"
                else
                    echo "configure: no saved password for '$ssid'; Wi-Fi join stays off" >&2
                fi
                ;;
            none)
                export PICO_CAR_WIFI_SSID="$ssid"
                export PICO_CAR_WIFI_PASSWORD=""
                export PICO_CAR_WIFI_AUTH="CYW43_AUTH_OPEN"
                echo "configure: Wi-Fi '$ssid' (open)"
                ;;
            wpa-eap|wpa-eap-suite-b-192|sae)
                echo "configure: Wi-Fi '$ssid' uses '$keymgmt'; the CYW43 supports open/WPA2-PSK only" >&2
                echo "configure: Wi-Fi join stays off; use a WPA2 hotspot or pass -D flags" >&2
                ;;
            *)
                echo "configure: Wi-Fi '$ssid' has unsupported auth '$keymgmt'; join stays off" >&2
                ;;
        esac
    fi
fi

# Private address of the host running the MQTT broker.
if [ -z "${PICO_CAR_MQTT_HOST:-}" ]; then
    iface=$(nmcli -t -f DEVICE,TYPE,STATE dev 2>/dev/null |
            awk -F: '$2=="wifi" && $3=="connected"{print $1; exit}' || true)
    ipv4=""
    if [ -n "$iface" ]; then
        ipv4=$(ip -4 -o addr show dev "$iface" 2>/dev/null |
               awk '{print $4}' | cut -d/ -f1 | head -n1 || true)
    fi
    if [ -z "$ipv4" ]; then
        ipv4=$(ip -4 route get 1.1.1.1 2>/dev/null |
               awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}' || true)
    fi
    if [ -n "$ipv4" ]; then
        export PICO_CAR_MQTT_HOST="$ipv4"
        echo "configure: MQTT broker host default $ipv4"
    fi
fi

# Extra arguments pass through to cmake.
# e.g., ./configure.sh -DCMAKE_BUILD_TYPE=Release
cmake -DPICO_BOARD=pico_w -S . -B build --fresh "$@"
