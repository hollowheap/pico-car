# comms test records

### 2026-11-26 — comms — Wi-Fi recovery

- Build: 3c299b5+dirty
- Kind: host test
- Setup: CTest against the CYW43 and lwIP fakes
- Steps: `ctest --test-dir build-tests -L '^comms$' --output-on-failure`
- Expected: RSSI refreshes every 1 s (2 s after a failed read); a DHCP lease without DNS still joins
- Observed: all 40 comms cases pass, including `comms.wifi_recovery.wifi_rssi_refresh` and `comms.wifi_recovery.wifi_lease_without_dns`. With no DNS server, `comms.mqtt_session.telemetry_mqtt_dns-unavailable` logs one warning and keeps retrying.
- Result: pass
- Evidence: CTest output
- Follow-up: confirm the RSSI refresh on the car
