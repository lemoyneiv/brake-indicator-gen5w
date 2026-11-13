#!/bin/sh
# Off-car engine & package structure test for brake-indicator-gen5w using the HMG-API Mod SDK.
# Run via:
#   HMGAPI_SDK=<sdk> HMGAPI_MOD_BUILD=<built mod> sh brake-indicator-gen5w/test.sh
set -e
M=$(cd "$(dirname "$0")" && pwd)
B=${HMGAPI_MOD_BUILD:?set HMGAPI_MOD_BUILD to the built mod (the out dir of bin/build-mod.sh)}
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
R=$T/root
. "${HMGAPI_SDK:?set HMGAPI_SDK to the unpacked mod SDK}/test/lib.sh"

mkdir -p "$R/system" "$R/data/hmgapi/settings"
printf 'ro.board.platform=tcc803x\n' > "$R/props.txt"

# Verify built APK and key classes inside classes.dex
check "built: BrakeIndicator.apk exists" yes "$(test -f "$B/app/BrakeIndicator.apk" && echo yes || echo no)"
DEX_STRINGS=$(unzip -p "$B/app/BrakeIndicator.apk" classes.dex | strings)
has "apk contains MonitorMain" "Ldev/lemoyne/brakeindicator/MonitorMain;" "$DEX_STRINGS"
has "apk contains OverlayService" "Ldev/lemoyne/brakeindicator/OverlayService;" "$DEX_STRINGS"
has "apk contains MainActivity" "Ldev/lemoyne/brakeindicator/MainActivity;" "$DEX_STRINGS"
has "apk contains UsbExporter" "Ldev/lemoyne/brakeindicator/UsbExporter;" "$DEX_STRINGS"
has "apk bundles HmgApi client" "Lhmgapi/client/HmgApi;" "$DEX_STRINGS"

# Verify stock-style launcher icons exist and stay well under Mods.java 256 KB limit
for ic in icon.png icon_night.png icon_disabled.png icon_disabled_night.png; do
    check "launcher tile $ic exists" yes "$(test -f "$M/icons/$ic" && echo yes || echo no)"
done

install_mod "$B"
cp -r "$M/icons" "$R/data/hmgapi/mods/brake-indicator-gen5w/"

eng plan
check "engine plan succeeds" 0 "$rc"
has "plan lists BrakeIndicator.apk" "dev.lemoyne.brakeindicator (app/BrakeIndicator.apk)" "$out"
has "plan lists monitor service as root" "service  brake-indicator-gen5w/monitor as root, at android" "$out"
has "plan lists USB folder" "usb      BrakeIndicator/ on a stick" "$out"
has "plan lists brake.pressure reading" "brake.pressure" "$out"
check "plan has zero problems" no "$(echo "$out" | grep -q '^problem ' && echo yes || echo no)"

eng apply
check "engine apply succeeds" 0 "$rc"
has "mod published as on" "1 of 1 mods on" "$out"
has "mod in public state.json" '"brake-indicator-gen5w"' "$(cat "$R/data/hmgapi/public/state.json")"
has "default signal_mode is native_and_decel" '"native_and_decel"' "$(cat "$R/data/hmgapi/public/state.json")"
has "monitor registered in services.txt" "brake-indicator-gen5w monitor root android yes jar app/BrakeIndicator.apk dev.lemoyne.brakeindicator.MonitorMain" "$(cat "$R/data/hmgapi/state/services.txt")"
has "vehicle hub auto-enabled in services.txt" "hmgapi vehicle root android yes jar" "$(cat "$R/data/hmgapi/state/services.txt")"
has "brake.pressure in vehicle.txt" "brake-indicator-gen5w brake.pressure" "$(cat "$R/data/hmgapi/state/vehicle.txt")"

# Test owner setting override round-trip
printf '{"signal_mode":"native","off_state":"dim","x_offset":300,"standstill_hold":false}\n' > "$R/data/hmgapi/settings/brake-indicator-gen5w.json"
eng apply
check "setting override applies cleanly" 0 "$rc"
has "public state reflects dim override" '"dim"' "$(cat "$R/data/hmgapi/public/state.json")"

eng remove brake-indicator-gen5w
check "engine remove succeeds" 0 "$rc"
eng apply
check "apply after remove succeeds" 0 "$rc"
check "mod removed from mods folder" "" "$(ls "$R/data/hmgapi/mods")"

exit "$fail"
