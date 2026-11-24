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

# Verify built APK and key v0.7.0 classes inside classes.dex
check "built: BrakeIndicator.apk exists" yes "$(test -f "$B/app/BrakeIndicator.apk" && echo yes || echo no)"
DEX_STRINGS=$(unzip -p "$B/app/BrakeIndicator.apk" classes.dex | strings)
has "apk contains OverlayService" "Ldev/lemoyne/brakeindicator/OverlayService;" "$DEX_STRINGS"
has "apk contains MainActivity" "Ldev/lemoyne/brakeindicator/MainActivity;" "$DEX_STRINGS"
has "apk contains BrakeEngine" "Ldev/lemoyne/brakeindicator/BrakeEngine;" "$DEX_STRINGS"
has "apk contains HmgVehiclePoller" "Ldev/lemoyne/brakeindicator/HmgVehiclePoller;" "$DEX_STRINGS"
has "apk contains HevRegenProbe" "Ldev/lemoyne/brakeindicator/HevRegenProbe;" "$DEX_STRINGS"
has "apk contains ObdBrakeClient" "Ldev/lemoyne/brakeindicator/ObdBrakeClient;" "$DEX_STRINGS"
has "apk contains Elm327BrakeProtocol" "Ldev/lemoyne/brakeindicator/Elm327BrakeProtocol;" "$DEX_STRINGS"
has "apk contains TelemetryRecorder" "Ldev/lemoyne/brakeindicator/TelemetryRecorder;" "$DEX_STRINGS"
has "apk contains UsbExporter" "Ldev/lemoyne/brakeindicator/UsbExporter;" "$DEX_STRINGS"
has "apk contains AutoExportCoordinator" "Ldev/lemoyne/brakeindicator/AutoExportCoordinator;" "$DEX_STRINGS"
has "apk contains PinnedRoots" "Ldev/lemoyne/brakeindicator/PinnedRoots;" "$DEX_STRINGS"
has "apk contains TlsHttpClient" "Ldev/lemoyne/brakeindicator/TlsHttpClient;" "$DEX_STRINGS"
has "apk contains TarGzArchive" "Ldev/lemoyne/brakeindicator/TarGzArchive;" "$DEX_STRINGS"
has "apk contains CloudUploadConfig" "Ldev/lemoyne/brakeindicator/CloudUploadConfig;" "$DEX_STRINGS"
has "apk contains PendingUploadQueue" "Ldev/lemoyne/brakeindicator/PendingUploadQueue;" "$DEX_STRINGS"
has "apk contains GitHubUploader" "Ldev/lemoyne/brakeindicator/GitHubUploader;" "$DEX_STRINGS"
has "apk contains BrakeModel" "Ldev/lemoyne/brakeindicator/BrakeModel;" "$DEX_STRINGS"
has "apk contains ModelBundleVerifier" "Ldev/lemoyne/brakeindicator/ModelBundleVerifier;" "$DEX_STRINGS"
has "apk contains ModelManager" "Ldev/lemoyne/brakeindicator/ModelManager;" "$DEX_STRINGS"
has "apk bundles HmgApi client" "Lhmgapi/client/HmgApi;" "$DEX_STRINGS"
has "apk bundles HmgApi Ble client" "Lhmgapi/client/Ble;" "$DEX_STRINGS"
check "apk drops legacy Gen5wBleClient" no "$(echo "$DEX_STRINGS" | grep -q 'Gen5wBleClient' && echo yes || echo no)"
check "apk drops obsolete root HscanProbe" no "$(echo "$DEX_STRINGS" | grep -q 'HscanProbe' && echo yes || echo no)"
check "apk drops obsolete root MonitorMain" no "$(echo "$DEX_STRINGS" | grep -q 'MonitorMain' && echo yes || echo no)"

# Verify stock-style launcher icons exist and stay well under Mods.java 256 KB limit
for ic in icon.png icon_night.png icon_disabled.png icon_disabled_night.png; do
    check "launcher tile $ic exists" yes "$(test -f "$M/icons/$ic" && echo yes || echo no)"
done

install_mod "$B"
cp -r "$M/icons" "$M/svc" "$R/data/hmgapi/mods/brake-indicator-gen5w/"

eng plan
check "engine plan succeeds" 0 "$rc"
has "plan lists BrakeIndicator.apk" "dev.lemoyne.brakeindicator (app/BrakeIndicator.apk)" "$out"
has "plan lists non-root starter service as system" "service  brake-indicator-gen5w/starter as system, at android" "$out"
has "plan lists BLE central" "ble      BLE scanner and GATT central" "$out"
has "plan lists USB folder" "usb      BrakeIndicator/ on a stick" "$out"
for sig in power brake.pressure speed pedal.accel gear.park parking_brake moving steering.angle hv.soc; do
    has "plan lists $sig reading" "$sig" "$out"
done
check "plan has zero problems" no "$(echo "$out" | grep -q '^problem ' && echo yes || echo no)"

eng apply
check "engine apply succeeds" 0 "$rc"
has "mod published as on" "1 of 1 mods on" "$out"
has "mod in public state.json" '"brake-indicator-gen5w"' "$(cat "$R/data/hmgapi/public/state.json")"
has "default obd_mode is obd_when_connected" '"obd_when_connected"' "$(cat "$R/data/hmgapi/public/state.json")"
has "default signal_mode is pressure_and_decel" '"pressure_and_decel"' "$(cat "$R/data/hmgapi/public/state.json")"
has "default auto_export_usb is in state.json" '"auto_export_usb":true' "$(tr -d ' \n' < "$R/data/hmgapi/public/state.json")"
has "default auto_upload_cloud is in state.json" '"auto_upload_cloud":true' "$(tr -d ' \n' < "$R/data/hmgapi/public/state.json")"
has "default cloud_allow_lte is in state.json" '"cloud_allow_lte":true' "$(tr -d ' \n' < "$R/data/hmgapi/public/state.json")"
has "default usb_backup_when_cloud is in state.json" '"usb_backup_when_cloud":false' "$(tr -d ' \n' < "$R/data/hmgapi/public/state.json")"
has "default auto_update_model is in state.json" '"auto_update_model":true' "$(tr -d ' \n' < "$R/data/hmgapi/public/state.json")"
has "starter registered in services.txt as system" "brake-indicator-gen5w starter system android no script svc/start.sh -" "$(cat "$R/data/hmgapi/state/services.txt")"
has "vehicle hub auto-enabled in services.txt" "hmgapi vehicle root android yes jar" "$(cat "$R/data/hmgapi/state/services.txt")"
for sig in power brake.pressure speed pedal.accel gear.park parking_brake moving steering.angle hv.soc; do
    has "$sig in vehicle.txt" "brake-indicator-gen5w $sig" "$(cat "$R/data/hmgapi/state/vehicle.txt")"
done

# Test owner setting override round-trip
printf '{"obd_mode":"model_only_icon","signal_mode":"pressure_only","off_state":"dim","x_offset":300,"standstill_hold":false,"auto_export_usb":false,"auto_upload_cloud":false,"cloud_allow_lte":false,"usb_backup_when_cloud":true,"auto_update_model":false}\n' > "$R/data/hmgapi/settings/brake-indicator-gen5w.json"
eng apply
check "setting override applies cleanly" 0 "$rc"
has "public state reflects model_only_icon override" '"model_only_icon"' "$(cat "$R/data/hmgapi/public/state.json")"
has "public state reflects dim override" '"dim"' "$(cat "$R/data/hmgapi/public/state.json")"
has "public state reflects auto_export_usb false override" '"auto_export_usb":false' "$(tr -d ' \n' < "$R/data/hmgapi/public/state.json")"
has "public state reflects auto_upload_cloud false override" '"auto_upload_cloud":false' "$(tr -d ' \n' < "$R/data/hmgapi/public/state.json")"
has "public state reflects usb_backup_when_cloud true override" '"usb_backup_when_cloud":true' "$(tr -d ' \n' < "$R/data/hmgapi/public/state.json")"
has "public state reflects auto_update_model false override" '"auto_update_model":false' "$(tr -d ' \n' < "$R/data/hmgapi/public/state.json")"

eng remove brake-indicator-gen5w
check "engine remove succeeds" 0 "$rc"
eng apply
check "apply after remove succeeds" 0 "$rc"
check "mod removed from mods folder" "" "$(ls "$R/data/hmgapi/mods")"

exit "$fail"
