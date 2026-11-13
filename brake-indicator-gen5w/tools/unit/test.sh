#!/bin/sh
# Host Java unit test runner (picked up automatically by HMG-API's mod.yml workflow).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
MOD=$(cd "$HERE/../.." && pwd)
SRC="$MOD/app/java/dev/lemoyne/brakeindicator"
BLE="$MOD/app/java/gen5w/ble"
STUBS="$HERE/stubs"
W=$(mktemp -d); trap 'rm -rf "$W"' EXIT

javac --release 8 -Xlint:-options -d "$W" \
    "$STUBS/android/net/LocalSocketAddress.java" \
    "$STUBS/android/net/LocalSocket.java" \
    "$BLE/Gen5wBleAdvRecord.java" \
    "$BLE/Gen5wBleClient.java" \
    "$SRC/IndicatorConfig.java" \
    "$SRC/BrakeEvaluator.java" \
    "$SRC/Elm327BrakeProtocol.java" \
    "$SRC/TelemetryRecorder.java" \
    "$SRC/UsbExporter.java" \
    "$SRC/ObdBrakeClient.java" \
    "$HERE/TestMain.java"

java -cp "$W" dev.lemoyne.brakeindicator.TestMain
