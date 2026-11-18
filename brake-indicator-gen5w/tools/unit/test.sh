#!/bin/sh
# Host Java unit test runner (picked up automatically by HMG-API's mod.yml workflow).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
MOD=$(cd "$HERE/../.." && pwd)
SRC="$MOD/app/java/dev/lemoyne/brakeindicator"
STUBS="$HERE/stubs"
if [ -z "${HMGAPI_SDK:-}" ]; then
    HMGAPI_SDK=$(cd "$MOD/../../HMG-API/out/sdk" && pwd)
fi
CLIENT_JAR="$HMGAPI_SDK/lib/hmgapi-client.jar"
W=$(mktemp -d); trap 'rm -rf "$W"' EXIT

javac --release 8 -Xlint:-options -cp "$CLIENT_JAR" -d "$W" \
    "$STUBS/android/net/LocalSocketAddress.java" \
    "$STUBS/android/net/LocalSocket.java" \
    "$STUBS/org/json/JSONObject.java" \
    "$SRC/IndicatorConfig.java" \
    "$SRC/BrakeEvaluator.java" \
    "$SRC/HmgVehiclePoller.java" \
    "$SRC/HevRegenProbe.java" \
    "$SRC/Elm327BrakeProtocol.java" \
    "$SRC/TelemetryRecorder.java" \
    "$SRC/UsbExporter.java" \
    "$SRC/PinnedRoots.java" \
    "$SRC/TlsHttpClient.java" \
    "$SRC/TarGzArchive.java" \
    "$SRC/CloudUploadConfig.java" \
    "$SRC/PendingUploadQueue.java" \
    "$SRC/GitHubUploader.java" \
    "$SRC/AutoExportCoordinator.java" \
    "$SRC/ObdBrakeClient.java" \
    "$HERE/TestMain.java"

java -cp "$W:$CLIENT_JAR" dev.lemoyne.brakeindicator.TestMain
