#!/bin/sh
# Host Java unit test runner (picked up automatically by HMG-API's mod.yml workflow).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
MOD=$(cd "$HERE/../.." && pwd)
SRC="$MOD/app/java/dev/lemoyne/brakeindicator"
W=$(mktemp -d); trap 'rm -rf "$W"' EXIT

javac --release 8 -Xlint:-options -d "$W" \
    "$SRC/CandidateFilter.java" \
    "$SRC/IndicatorConfig.java" \
    "$SRC/BrakeEvaluator.java" \
    "$SRC/IpcProtocol.java" \
    "$SRC/UsbExporter.java" \
    "$HERE/TestMain.java"

java -cp "$W" dev.lemoyne.brakeindicator.TestMain
