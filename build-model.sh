#!/bin/sh
# Build, self-test, and ECDSA-sign a standalone OTA brake model plugin into dist-model/:
#   dist-model/brake-model-<model_version>.jar
#   dist-model/latest.json
#   dist-model/SHA256SUMS
#
# Uses the exact same MOD_KEY (HMGAPI_KEY) as build.sh so the host APK's pinned
# ModelBundleVerifier accepts the bundle over LTE/Wi-Fi or offline USB import.
set -e
cd "$(dirname "$0")"
MOD=brake-indicator-gen5w
META_JSON=${OTA_MODEL_JSON:-ota-model.json}

# Locate Android SDK (Linux or macOS Homebrew commandlinetools)
if [ -z "${ANDROID_HOME:-}" ]; then
    if [ -d "$HOME/Android" ]; then ANDROID_HOME="$HOME/Android"
    elif [ -d /opt/homebrew/share/android-commandlinetools ]; then ANDROID_HOME=/opt/homebrew/share/android-commandlinetools
    elif [ -d "$HOME/Library/Android/sdk" ]; then ANDROID_HOME="$HOME/Library/Android/sdk"
    fi
fi
export ANDROID_HOME

# Locate HMG-API Mod SDK
if [ -z "${HMGAPI_SDK:-}" ]; then
    if [ -d ../HMG-API/out/sdk ]; then
        HMGAPI_SDK=$(cd ../HMG-API/out/sdk && pwd)
    elif [ -x ../HMG-API/build.sh ]; then
        (cd ../HMG-API && ./build.sh sdk)
        HMGAPI_SDK=$(cd ../HMG-API/out/sdk && pwd)
    else
        echo "build-model.sh: set HMGAPI_SDK to an unpacked HMG-API mod SDK (or place ../HMG-API beside this repo)" >&2
        exit 1
    fi
fi
export HMGAPI_SDK
. "$HMGAPI_SDK/bin/env.sh"

if [ ! -f "${ANDROID_JAR:-}" ] || [ ! -x "${D8:-}" ]; then
    echo "build-model.sh: ANDROID_HOME ($ANDROID_HOME) missing platforms/android-*/android.jar or build-tools/*/d8" >&2
    exit 1
fi

# Locate or create persistent private signing key (MOD_KEY)
if [ -n "${HMGAPI_KEY:-}" ]; then
    KEY_DIR=$(dirname "$HMGAPI_KEY")
elif [ -f "$HOME/.hmgapi/author.key" ]; then
    KEY_DIR="$HOME/.hmgapi"
    HMGAPI_KEY="$KEY_DIR/author.key"
elif [ -d "$HOME/.gemini" ]; then
    KEY_DIR="$HOME/.gemini/hmgapi"
    HMGAPI_KEY="$KEY_DIR/author.key"
else
    KEY_DIR="$HOME/.hmgapi"
    HMGAPI_KEY="$KEY_DIR/author.key"
fi

umask 077
mkdir -p "$KEY_DIR"
if [ ! -f "$HMGAPI_KEY" ]; then
    java -cp "$HMGAPI_SDK/lib/hmgapi-engine.jar" hmgapi.engine.Signing keygen "$HMGAPI_KEY"
fi
umask 022

MVER=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["model_version"])' "$META_JSON")
MSEQ=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["model_seq"])' "$META_JSON")
MIN_API=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("min_host_api", 1))' "$META_JSON")
MAX_API=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("max_host_api", 1))' "$META_JSON")
ENTRY_CLASS=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["entry_class"])' "$META_JSON")

W=$(mktemp -d); trap 'rm -rf "$W"' EXIT
mkdir -p "$W/host_classes" "$W/gen" "$W/plugin_classes" "$W/payload"

SRC="$MOD/app/java/dev/lemoyne/brakeindicator"
STUBS="$MOD/tools/unit/stubs"
CLIENT_JAR="$HMGAPI_SDK/lib/hmgapi-client.jar"

echo "== 1. Compile host BrakeModel ABI & verifier tools =="
javac --release 8 -Xlint:-options -cp "$CLIENT_JAR" -d "$W/host_classes" \
    "$STUBS/android/net/LocalSocketAddress.java" \
    "$STUBS/android/net/LocalSocket.java" \
    "$STUBS/org/json/JSONObject.java" \
    "$SRC/IndicatorConfig.java" \
    "$SRC/BrakeModel.java" \
    "$SRC/BrakeEvaluator.java" \
    "$SRC/ModelBundleVerifier.java" \
    "$SRC/ModelManager.java" \
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
    "$SRC/ObdBrakeClient.java"

echo "== 2. Prepare & compile isolated OTA model plugin ($ENTRY_CLASS, v$MVER, seq=$MSEQ) =="
if [ -d "$MOD/model/java" ] && [ -n "$(find "$MOD/model/java" -name '*.java' -print -quit)" ]; then
    PLUGIN_SRCS=$(find "$MOD/model/java" -name '*.java' | sort)
else
    python3 - "$SRC/BrakeEvaluator.java" "$META_JSON" "$W/gen" <<'PY'
import json, os, re, sys

src_path, meta_path, gen_root = sys.argv[1:4]
meta = json.load(open(meta_path, "r", encoding="utf-8"))
mver = meta["model_version"].strip()
mseq = int(meta["model_seq"])
entry_cls = meta["entry_class"].strip()
if not entry_cls.startswith("dev.lemoyne.brakeindicator.model."):
    raise SystemExit(f"entry_class must start with dev.lemoyne.brakeindicator.model.: {entry_cls}")
pkg_name, simple_cls = entry_cls.rsplit(".", 1)

raw = open(src_path, "r", encoding="utf-8").read()
# Strip host-owned DTO declarations so plugin references host parent ClassLoader DTOs cleanly
raw, n_dto = re.subn(
    r"\n\s*// === HOST_ABI_DTOS_BEGIN ===.*?// === HOST_ABI_DTOS_END ===\n",
    "\n",
    raw,
    count=1,
    flags=re.DOTALL,
)
if n_dto != 1:
    raise SystemExit("BrakeEvaluator.java missing HOST_ABI_DTOS_BEGIN/END markers")

header_imports = (
    f"package {pkg_name};\n\n"
    "import dev.lemoyne.brakeindicator.BrakeEvaluator.Input;\n"
    "import dev.lemoyne.brakeindicator.BrakeEvaluator.ObdSample;\n"
    "import dev.lemoyne.brakeindicator.BrakeEvaluator.Snapshot;\n"
    "import dev.lemoyne.brakeindicator.BrakeModel;\n"
    "import dev.lemoyne.brakeindicator.IndicatorConfig;\n"
)
raw, n_pkg = re.subn(r"^package\s+dev\.lemoyne\.brakeindicator\s*;", header_imports, raw, count=1, flags=re.MULTILINE)
if n_pkg != 1:
    raise SystemExit("Failed to rewrite package header in generated OTA model")

raw, n_cls = re.subn(
    r"public\s+final\s+class\s+BrakeEvaluator\s+implements\s+BrakeModel",
    f"public final class {simple_cls} implements BrakeModel",
    raw,
    count=1,
)
if n_cls != 1:
    raise SystemExit("Failed to rewrite class declaration in generated OTA model")

raw, n_ver = re.subn(
    r'BUILTIN_MODEL_VERSION\s*=\s*"[^"]+"',
    f'BUILTIN_MODEL_VERSION = "{mver}"',
    raw,
    count=1,
)
raw, n_seq = re.subn(
    r"BUILTIN_MODEL_SEQ\s*=\s*[0-9]+L",
    f"BUILTIN_MODEL_SEQ = {mseq}L",
    raw,
    count=1,
)
if n_ver != 1 or n_seq != 1:
    raise SystemExit("Failed to stamp model_version / model_seq into generated OTA model")

out_file = os.path.join(gen_root, *pkg_name.split("."), f"{simple_cls}.java")
os.makedirs(os.path.dirname(out_file), exist_ok=True)
with open(out_file, "w", encoding="utf-8") as fh:
    fh.write(raw)
PY
    PLUGIN_SRCS=$(find "$W/gen" -name '*.java' | sort)
fi

# Compile plugin against API-19 android.jar + host ABI classes
# shellcheck disable=SC2086
javac --release 8 -Xlint:-options -cp "$ANDROID_JAR:$W/host_classes" -d "$W/plugin_classes" $PLUGIN_SRCS

# Dex ONLY the plugin classes (min-api 19 for Gen5W Android 4.4.2 Dalvik)
PLUGIN_CLASS_FILES=$(find "$W/plugin_classes" -name '*.class' | sort)
# shellcheck disable=SC2086
"$D8" --release --min-api 19 \
    --lib "$ANDROID_JAR" \
    --classpath "$W/host_classes" \
    --output "$W/payload" \
    $PLUGIN_CLASS_FILES

# Include JVM .class files alongside classes.dex so both Dalvik DexClassLoader and host OpenJDK
# URLClassLoader can load and self-test the exact same signed .jar (~16 KB).
cp -r "$W/plugin_classes/dev" "$W/payload/dev"

echo "== 3. Sign & verify OTA model bundle =="
rm -rf dist-model
mkdir -p dist-model
OUT_JAR="dist-model/brake-model-${MVER}.jar"

JAR_SHA256=$(java -cp "$W/host_classes:$CLIENT_JAR" \
    dev.lemoyne.brakeindicator.ModelBundleVerifier \
    sign "$W/payload" "$OUT_JAR" "$META_JSON" "$ENTRY_CLASS" "$HMGAPI_KEY")

KEY_UUID=$(java -cp "$HMGAPI_SDK/lib/hmgapi-engine.jar" hmgapi.engine.Signing uuid "${HMGAPI_KEY}.pub")
PINNED_UUID="28ad71e0-07c0-8e61-8bf3-380cc71f9280"

if [ "$KEY_UUID" = "$PINNED_UUID" ] || [ "${REQUIRE_PINNED_MOD_KEY:-0}" = "1" ]; then
    java -cp "$W/host_classes:$CLIENT_JAR" \
        dev.lemoyne.brakeindicator.ModelBundleVerifier \
        verify "$OUT_JAR"
fi

# Load the signed .jar via URLClassLoader and run ModelManager's 6-tick Gate 1 self-test
cat > "$W/SelfTestRunner.java" <<'JAVA'
package dev.lemoyne.brakeindicator;
import java.io.File;
import java.net.URL;
import java.net.URLClassLoader;
public final class SelfTestRunner {
    public static void main(String[] args) throws Exception {
        File jar = new File(args[0]);
        String entry = args[1];
        URLClassLoader cl = new URLClassLoader(
                new URL[]{jar.toURI().toURL()}, BrakeModel.class.getClassLoader());
        Class<?> c = Class.forName(entry, true, cl);
        BrakeModel m = (BrakeModel) c.getDeclaredConstructor().newInstance();
        ModelManager.runSelfTest(m);
        System.out.println("self-test ok: " + m.modelVersion() + " (seq=" + m.modelSeq()
                + ", api=" + m.modelApiVersion() + ", size=" + jar.length() + " bytes)");
    }
}
JAVA
javac --release 8 -Xlint:-options -cp "$W/host_classes:$CLIENT_JAR" -d "$W/host_classes" "$W/SelfTestRunner.java"
java -cp "$W/host_classes:$CLIENT_JAR" dev.lemoyne.brakeindicator.SelfTestRunner "$OUT_JAR" "$ENTRY_CLASS"

python3 - "$OUT_JAR" "$MVER" "$MSEQ" "$MIN_API" "$MAX_API" "$JAR_SHA256" "dist-model/latest.json" <<'PY'
import json, os, sys

jar_file, mver, mseq, min_api, max_api, sha256_hex, out_json = sys.argv[1:8]
doc = {
    "format": 1,
    "model_version": mver,
    "model_seq": int(mseq),
    "min_host_api": int(min_api),
    "max_host_api": int(max_api),
    "jar_path": f"models/brake-model-{mver}.jar",
    "jar_sha256": sha256_hex,
    "jar_size": os.path.getsize(jar_file),
}
with open(out_json, "w", encoding="utf-8") as fh:
    json.dump(doc, fh, indent=2)
    fh.write("\n")
PY

(cd dist-model && sha256sum "brake-model-${MVER}.jar" latest.json > SHA256SUMS)
echo "== Built signed OTA brake model (author key UUID: $KEY_UUID) =="
cat dist-model/SHA256SUMS
