#!/bin/sh
# Build, test, and sign installable HMG-API and NEMESIS packages into dist/:
#   dist/brake-indicator-gen5w-v<ver>.hmgapi.zip
#   dist/brake-indicator-gen5w-v<ver>-hmgapi-usb.zip
#   dist/brake-indicator-gen5w-v<ver>-nemesis-package.zip
#   dist/brake-indicator-gen5w-v<ver>-nemesis-usb.zip
#   dist/SHA256SUMS
#
# Signing keys (kept private outside the repo; reuse the same keys for updates):
#   HMGAPI_KEY      mod author ECDSA key (default: ~/.hmgapi/author.key, or ~/.gemini/hmgapi/author.key)
#   HMGAPI_APP_KEY  APK PKCS#12 keystore, password "example" (default alongside author.key)
#   HMGAPI_SDK      unpacked HMG-API Mod SDK (auto-detected from ../HMG-API/out/sdk or built from ../HMG-API)
#   ANDROID_HOME    Android SDK with platforms/android-19 and build-tools
set -e
cd "$(dirname "$0")"
MOD=brake-indicator-gen5w
VER=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["version"])' "$MOD/mod.json")

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
        echo "build.sh: set HMGAPI_SDK to an unpacked HMG-API mod SDK (or place ../HMG-API beside this repo)" >&2
        exit 1
    fi
fi
export HMGAPI_SDK

# Locate or create persistent private signing keys
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
export HMGAPI_APP_KEY=${HMGAPI_APP_KEY:-$KEY_DIR/brake-indicator-app.p12}
if [ ! -f "$HMGAPI_APP_KEY" ]; then
    keytool -genkeypair -keystore "$HMGAPI_APP_KEY" -storetype pkcs12 \
        -storepass example -keypass example -alias app \
        -keyalg RSA -keysize 2048 -validity 15000 \
        -dname "CN=Brake Light Indicator Gen5W" >/dev/null 2>&1
fi
umask 022

echo "== 1. Build APK & stage mod =="
rm -rf out/build out/work out/packages out/dist dist
sh "$HMGAPI_SDK/bin/build-mod.sh" "$MOD" "out/build/$MOD" "out/work/$MOD"

echo "== 2. Run tests & head-unit script check =="
HMGAPI_MOD_BUILD="$PWD/out/build/$MOD" sh "$MOD/test.sh"
HMGAPI_MOD_BUILD="$PWD/out/build/$MOD" sh "$MOD/tools/unit/test.sh"
sh "$HMGAPI_SDK/bin/check-scripts.sh" "$MOD"

echo "== 3. Sign HMG-API & NEMESIS installable packages =="
PKG_OUT=$(HMGAPI_MOD_BUILD="$PWD/out/build/$MOD" sh "$HMGAPI_SDK/bin/mkmodpkg.sh" "$MOD" "$HMGAPI_KEY")
echo "$PKG_OUT"
SIGNED_ZIP=$(echo "$PKG_OUT" | tail -1)
sh "$HMGAPI_SDK/bin/mknemesismod.sh" "$SIGNED_ZIP"

mkdir -p dist
HMGAPI_ZIP="dist/${MOD}-v${VER}.hmgapi.zip"
HMGAPI_USB="dist/${MOD}-v${VER}-hmgapi-usb.zip"
NEM_PKG="dist/${MOD}-v${VER}-nemesis-package.zip"
NEM_USB="dist/${MOD}-v${VER}-nemesis-usb.zip"

cp "$SIGNED_ZIP" "$HMGAPI_ZIP"
cp "out/dist/${MOD}-${VER}-nemesis-package.zip" "$NEM_PKG"
cp "out/dist/${MOD}-${VER}-nemesis.zip" "$NEM_USB"

# Replace bin/open.sh in NEMESIS package so NEMESIS Manager -> Launch opens Brake Indicator directly,
# and wrap the HMG-API USB stick zip (HMGAPI/updates/<id>-v<ver>.zip).
python3 - "$MOD" "$VER" "$HMGAPI_ZIP" "$HMGAPI_USB" "$NEM_PKG" "$NEM_USB" <<'PY'
import os, sys, zipfile

mod, ver, hmg_zip, hmg_usb, nem_pkg, nem_usb = sys.argv[1:7]

def swap_in_zip(path, member, new_bytes):
    with zipfile.ZipFile(path, "r") as z:
        items = [(i, z.read(i)) for i in z.infolist()]
    with zipfile.ZipFile(path, "w") as z:
        for info, data in items:
            z.writestr(info, new_bytes if info.filename == member else data)

open_sh = os.path.join(mod, "open.sh")
if os.path.isfile(open_sh):
    swap_in_zip(nem_pkg, "bin/open.sh", open(open_sh, "rb").read())
    swap_in_zip(nem_usb, f"packages/hmgapi.{mod}.zip", open(nem_pkg, "rb").read())

with zipfile.ZipFile(hmg_usb, "w", zipfile.ZIP_STORED) as z:
    zi = zipfile.ZipInfo(f"HMGAPI/updates/{mod}-v{ver}.zip", (2026, 1, 1, 0, 0, 0))
    zi.compress_type = zipfile.ZIP_STORED
    zi.external_attr = 0o644 << 16
    z.writestr(zi, open(hmg_zip, "rb").read())
PY

(cd dist && sha256sum *.zip > SHA256SUMS)
UUID=$(java -cp "$HMGAPI_SDK/lib/hmgapi-engine.jar" hmgapi.engine.Signing uuid "${HMGAPI_KEY}.pub")
echo "== Built installable packages (author key UUID: $UUID) =="
cat dist/SHA256SUMS
