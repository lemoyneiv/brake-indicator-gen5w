# Brake Light Indicator for Gen5W (`brake-indicator-gen5w`)

An [HMG-API](https://github.com/Open-HMG/HMG-API) mod package for Hyundai / Kia / Genesis **Gen5W** head units (EV6, Ioniq 5, Ioniq 6, etc.) that displays a non-intrusive **brake light indicator icon** (`(( ● ))`) on the top status bar row of the screen.

On E-GMP and other Hyundai/Kia EVs/HEVs, regenerative braking profiles (Level 1–3, Auto Regen, i-Pedal, and Smart Cruise Control) frequently illuminate the rear brake lamps without the driver touching the brake pedal. Because Gen5W displays no brake-lamp indicator on the cluster or infotainment screen, this mod provides clear in-car visual confirmation whenever the brake lights come on.

---

## Architecture (Option 1 — Native OEM Signal Path)

`brake-indicator-gen5w` is completely standalone (`"live": true`, no `/system` file patches, no root shell scripts) and consists of two cooperating components bundled in `app/BrakeIndicator.apk`:

1. **`MonitorMain` (`system`-UID HMG-API service)**:
   - Polls HMG-API's shared vehicle service (`brake.pressure`, `speed`, `pedal.accel`, `gear.park`, `parking_brake`, `moving`) at 10 Hz.
   - Loads `/system/framework/com.hkmc.ext.hcan.jar` directly into Dalvik via `PathClassLoader` and binds `tm_hscan_service` (`ITM_HSCANService$Stub.asInterface`) as `uid 1000` (`system`).
   - Runs a strict read-only safety classifier (`CandidateFilter`) across `ITM_HSCANService` to discover any direct OEM stop-lamp / brake-switch getters (`DIRECT_STOP_LAMP`), master/IEB cylinder pressure getters (`getCylPrsrVal`), and diagnostic chassis/regen getters (`DIAGNOSTIC_CANDIDATE`) — while blocking all mutating methods, side-effecting calls (`getDriveMode`), and per-car identity/location fields (VIN, GPS, IMEI, MSISDN, USIM, MAC, etc.).
   - Evaluates illumination priority via `BrakeEvaluator`:
     1. **Direct OEM Stop-Lamp Getter** (`ITM_HSCANService` binary stop-lamp / brake-switch getter, if present on the car's firmware)
     2. **Brake Cylinder Pressure** (`brake.pressure` / `getCylPrsrVal` with hysteresis and `409.5` CAN-invalid filtering — covers foot-pedal braking and IEB blended hydraulic pressure)
     3. **Optional Regen Deceleration Fallback** (`signal_mode = native_and_decel`: smoothed CAN speed `-dv/dt` with hysteresis at the UNECE Reg. 13-H `1.3 m/s²` threshold when off-throttle and out of Park, for firmwares whose HSCAN interface does not forward pure-regen stop-lamp state to the head unit).

2. **`OverlayService` & `MainActivity` (`dev.lemoyne.brakeindicator`)**:
   - Renders an anti-aliased brake-lamp icon in a non-touchable `TYPE_SYSTEM_OVERLAY` window (`FLAG_NOT_FOCUSABLE | FLAG_NOT_TOUCHABLE | FLAG_LAYOUT_IN_SCREEN`) aligned with the top status bar row (`51×36` px default), visible across the Gen5W launcher, built-in navigation, Apple CarPlay, and Android Auto without intercepting touches or conflicting with status-bar slot mods.
   - Provides a **Brake Indicator** tile in the Gen5W launcher (and an optional ★ custom button entry) opening a dark diagnostic dashboard with:
     - Live indicator preview & telemetry (`brake.pressure`, `speed`, `-dv/dt`, `pedal.accel`, `gear.park`)
     - **Test Icon (3s)** button to verify top-bar placement while parked
     - **Option 1 `ITM_HSCANService` Signal Probe** listing every discovered brake/regen getter in `com.hkmc.ext.hcan.jar` and its live transitions
     - **Export USB** button saving `BrakeIndicator/hscan-probe-<timestamp>.txt` to an attached USB drive.

---

## Settings (`settings.json`)

Configurable directly inside **HMG-API → Mods → Brake Light Indicator**:

| Setting | Default | Description |
|---|---|---|
| `signal_mode` | `native` | `native` (Option 1 OEM HSCAN getters + `brake.pressure` only) or `native_and_decel` (adds CAN speed deceleration fallback for pure regen) |
| `off_state` | `hidden` | `hidden` (shown only while brake lights are on) or `dim` (subtle grey outline when off) |
| `anchor` | `top_right` | `top_right`, `top_center`, or `top_left` |
| `x_offset` | `260` | Horizontal offset in px from `anchor` (`260` sits just left of the status-bar icons/clock on 1920×720) |
| `y_offset` | `6` | Vertical offset in px from the top edge |
| `icon_size` | `36` | Indicator height in px (`20`–`96`; stock status bar icons are `36` px) |
| `pressure_threshold` | `15` | `brake.pressure` threshold in tenths (`15` = `1.5`) |
| `decel_threshold` | `13` | Regen deceleration threshold in tenths of `m/s²` (`13` = `1.3 m/s²`) when `native_and_decel` is selected |
| `probe_logging` | `true` | Record read-only `ITM_HSCANService` candidate transitions for Option 1 verification |

---

## On-Car Option 1 Verification Workflow

Because public Gen5W Binder dumps omitted `/system/framework/com.hkmc.ext.hcan.jar`, the built-in probe lets you verify on your car in two minutes:

1. Install and enable **Brake Light Indicator** in HMG-API.
2. Open **Brake Indicator** from the launcher and tap **Test Icon (3s)** to confirm top-bar placement.
3. Check the right-hand **Option 1 OEM Signal Probe** panel while pressing the brake pedal in Park/Drive, then perform a brief regenerative deceleration lift-off in Level 2/3 or i-Pedal.
4. Insert a USB stick and tap **Export USB** (or run `adb shell hmgapid service-log monitor`) to inspect `BrakeIndicator/hscan-probe-<timestamp>.txt` and see:
   - Whether `ITM_HSCANService` exposes a dedicated stop-lamp / regen-brake getter on your firmware.
   - Whether `getCylPrsrVal()` (`brake.pressure`) rises during pure E-GMP IEB regen or only friction/blended braking.

---

## Building & Testing Locally

Requires Java 17+, Python 3, the Android SDK (`ANDROID_HOME` with `platforms/android-19` and `build-tools`), and an unpacked HMG-API Mod SDK (`HMGAPI_SDK`):

```sh
# 1. Verify HMG-API floor & hook points
python3 "$HMGAPI_SDK/bin/check-points.py" brake-indicator-gen5w --repo Open-HMG/HMG-API

# 2. Build APK & mod staging directory
sh "$HMGAPI_SDK/bin/build-mod.sh" brake-indicator-gen5w out/build/brake-indicator-gen5w out/work/brake-indicator-gen5w

# 3. Run HMG-API engine tests + Java unit tests
HMGAPI_SDK="$HMGAPI_SDK" HMGAPI_MOD_BUILD="$PWD/out/build/brake-indicator-gen5w" sh brake-indicator-gen5w/test.sh
sh brake-indicator-gen5w/tools/unit/test.sh

# 4. Verify head-unit script compatibility
sh "$HMGAPI_SDK/bin/check-scripts.sh" brake-indicator-gen5w

# 5. Package signed HMG-API mod zip (and optional NEMESIS zip)
sh "$HMGAPI_SDK/bin/mkmodpkg.sh" brake-indicator-gen5w /path/to/author.key
sh "$HMGAPI_SDK/bin/mknemesismod.sh" out/packages/brake-indicator-gen5w-0.1.0.zip
```

## License

GNU Affero General Public License v3.0 ([LICENSE](LICENSE)), matching [Open-HMG/HMG-API](https://github.com/Open-HMG/HMG-API).
