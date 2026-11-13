# Brake Light Indicator for Gen5W (`brake-indicator-gen5w`)

An [HMG-API](https://github.com/Open-HMG/HMG-API) mod package for Hyundai / Kia / Genesis **Gen5W** head units (EV6, Ioniq 5, Ioniq 6, etc.) that displays a non-intrusive **brake light indicator icon** (`(( ● ))`) on the top status bar row of the screen.

On E-GMP and other Hyundai/Kia EVs/HEVs, regenerative braking profiles (Level 1–3, Auto Regen, i-Pedal, and Smart Cruise Control) frequently illuminate the rear brake lamps without the driver touching the brake pedal. Because Gen5W displays no brake-lamp indicator on the cluster or infotainment screen, this mod provides clear in-car visual confirmation whenever the brake lights come on.

---

## Architecture

`brake-indicator-gen5w` is completely standalone (`"live": true`, no `/system` file patches) and consists of two cooperating components bundled in `app/BrakeIndicator.apk`:

1. **`MonitorMain` (`root` HMG-API background service, 25 Hz zero-hop Binder loop)**:
   - Binds directly to Gen5W's OEM Binder interfaces (`tm_hscan_service` in `/system/framework/com.hkmc.ext.hcan.jar`, `com.hkmc.micom.ISystemService`, and `com.hkmc.bcm.IBCMService`) for `< 1 ms` in-process 25 Hz (`40 ms`) polling of `getCylPrsrVal()`, `getCarSpeed((byte) 0)`, `getAccelPdlVal()`, `IsCarMoving()`, `getGearPStatus()`, and `getEPBStatus()` — working even on firmwares without a pre-built HMG-API hook map and eliminating socket/cache lag.
   - Evaluates illumination priority via `BrakeEvaluator`:
     1. **Direct OEM Stop-Lamp Getter** (`ITM_HSCANService` / `ISystemService` / `IBCMService` binary stop-lamp or brake-switch getter, if exposed on the firmware)
     2. **Brake Cylinder Pressure** (`getCylPrsrVal` / `brake.pressure` with a responsive `0.4` onset threshold, hysteresis, and `409.5` CAN-invalid filtering for instant foot-pedal & IEB hydraulic response)
     3. **Regenerative & Smart Cruise Control Deceleration** (`signal_mode = native_and_decel`, enabled by default: 240 ms sliding-window CAN speed `-dv/dt` with EMA + hysteresis at `1.2 m/s²` onset / `0.72 m/s²` release, supporting modulated i-Pedal & Level 3 regen as well as Smart Cruise Control braking)
     4. **Standstill Hold** (`standstill_hold = true`: keeps the brake indicator lit at `0 km/h` after decelerating to a stop in i-Pedal or Smart Cruise Control until throttle is pressed, the car moves, or Park is selected).
   - Runs a strict read-only safety classifier (`CandidateFilter`) across `ITM_HSCANService`, `ISystemService`, and `IBCMService` to inventory all safe zero-argument primitive getters and log live value transitions during driving — while blocking all mutating methods, side-effecting calls (`getDriveMode`), and per-car identity/location fields (VIN, GPS, IMEI, MSISDN, USIM, MAC, etc.).

2. **`OverlayService` & `MainActivity` (`dev.lemoyne.brakeindicator`)**:
   - Renders an anti-aliased brake-lamp icon in a non-touchable `TYPE_SYSTEM_OVERLAY` window (`FLAG_NOT_FOCUSABLE | FLAG_NOT_TOUCHABLE | FLAG_LAYOUT_IN_SCREEN`) aligned with the top status bar row (`51×36` px default), visible across the Gen5W launcher, built-in navigation, Apple CarPlay, and Android Auto without intercepting touches or conflicting with status-bar slot mods.
   - Provides a **Brake Indicator** tile in the Gen5W launcher (and an optional ★ custom button entry) opening a dark diagnostic dashboard with:
     - Live indicator preview & telemetry (`brake.pressure`, `speed`, `-dv/dt`, `pedal.accel`, `gear.park`)
     - **Test Icon (3s)** button to verify top-bar placement while parked
     - **OEM CAN Signal Probe** listing all safe read-only getters in `com.hkmc.ext.hcan.jar` and live transitions recorded during braking/regen/cruise
     - **Export USB** button (handled as `root` via `/proc/mounts` detection + `fsync`) creating `BrakeIndicator/hscan-probe-<timestamp>.txt` directly on any plugged-in FAT32/exFAT USB stick without requiring pre-existing folders.

---

## Settings (`settings.json`)

Configurable directly inside **HMG-API → Mods → Brake Light Indicator**:

| Setting | Default | Description |
|---|---|---|
| `signal_mode` | `native_and_decel` | `native_and_decel` (OEM signals + Regen / Smart Cruise deceleration & standstill hold) or `native` (OEM HSCAN getters + `brake.pressure` only) |
| `off_state` | `hidden` | `hidden` (shown only while brake lights are on) or `dim` (subtle grey outline when off) |
| `anchor` | `top_right` | `top_right`, `top_center`, or `top_left` |
| `x_offset` | `260` | Horizontal offset in px from `anchor` (`260` sits just left of the status-bar icons/clock on 1920×720) |
| `y_offset` | `6` | Vertical offset in px from the top edge |
| `icon_size` | `36` | Indicator height in px (`20`–`96`; stock status bar icons are `36` px) |
| `pressure_threshold` | `4` | `brake.pressure` threshold in tenths (`4` = `0.4`) for immediate foot-pedal response |
| `decel_threshold` | `12` | Regen / Smart Cruise deceleration threshold in tenths of `m/s²` (`12` = `1.2 m/s²`) |
| `standstill_hold` | `true` | Keep illuminated at `0 km/h` after a regen or Smart Cruise stop until throttle, movement, or Park |
| `probe_logging` | `true` | Record read-only OEM CAN getter transitions for on-car signal verification and USB export |

---

## Installation & Updating

Download the latest release assets from [Releases](https://github.com/lemoyneiv/brake-indicator-gen5w/releases):

- **With HMG-API (USB Updater):** Unzip `brake-indicator-gen5w-v0.2.0-hmgapi-usb.zip` to the root of a FAT32/exFAT USB drive (creates `HMGAPI/updates/brake-indicator-gen5w-v0.2.0.zip`), plug into the head unit's data USB port, and install/update from **HMG-API → Updates**.
- **With NEMESIS (USB Auto-Prompt):** Unzip `brake-indicator-gen5w-v0.2.0-nemesis-usb.zip` to the root of a USB drive (`NEMESIS.data` and `packages/hmgapi.brake-indicator-gen5w.zip`) and plug into the head unit once Android has booted. Tapping **Launch** on **Brake Light Indicator** in NEMESIS Manager opens the **Brake Indicator** app directly.
- **Direct Packages:** `brake-indicator-gen5w-v0.2.0.hmgapi.zip` (raw signed HMG-API mod package) and `brake-indicator-gen5w-v0.2.0-nemesis-package.zip` (NEMESIS Package Store / `nemesis install` archive).

> **Note on USB Export & USB ADB:** When exporting probe logs via **Export USB**, make sure **USB ADB** is turned off so the head unit's data USB port is in USB host mode (able to mount a flash drive). A fallback copy is also written to `/data/local/tmp/brake-indicator-probe.txt`.

---

## Building & Signing Installable Packages

Requires Java 17+, Python 3, the Android SDK (`ANDROID_HOME` with `platforms/android-19` and `build-tools`), and an unpacked HMG-API Mod SDK (`HMGAPI_SDK` or `../HMG-API`):

```sh
./build.sh
```

`./build.sh` compiles the APK, runs both test suites and `check-scripts.sh`, signs with `${HMGAPI_KEY:-~/.hmgapi/author.key}`, and writes all four installable `.zip` archives plus `SHA256SUMS` into `dist/`.

## License

GNU Affero General Public License v3.0 ([LICENSE](LICENSE)), matching [Open-HMG/HMG-API](https://github.com/Open-HMG/HMG-API).
