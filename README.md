# Brake Light Indicator for Gen5W (`brake-indicator-gen5w`)

An [HMG-API](https://github.com/Open-HMG/HMG-API) mod package for Hyundai / Kia / Genesis **Gen5W** head units (EV6, Ioniq 5, Ioniq 6, etc.) that displays a non-intrusive **brake light indicator icon** (`(( ● ))`) on the top status bar row of the screen.

On E-GMP and other Hyundai/Kia EVs/HEVs, regenerative braking profiles (Level 1–3, Auto Regen, i-Pedal, and Smart Cruise Control) frequently illuminate the rear brake lamps without the driver touching the brake pedal. Because Gen5W displays no brake-lamp indicator on the cluster or infotainment screen, this mod provides clear in-car visual confirmation whenever the brake lights come on.

---

## Architecture (`v0.5.0`)

`brake-indicator-gen5w` is completely standalone (`"live": true`, zero `/system` file patches, zero `root` mod daemons) and runs inside a single lightweight Android app (`app/BrakeIndicator.apk`, `OverlayService` + `BrakeEngine`):

1. **Poll-Aligned Evaluation & E-GMP UN ECE R13-H Brake-Lamp Model (`BrakeEngine` + `BrakeEvaluator`)**:
   - Runs a dedicated `50 ms` (`20 Hz`) evaluation and RAM telemetry recorder thread while polling HMG-API's built-in readings (`brake.pressure`, `speed`, `pedal.accel`, `gear.park`, `parking_brake`, `moving`, `steering.angle`, `hv.soc`) over `@hmgapid` (`LFD1`) at `20 Hz` (`< 2 ms` on updated HMG-API, or `650 ms` with automatic `90 ms` `'O'`-frame recovery on legacy KitKat `Hub`s).
   - Aligns longitudinal `-dv/dt` updates to exact HMG-API poll timestamps (`hmgSampleElapsedMs`) and suppresses `< 380 ms` `0.8 km/h` (`0.5 mph`) cluster speed-quantization repeats so intermediate `50 ms` ticks never inject artificial `0.0 m/s²` sawtooth dropouts on either `20 Hz` or `650 ms` `Hub`s:
     1. **Hydraulic / IEB Cylinder Pressure** (`brake.pressure` with a responsive `0.4` onset threshold, `0.70×` release hysteresis, and `409.5` CAN-invalid filtering for instant foot-pedal & IEB hydraulic braking).
     2. **Calibrated Regenerative & Smart Cruise Deceleration** (`signal_mode = pressure_and_decel`, enabled by default: models E-GMP's UN ECE R13-H regen brake-lamp law with immediate lift-off onset in `L2 / L3 / i-Pedal`, feathered one-pedal regen, latch hysteresis down to `0.55 m/s²` while `pedal.accel < 11.5%`, and `450 ms` transition smoothing).
     3. **Standstill Hold** (`standstill_hold = true`: keeps the indicator illuminated at `0 km/h` after decelerating to a stop in `i-Pedal` or Smart Cruise Control until throttle `>= 2%`, vehicle movement, or Park).

2. **Automatic `L0 / L1 / L2 / L3 / i-Pedal` Regen-Mode Detection**:
   - **Settled Rolling Coasting Chord (`L0`–`L3+`)**: Automatically classifies `L0`, `L1`, `L2`, and `L3+` whenever the driver coasts off both pedals (`pedal.accel < 1%`, `brake.pressure < 0.25`) above `21 km/h` (above E-GMP's `< 20 km/h` creep taper), using a `520 ms` settle + `520–1400 ms` sliding chord (`0.44 / 0.78 / 1.22 m/s²` boundaries with `±0.06 m/s²` hysteresis).
   - **`i-Pedal` (`4`) vs `L3` (`3`) Discrimination**: Distinguishes `i-Pedal` from `L3` via low-speed zero-pedal full-stop regen (`2–14 km/h` at `>= 1.35 m/s²` where `L0–L3` taper to creep) and steady-speed neutral cruise pedal (`23–30%` for `>= 1.1 s`), stepping back down to `L3` if low-speed zero-pedal creep taper (`0.15–0.80 m/s²` across `9–17 km/h`) is observed.

3. **Optional OBD-II Ground-Truth Brake Lamp Reader (`IGMP 770 → 778`, UDS `22 BC06`)**:
   - Reads the physical E-GMP Integrated Gateway (`IGMP`) brake-lamp output byte (`22 BC06`: `0` = OFF; bit `0` = `0` [`4, 36, 40, 44`] = pure Regen / i-Pedal / Smart Cruise / Auto-Hold brake lamps ON; bit `0` = `1` [`37, 41, 45`] = physical foot-brake pedal switch active) to display exact hardware brake-lamp state and score the HMG-API model.
   - **Connection Priority (`ObdBrakeClient`)**:
     1. **Primary — Direct BLE via `@gen5w_ble` (`Gen5wBleClient`)**: Connects directly over BLE GATT when paired with a BLE ELM327/STN scanner (Veepeak OBDCheck BLE, Vgate, OBDLink, Nordic NUS, ISSC, etc.), with strict LNEX BVM02 12V battery-sensor hardware exclusion (`0xFFF4` / `0xFFF1` guard), read-only command filtering (blocks VIN `0902`/`22F190` and all UDS write services), and `IG OFF` CAN sleep protection.
     2. **Fallback — Localhost ELM327 Bridge (`127.0.0.1:35000`)**: Automatically falls back to [`obd-gen5w`](https://github.com/lemoyneiv/obd-gen5w)'s local TCP ELM327 bridge whenever `obd-gen5w` already owns the BLE adapter (`ALREADY_CONNECTED`) or no BLE adapter is paired directly. Traffic stays strictly on the Linux kernel loopback interface (`lo`) and never touches `wlan0` or wireless projection.

4. **Live Accuracy Scorer & Zero-Flash-Write Calibration Dataset (`TelemetryRecorder` + `UsbExporter`)**:
   - Continuously scores the HMG-API model against live OBD `22 BC06` ground truth (`TP`, `FP`, `TN`, `FN`, overall agreement %, precision %, recall %, separate Pedal Switch vs Pure Regen/SCC recall %, and signed onset/release lag in `ms`).
   - Buffers up to `36,000` synchronized HMG-API + OBD + Regen Mode rows (~30 minutes at `20 Hz`, automatically downsampled to `1 Hz` while quietly parked) in compact primitive arrays in RAM (zero internal eMMC flash writes).
   - **Export USB** atomically writes `BrakeIndicator/brake-dataset-<timestamp>.csv` and `BrakeIndicator/brake-summary-<timestamp>.txt` (`*.part` + `fsync` + `rename`) to any mounted FAT32/exFAT USB drive.

5. **Top-Bar Overlay & Diagnostic Dashboard (`OverlayService` + `MainActivity`)**:
   - Renders an anti-aliased brake-lamp icon in a non-touchable `TYPE_SYSTEM_OVERLAY` window (`FLAG_NOT_FOCUSABLE | FLAG_NOT_TOUCHABLE | FLAG_LAYOUT_IN_SCREEN`) aligned with the top status bar row (`51×36` px default), visible across the Gen5W launcher, built-in navigation, and wireless projection without intercepting touches.
   - Provides a **Brake Indicator** tile in the Gen5W launcher (and an optional ★ custom button entry) with live HMG-API vs OBD telemetry, detected `L0..i-Pedal` mode readout, quick **OBD Mode** toggle, instant **Mark Log** bookmark button, **Scan BLE** / **Pair** controls, live transition log, and **Export USB**.

---

## Settings (`settings.json`)

Configurable inside **HMG-API → Mods → Brake Light Indicator** (and `obd_mode` can also be cycled directly from the Brake Indicator app):

| Setting | Default | Description |
|---|---|---|
| `obd_mode` | `obd_when_connected` | `obd_when_connected` (real OBD `22BC06` drives top-bar icon when connected, HMG-API model fallback + parallel scoring), `model_only_icon` (HMG-API model drives icon while OBD scores live accuracy in background), or `disabled` (HMG-API readings only) |
| `signal_mode` | `pressure_and_decel` | `pressure_and_decel` (`brake.pressure` + Regen / Smart Cruise deceleration & standstill hold) or `pressure_only` (`brake.pressure` only) |
| `off_state` | `hidden` | `hidden` (shown only while brake lights are on) or `dim` (subtle grey outline when off) |
| `anchor` | `top_right` | `top_right`, `top_center`, or `top_left` |
| `x_offset` | `260` | Horizontal offset in px from `anchor` (`260` sits just left of the status-bar icons/clock on 1920×720) |
| `y_offset` | `6` | Vertical offset in px from the top edge |
| `icon_size` | `36` | Indicator height in px (`20`–`96`; stock status bar icons are `36` px) |
| `pressure_threshold` | `4` | `brake.pressure` threshold in tenths (`4` = `0.4`) for immediate foot-pedal response |
| `decel_threshold` | `12` | Regen / Smart Cruise deceleration threshold scale in tenths of `m/s²` (`12` = default E-GMP calibration) |
| `standstill_hold` | `true` | Keep illuminated at `0 km/h` after a regen or Smart Cruise stop until throttle, movement, or Park |
| `bridge_port` | `35000` | Localhost TCP ELM327 fallback port (`127.0.0.1:<port>`, matching `obd-gen5w`) |

---

## Installation & Updating

Download the latest release assets from [Releases](https://github.com/lemoyneiv/brake-indicator-gen5w/releases):

- **With HMG-API (USB Updater):** Unzip `brake-indicator-gen5w-v0.5.0-hmgapi-usb.zip` to the root of a FAT32/exFAT USB drive (creates `HMGAPI/updates/brake-indicator-gen5w-v0.5.0.zip`), plug into the head unit's data USB port, and install/update from **HMG-API → Updates**.
- **With NEMESIS (USB Auto-Prompt):** Unzip `brake-indicator-gen5w-v0.5.0-nemesis-usb.zip` to the root of a USB drive (`NEMESIS.data` and `packages/hmgapi.brake-indicator-gen5w.zip`) and plug into the head unit once Android has booted. Tapping **Launch** on **Brake Light Indicator** in NEMESIS Manager opens the **Brake Indicator** app directly.
- **Direct Packages:** `brake-indicator-gen5w-v0.5.0.hmgapi.zip` (raw signed HMG-API mod package) and `brake-indicator-gen5w-v0.5.0-nemesis-package.zip` (NEMESIS Package Store / `nemesis install` archive).

> **Note on USB Export & USB Host Mode:** When exporting calibration CSV datasets via **Export USB**, make sure **USB ADB** is turned off so the head unit's front data USB port operates in USB host mode (able to mount a FAT32/exFAT flash drive).

---

## Building & Signing Installable Packages

Requires Java 17+, Python 3, the Android SDK (`ANDROID_HOME` with `platforms/android-19` and `build-tools`), and an unpacked HMG-API Mod SDK (`HMGAPI_SDK` or `../HMG-API`):

```sh
./build.sh
```

`./build.sh` compiles the APK, runs both test suites and `check-scripts.sh`, signs with `${HMGAPI_KEY:-~/.hmgapi/author.key}`, and writes all four installable `.zip` archives plus `SHA256SUMS` into `dist/`.

## License

GNU Affero General Public License v3.0 ([LICENSE](LICENSE)), matching [Open-HMG/HMG-API](https://github.com/Open-HMG/HMG-API).
