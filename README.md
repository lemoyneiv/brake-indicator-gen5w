# Brake Light Indicator for Gen5W (`brake-indicator-gen5w`)

An [HMG-API](https://github.com/Open-HMG/HMG-API) mod package for Hyundai / Kia / Genesis **Gen5W** head units (EV6, Ioniq 5, Ioniq 6, etc.) that displays a non-intrusive **brake light indicator icon** (`(( ● ))`) on the top status bar row of the screen.

On E-GMP and other Hyundai/Kia EVs/HEVs, regenerative braking profiles (Level 1–3, Auto Regen, i-Pedal, and Smart Cruise Control) frequently illuminate the rear brake lamps without the driver touching the brake pedal. Because Gen5W displays no brake-lamp indicator on the cluster or infotainment screen, this mod provides clear in-car visual confirmation whenever the brake lights come on.

---

## Architecture (`v0.7.9`)

`brake-indicator-gen5w` is completely standalone (`"live": true`, `"ble": {"central": true}`, zero `/system` file patches, zero `root` mod daemons) and runs inside a single lightweight Android app (`app/BrakeIndicator.apk`, `OverlayService` + `BrakeEngine`):

1. **Poll-Aligned Evaluation & E-GMP UN ECE R13-H Brake-Lamp Model (`BrakeEngine` + `BrakeEvaluator`)**:
   - Runs a dedicated `50 ms` (`20 Hz`) evaluation and RAM telemetry recorder thread while polling HMG-API's built-in readings (`brake.pressure`, `speed`, `pedal.accel`, `gear.park`, `parking_brake`, `moving`, `steering.angle`, `hv.soc`) over `@hmgapid` (`LFD1`) at `20 Hz` (`< 2 ms` with `100 ms` `speed`/`moving` freshness on updated HMG-API, or `650 ms` with automatic `90 ms` `'O'`-frame recovery on legacy KitKat `Hub`s).
   - Aligns longitudinal `-dv/dt` updates to exact HMG-API poll timestamps (`hmgSampleElapsedMs`), suppresses `< 560 ms` `0.8 km/h` (`0.5 mph`) cluster speed-quantization repeats while bounding stale high deceleration across flat-speed intervals on `20 Hz` Hubs, gates `lastFootBrakeMs` on latched pedal pressure (`pressureLatched`) so sub-latch pedal brushes never cut off active regen, isolates mild post-foot-brake hydraulic & intersection trail-braking turn-scrub (`>= 80°` steering above `18 km/h`) `-dv/dt` tails, and subtracts both passive highway aerodynamic drag excess above `36 km/h` (`10 m/s`) and low-speed full-lock Ackermann tire-scrub drag (`|steering.angle| > 220°` below `18.5 km/h` in `L0 / L1 / Auto`) so wind drag and sharp turns do not masquerade as motor regen torque:
     1. **Hydraulic / IEB Cylinder Pressure** (`brake.pressure` with a responsive `0.4` onset threshold, poll-rate-aware rolling brush sensitivity [`0.26 bar` on `20 Hz` Hubs], gentle low-speed feather hold [`0.18 bar` release], and `409.5` CAN-invalid filtering for instant foot-pedal & IEB hydraulic braking).
     2. **Calibrated Regenerative, Auto Regen (`Smart Recuperation`), One-Pedal `i-Pedal`, & Smart Cruise Deceleration** (`signal_mode = pressure_and_decel`, enabled by default: models E-GMP's UN ECE R13-H regen brake-lamp law with first-class Auto-Regen detection [suppressing open-road lift-off anticipation and light-throttle cold-onset false positives, treating `< 1.7%` highway pedal deadzone as off-pedal coast, preserving light `1–12.8%` throttle feathering while active radar regen stays above R13-H hold thresholds, and applying speed-banded `< 14.2 / 14.2–39.5 / >= 39.5 km/h` onset & settled-coast release hysteresis], cruise vs post-acceleration VCU torque-ramp lift-off timing in manual `L2 / L3 / i-Pedal`, piecewise speed-dependent `i-Pedal` feathered one-pedal braking onset [`5.5%–16.0%` pedal] and neutral-pedal / low-speed crawl release hysteresis [`11.2%–18.6%` pedal], `< 14.2 km/h` paddle creep-taper release vs seamless `< 1.2 km/h` full-stop `i-Pedal` continuity, Reverse `0%`-pedal creep plateau suppression, mode-specific paddle release thresholds, and dedicated Smart Cruise Control braking thresholds).
     3. **Auto-Hold `ON / OFF`, `i-Pedal`, & Smart Cruise Standstill Hold** (`standstill_hold = true`: infers console `AUTO HOLD` ON vs OFF from observed `0%`-pedal idle creep and `parking_brake` actuator release transitions so foot-brake stops with `AUTO HOLD` OFF and early `>= 1.5 km/h` rolling brake releases clear cleanly, while true Auto-Hold, `i-Pedal`, and Smart Cruise standstill holds stay illuminated until throttle `>= 1.5%`, `0.7 km/h` creep, `moving`, EPB, or Park).

2. **Automatic `L0 / L1 / L2 / L3 / i-Pedal` + `Auto Regen` + Smart Cruise Control (`SCC`) Detection**:
   - **History-Ring Coasting Chord (`L0`–`L3+`) & First-Class Auto Regen (`Smart Recuperation`) Tracking**: Automatically classifies `L0`, `L1`, `L2`, and `L3+` whenever the driver coasts off both pedals (`pedal.accel < 1%`, `brake.pressure < 0.25`) above `21 km/h`, using a `520 ms` settle + `1.05–1.95 s` aero-compensated history chord (`0.44 / 0.82 / 1.22 m/s²` boundaries with `±0.08 m/s²` hysteresis) and post-WOT acceleration-crest reset. Detects `Auto Regen` (`(Auto)`) from intra-coast radar modulation (`L0 -> L2/L3`, `L2/L3 -> L0/L1`, or `>= 0.36 m/s²` intra-coast swing), confirmed `L0 -> L2/L3` modulation on `20 Hz` Hubs, and cross-coast return to `L0`, enforcing strict mutual exclusion with `i-Pedal`, clearing fixed-coast counters on any settled `L0 / L1` open-road coast even after `>= 3.5%` throttle resets temporary radar `L2/L3` back to base `L0`, and exiting `Auto` only on `i-Pedal`, Park, or `3` consecutive unmodulated manual `L2/L3` coasts.
   - **`i-Pedal` (`4`) vs `L3` (`3`) Discrimination**: Distinguishes `i-Pedal` from `L3` via post-Park urban pull-away throttle (`29–38.5%` pedal at `8–36 km/h` yielding moderate `0.55–1.65 m/s²` acceleration above `i-Pedal`'s `~22%` neutral zone), post-brake-settled steering-scrub-compensated low-speed zero-pedal full-stop regen (`2–14 km/h` at `>= 1.35 m/s²`), and urban neutral / gentle creep-up pedal (`23–30%` across `16–56 km/h`), stepping back down to `L3` on settled low-speed creep taper (`>= 780 ms`) or low-pedal steady cruising (`3–12%`).
   - **Smart Cruise Control (`SCC / HDA`) Detection**: Detects active SCC from settled zero-pedal acceleration or sustained zero-pedal constant-speed cruise (with an extended `9 s` `L0` slight-downgrade guard), freezing the driver's learned paddle regen mode during SCC stop-and-go and exiting immediately on foot brake, Park/EPB, or sustained manual throttle.

3. **Optional OBD-II Ground-Truth Brake Lamp Reader (`IGMP 770 → 778`, UDS `22 BC06`)**:
   - Reads the physical E-GMP Integrated Gateway (`IGMP`) brake-lamp output byte (`22 BC06`: `0` = OFF; bit `0` = `0` [`4, 36, 40, 44`] = pure Regen / i-Pedal / Smart Cruise / Auto-Hold brake lamps ON; bit `0` = `1` [`37, 41, 45`] = physical foot-brake pedal switch active, including `36/37/40/41` turn-signal stop-channel states) to display exact hardware brake-lamp state and score the HMG-API model.
   - **Coexistence & Native HMG-API BLE Central (`ObdBrakeClient`)**:
     1. **Coexistence Check — Localhost ELM327 Bridge (`127.0.0.1:35000`)**: Automatically detects when [`obd-gen5w`](https://github.com/lemoyneiv/obd-gen5w)'s local TCP ELM327 multiplexer bridge is active (`ATI` check) and yields to `127.0.0.1:35000` so two apps never interleave stateful `ATSH` CAN headers over a shared BLE GATT link. Traffic stays strictly on the Linux kernel loopback interface (`lo`) and never touches `wlan0` or wireless projection.
     2. **Direct Native BLE — HMG-API `@hmgapi-ble` (`hmgapi.client.Ble`)**: Uses HMG-API's built-in `"ble": {"central": true}` stack (`@hmgapi-ble` inside `com.android.bluetooth`) to scan and connect directly over BLE GATT to ELM327/STN scanners (Veepeak OBDCheck BLE, Vgate, OBDLink, Nordic NUS, ISSC, etc.) with uninterrupted manual BLE scan windows, immediate `@hmgapi-ble` stream recycling + wake-scan recovery on reconnect/timeout, sticky `ADV_IND` + `SCAN_RSP` advertisement merging, OBD-first scan priority sorting, automatic recovery from incomplete cold-boot GATT discovery, negotiated ATT MTU chunking, strict LNEX BVM02 12V battery-sensor MAC/GATT exclusion + `PROTECT_CHAR fff1` defense-in-depth, read-only command filtering (blocks VIN `0902`/`22F190` and all UDS write services), and `IG OFF` CAN sleep protection.

4. **Live Accuracy Scorer & Zero-Flash-Write Calibration Dataset (`TelemetryRecorder` + `UsbExporter`)**:
   - Continuously scores the HMG-API model against live OBD `22 BC06` ground truth (`TP`, `FP`, `TN`, `FN`, overall agreement %, precision %, recall %, separate Pedal Switch vs Pure Regen/SCC recall %, and signed onset/release lag in `ms`), and logs deduplicated BLE session/scan diagnostic transitions directly into `brake-summary-<timestamp>.txt`.
   - Buffers up to `36,000` synchronized HMG-API + OBD + Regen Mode rows (~30 minutes at `20 Hz`, automatically downsampled to `1 Hz` while quietly parked) in compact primitive arrays in RAM (zero internal eMMC flash writes).
   - **Export USB** atomically writes `BrakeIndicator/brake-dataset-<timestamp>.csv` and `BrakeIndicator/brake-summary-<timestamp>.txt` (`*.part` + `fsync` + `rename`) to any mounted FAT32/exFAT USB drive.

5. **Top-Bar Overlay & Diagnostic Dashboard (`OverlayService` + `MainActivity`)**:
   - Renders an anti-aliased brake-lamp icon in a non-touchable `TYPE_SYSTEM_OVERLAY` window (`FLAG_NOT_FOCUSABLE | FLAG_NOT_TOUCHABLE | FLAG_LAYOUT_IN_SCREEN`) aligned with the top status bar row (`51×36` px default), visible across the Gen5W launcher, built-in navigation, and wireless projection without intercepting touches.
   - Provides a **Brake Indicator** tile in the Gen5W launcher (and an optional ★ custom button entry) with live HMG-API vs OBD telemetry, detected `L0..i-Pedal` + `(Auto)` / `(SCC)` mode readout, quick **OBD Mode** toggle, instant **Mark Log** bookmark button, **Scan BLE** / **Pair** controls, live transition log, and **Export USB**.

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
| `standstill_hold` | `true` | Keep illuminated at `0 km/h` after an Auto-Hold, regen, or Smart Cruise stop until throttle, creep, EPB, or Park |
| `bridge_port` | `35000` | Localhost TCP ELM327 coexistence/fallback port (`127.0.0.1:<port>`, matching `obd-gen5w`) |

---

## Installation & Updating

Download the latest release assets from [Releases](https://github.com/lemoyneiv/brake-indicator-gen5w/releases):

- **With HMG-API (USB Updater):** Unzip `brake-indicator-gen5w-v0.7.9-hmgapi-usb.zip` to the root of a FAT32/exFAT USB drive (creates `HMGAPI/updates/brake-indicator-gen5w-v0.7.9.zip`), plug into the head unit's data USB port, and install/update from **HMG-API → Updates**.
- **With NEMESIS (USB Auto-Prompt):** Unzip `brake-indicator-gen5w-v0.7.9-nemesis-usb.zip` to the root of a USB drive (`NEMESIS.data` and `packages/hmgapi.brake-indicator-gen5w.zip`) and plug into the head unit once Android has booted. Tapping **Launch** on **Brake Light Indicator** in NEMESIS Manager opens the **Brake Indicator** app directly.
- **Direct Packages:** `brake-indicator-gen5w-v0.7.9.hmgapi.zip` (raw signed HMG-API mod package) and `brake-indicator-gen5w-v0.7.9-nemesis-package.zip` (NEMESIS Package Store / `nemesis install` archive).

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
