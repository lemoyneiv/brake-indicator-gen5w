# Brake Light Indicator for Gen5W (`brake-indicator-gen5w`)

An [HMG-API](https://github.com/Open-HMG/HMG-API) mod package for Hyundai / Kia / Genesis **Gen5W** head units (EV6, Ioniq 5, Ioniq 6, etc.) that displays a non-intrusive **brake light indicator icon** (`(( ● ))`) on the top status bar row of the screen.

On E-GMP and other Hyundai/Kia EVs/HEVs, regenerative braking profiles (Level 1–3, Auto Regen, i-Pedal, and Smart Cruise Control) frequently illuminate the rear brake lamps without the driver touching the brake pedal. Because Gen5W displays no brake-lamp indicator on the cluster or infotainment screen, this mod provides clear in-car visual confirmation whenever the brake lights come on.

---

## Architecture (`v0.10.3`)

`brake-indicator-gen5w` is completely standalone (`"live": true`, `"ble": {"central": true}`, zero `/system` file patches, zero `root` mod daemons) and runs inside a single lightweight Android app (`app/BrakeIndicator.apk`, `OverlayService` + `BrakeEngine`):

1. **Poll-Aligned Evaluation & E-GMP UN ECE R13-H Brake-Lamp Model (`BrakeEngine` + `BrakeModel` + `BrakeEvaluator`)**:
   - Runs a dedicated `50 ms` (`20 Hz`) evaluation and RAM telemetry recorder thread while polling HMG-API's built-in readings (`brake.pressure`, `speed`, `pedal.accel`, `gear.park`, `parking_brake`, `moving`, `steering.angle`, `hv.soc`) over `@hmgapid` (`LFD1`) at `20 Hz` (`< 2 ms` with `100 ms` `speed`/`moving` freshness on updated HMG-API, or `650 ms` with automatic `90 ms` `'O'`-frame recovery on legacy KitKat `Hub`s).
   - Aligns longitudinal `-dv/dt` updates to exact HMG-API poll timestamps (`hmgSampleElapsedMs`), suppresses `< 560 ms` `0.8 km/h` (`0.5 mph`) cluster speed-quantization repeats while bounding stale high deceleration across flat-speed intervals on `20 Hz` Hubs, gates `lastFootBrakeMs` on latched pedal pressure (`pressureLatched`) so sub-latch pedal brushes never cut off active regen, isolates mild post-foot-brake hydraulic & intersection trail-braking turn-scrub (`>= 80°` steering above `18 km/h`) `-dv/dt` tails, and subtracts both passive highway aerodynamic drag excess above `36 km/h` (`10 m/s`) and low-speed full-lock Ackermann tire-scrub drag (`|steering.angle| > 220°` below `18.5 km/h` in `L0 / L1 / Auto`) so wind drag and sharp turns do not masquerade as motor regen torque:
     1. **Hydraulic / IEB Cylinder Pressure** (`brake.pressure` with a responsive `0.4` onset threshold, poll-rate-aware rolling brush sensitivity [`0.26 bar` on `20 Hz` Hubs], gentle low-speed feather hold [`0.18 bar` release], and `409.5` CAN-invalid filtering for instant foot-pedal & IEB hydraulic braking).
     2. **Calibrated Regenerative, Auto Regen (`Smart Recuperation`), Manual `L3`, One-Pedal `i-Pedal`, & Smart Cruise Deceleration** (`signal_mode = pressure_and_decel`, enabled by default: models E-GMP's UN ECE R13-H regen brake-lamp law with first-class Auto-Regen detection [suppressing open-road lift-off anticipation, passive `L1 (Auto)` base highway coasting below `0.85 m/s²`, and light-throttle cold-onset false positives, treating `< 1.7%` highway pedal deadzone as off-pedal coast, preserving light `1–12.8%` throttle feathering while active radar regen stays above R13-H hold thresholds, and applying speed-banded `< 14.2 / 14.2–39.5 / >= 39.5 km/h` onset & settled-coast release hysteresis], cruise vs post-acceleration VCU torque-ramp lift-off timing in manual `L2 / L3 / i-Pedal` [including shift-interlock-verified `< 7 km/h` forward `0%`-pedal `i-Pedal` liftoff while preserving Reverse `0%`-pedal creep suppression], dedicated speed-scaled `L3` feathered highway braking [`6.5%–14.8%` onset / `7.5%–16.4%` release across `38–110+ km/h`], piecewise crawl-to-highway speed-dependent `i-Pedal` feathered one-pedal braking onset [`1.8%–22.6%` pedal across `1.5–125+ km/h`, with onset-relative highway motor-regen confirmation and sharp-steering cornering boost] and neutral-pedal / low-speed crawl release hysteresis [`2.1%–24.8%` pedal across `1.5–92+ km/h` plus straightaway throttle-reapply `-dv/dt` tail release], `< 14.2 km/h` paddle creep-taper release vs seamless `< 1.2 km/h` full-stop `i-Pedal` continuity, mode-specific paddle release thresholds, and dedicated Smart Cruise Control braking thresholds).
     3. **Auto-Hold `ON / OFF`, `i-Pedal`, & Smart Cruise Standstill Hold** (`standstill_hold = true`: infers console `AUTO HOLD` ON vs OFF from observed `0%`-pedal idle creep and `parking_brake` actuator release transitions so foot-brake stops with `AUTO HOLD` OFF and early `>= 1.5 km/h` rolling brake releases clear cleanly, while true Auto-Hold, `i-Pedal` [`< 0.6%` latch / `< 1.0%` hold], and Smart Cruise standstill holds stay illuminated until throttle `>= 1.0%`, `0.7 km/h` creep, `moving`, EPB, or Park).

2. **Automatic `L0 / L1 / L2 / L3 / i-Pedal` + `Auto Regen` + Smart Cruise Control (`SCC`) Detection**:
   - **History-Ring Coasting Chord (`L0`–`L3+`) & First-Class Auto Regen (`Smart Recuperation`) Tracking**: Automatically classifies `L0`, `L1`, `L2`, and `L3+` whenever the driver coasts off both pedals (`pedal.accel < 1%`, `brake.pressure < 0.25`) above `21 km/h` (preserving `L3` across `< 26.5 km/h` low-speed creep taper), using a `520 ms` settle + `1.05–1.95 s` aero-compensated history chord (`0.44 / 0.82 / 1.22 m/s²` boundaries with `±0.08 m/s²` hysteresis) and post-WOT acceleration-crest reset. Detects `Auto Regen` (`(Auto)`) from intra-coast radar modulation (`L0 -> L2/L3`, `L2/L3 -> L0/L1`, or `>= 0.36 m/s²` intra-coast swing), confirmed `L0 -> L2/L3` modulation on `20 Hz` Hubs, and cross-coast return to `L0`, enforcing strict mutual exclusion with `i-Pedal`, clearing fixed-coast counters on any settled `L0 / L1` open-road coast even after `>= 3.5%` throttle resets temporary radar `L2/L3` back to base `L0`, and exiting `Auto` only on `i-Pedal`, Park, or `3` consecutive unmodulated manual `L2/L3` coasts.
   - **`i-Pedal` (`4`) vs `L3` (`3`) Shifted-Neutral Discrimination**: Accounts for E-GMP's ignition-cycle `i-Pedal → L3` reset and `L3`'s upward-shifted neutral-pedal curve (`7–11%` urban, `20–21%` at `45–60 km/h`, `27–31%` at `70–100 km/h`—sitting `5–12%` below `i-Pedal` across all speeds). Distinguishes `i-Pedal` from `L3` and `Auto Regen` via post-Park cold-start urban pull-away throttle (`29.5–38.5%` pedal at `8–36 km/h`), post-brake-settled steering-scrub-compensated low-speed zero-pedal full-stop regen (`2–14 km/h` at `>= 1.35 m/s²`), speed-scaled urban & highway neutral-pedal cruise strictly above `L3`'s neutral map (`20.8–31%` across `16–56 km/h` and `24.6–50%` across `56–125 km/h`), and feathered one-pedal braking above `L3`'s highway feathering envelope (`13.5–22.5%` pedal sustaining `>= 0.72–0.86 m/s²` motor regen or `6.8–13.5%` sustaining `>= 1.05 m/s²` at `18–44 km/h`), stepping back down `i-Pedal → L3` on settled low-speed creep taper (`>= 780 ms`) or speed-scaled sub-`i-Pedal` flat cruising (`3–12%` urban up to `24%` at `>= 90 km/h`).
   - **Smart Cruise Control (`SCC / HDA`) Detection**: Detects active SCC from settled zero-pedal acceleration or sustained zero-pedal constant-speed cruise (with an extended `9 s` `L0` slight-downgrade guard), freezing the driver's learned paddle regen mode during SCC stop-and-go and exiting immediately on foot brake, Park/EPB, or sustained manual throttle.

3. **Modular ECDSA-Signed OTA Brake Model Updates over Built-In LTE / Wi-Fi (`BrakeModel` + `ModelBundleVerifier` + `ModelManager`)**:
   - Decouples the host APK (UI, overlay, `@hmgapid` poller, `@hmgapi-ble` OBD-II stack, TLS 1.2 uploader) from the brake-lamp & regen-inference algorithm (`BrakeModel` interface, `HOST_MODEL_API_VERSION = 1`) so future calibration curves and new state-machine logic ship as ~`16 KB` downloadable `brake-model-<ver>.jar` bundles (`classes.dex`, `min-api 19`) without a USB app reinstall.
   - **Pinned ECDSA P-256 Verification (`ModelBundleVerifier`)**: Every `.jar` is signed with the mod author's `secp256r1` (`SHA256withECDSA`) key (`UUID = 28ad71e0-07c0-8e61-8bf3-380cc71f9280`, pinned inside the APK). No unverified or downgraded (`model_seq <= activeSeq`) bytecode is ever written to disk or passed to Dalvik `DexClassLoader`.
   - **Zero-Token Public `ota` Channel + Zero-Flash-Wear Polling (`ModelManager`)**: Fetches `latest.json` and `models/brake-model-<ver>.jar` directly from the public `ota` branch of `lemoyneiv/brake-indicator-gen5w` over `api.github.com` (`Accept: application/vnd.github.raw+json`)—requiring **zero GitHub PAT** and **zero S3 redirects**—automatically when shifting into **Park** (or once per boot while Parked, `auto_update_model = true`), keeping `ETag` (`HTTP 304 Not Modified`) and `15 min` cooldown state strictly in RAM so up-to-date checks cause **zero internal eMMC writes**.
   - **6-Tick Pre-Swap Self-Test, Same-Tick Watchdog Fallback, & Manual Reset**: Runs a 6-scenario deterministic sanity test before activating any new model, wraps every `50 ms` evaluation tick in a `try/catch (Throwable)` watchdog that falls back on the **exact same tick** to the compiled-in `BrakeEvaluator` and quarantines any faulting plugin, automatically retires older cached OTA models whenever a newer full APK is installed, and provides an **`Update Model`** sidebar button (tap to check GitHub `ota` or import an offline signed `<usb>/BrakeIndicator/brake-model-*.jar`; long-press to revert to the built-in APK model).

4. **Optional OBD-II Ground-Truth Brake Lamp Reader (`IGMP 770 → 778`, UDS `22 BC06`)**:
   - Reads the physical E-GMP Integrated Gateway (`IGMP`) brake-lamp output byte (`22 BC06`: `0` = OFF; bit `0` = `0` [`4, 36, 40, 44`] = pure Regen / i-Pedal / Smart Cruise / Auto-Hold brake lamps ON; bit `0` = `1` [`37, 41, 45`] = physical foot-brake pedal switch active, including `36/37/40/41` turn-signal stop-channel states) to display exact hardware brake-lamp state and score the HMG-API model.
   - **Coexistence & Native HMG-API BLE Central (`ObdBrakeClient`)**:
     1. **Coexistence Check — Localhost ELM327 Bridge (`127.0.0.1:35000`)**: Automatically detects when [`obd-gen5w`](https://github.com/lemoyneiv/obd-gen5w)'s local TCP ELM327 multiplexer bridge is active (`ATI` check) and yields to `127.0.0.1:35000` so two apps never interleave stateful `ATSH` CAN headers over a shared BLE GATT link. Traffic stays strictly on the Linux kernel loopback interface (`lo`) and never touches `wlan0` or wireless projection.
     2. **Direct Native BLE — HMG-API `@hmgapi-ble` (`hmgapi.client.Ble`)**: Uses HMG-API's built-in `"ble": {"central": true}` stack (`@hmgapi-ble` inside `com.android.bluetooth`) to scan and connect directly over BLE GATT to ELM327/STN scanners (Veepeak OBDCheck BLE, Vgate, OBDLink, Nordic NUS, ISSC, etc.) with uninterrupted manual BLE scan windows, immediate `@hmgapi-ble` stream recycling + wake-scan recovery on reconnect/timeout, sticky `ADV_IND` + `SCAN_RSP` advertisement merging, OBD-first scan priority sorting, automatic recovery from incomplete cold-boot GATT discovery, negotiated ATT MTU chunking, strict LNEX BVM02 12V battery-sensor MAC/GATT exclusion + `PROTECT_CHAR fff1` defense-in-depth, read-only command filtering (blocks VIN `0902`/`22F190` and all UDS write services), and `IG OFF` CAN sleep protection.

5. **Driver Ground-Truth Regen & Auto-Hold Labeling, Live Scorer, Cloud Upload, & USB Fallback (`TelemetryRecorder` + `AutoExportCoordinator` + `GitHubUploader` + `TlsHttpClient` + `UsbExporter`)**:
   - **Zero-Leakage Driver Ground-Truth Telemetry**: Captures optional 1-tap driver confirmations of `i-Pedal`, `L3`, `L2`, `L1`, `L0`, `Auto Regen` (`Smart Recuperation`), `SCC Cruise`, and console `Auto-Hold` (`ON / OFF / ?`) directly into the `20 Hz` CSV (`inferred_auto_regen,inferred_scc,driver_regen_level,driver_auto_regen,driver_scc,driver_autohold,driver_gt_age_s,ui_focused`) without ever feeding driver labels into `BrakeModel.evaluate(...)`. Enforces E-GMP hardware rules (`i-Pedal` vs `Auto` mutual exclusion, automatic `SCC` clear on foot brake, preservation of pre-Drive Park confirmations, and automatic regen-label reset on post-drive shift to Park), tracks tap-only confirmation age (`driver_gt_age_s` resets only on explicit button taps, never on reopening the app), and separates **High-Confidence** (`<= 90s` focused) vs **Carry-Forward** (`> 90s` or post-backgrounding) regen accuracy in `brake-summary-<timestamp>.txt`.
   - Continuously scores the active `BrakeModel` against live OBD `22 BC06` ground truth (`TP`, `FP`, `TN`, `FN`, overall agreement %, precision %, recall %, separate Pedal Switch vs Pure Regen/SCC recall %, and signed onset/release lag in `ms`), stamps `Active Brake Model` (`builtin` vs `OTA` + SHA-256) into `brake-summary-<timestamp>.txt` and cloud upload commit messages, and logs deduplicated BLE/OTA/DriverGT transitions.
   - Buffers up to `36,000` synchronized HMG-API + OBD + Regen Mode + Driver Ground-Truth rows (~30 minutes at `20 Hz`, automatically downsampled to `1 Hz` while quietly parked) in compact primitive arrays in RAM (zero internal eMMC flash writes).
   - **Automatic Compressed GitHub Cloud Upload over Built-In LTE / Wi-Fi (`auto_upload_cloud = true` by default)**:
     - Compresses `brake-dataset-<stamp>.csv` + `brake-summary-<stamp>.txt` purely in RAM into a standard POSIX `.tar.gz` archive (`drives/YYYY-MM-DD/brake-drive-<stamp>-<trigger>.tar.gz`, shrinking a 30-min `~3.8 MB` drive to `~250–320 KB`) and uploads it via HTTPS (`PUT /repos/<owner>/<repo>/contents/...`) on **shift to Park** or **95% RAM buffer capacity** (`34,200 / 36,000` rows).
     - Includes a self-contained pure-Java **TLS 1.2** stack (`ECDHE_{ECDSA,RSA}_WITH_AES_{128,256}_GCM_SHA{256,384}`, FIPS 186-4 P-256/P-384, NIST SP 800-38D AES-GCM) and **17 embedded modern root CAs** (`PinnedRoots`) so Android 4.4.2 (`API 19`) Gen5W head units connect securely to `api.github.com` without relying on KitKat's 2014 `/system/etc/security/cacerts` store or TLS 1.0/1.1 defaults.
     - Detects both Android `ConnectivityManager` networks and Gen5W kernel `/proc/net/route` telematics default routes (`rmnet*` / `ccmni*` / `usb0`) even while `wlan0` serves wireless CarPlay / Android Auto, enforces an in-RAM `5 MB/day` LTE byte cap (`DailyByteBudget`), and holds up to `4` compressed drives (`<= 2 MB` ceiling) in a zero-flash-wear in-RAM `PendingUploadQueue` when parked in an underground garage—draining automatically once LTE/Wi-Fi returns or unpacking to USB if a flash drive is plugged in.
     - **One-Time Zero-Typing USB Provisioning (`<usb>/BrakeIndicator/cloud-upload.json`)**: Drop a Fine-Grained GitHub PAT (`Contents: Read and write` scoped strictly to a dedicated private logs repository) into `BrakeIndicator/cloud-upload.json` on a USB drive once; the app imports the config into private `SharedPreferences` and atomically scrubs the plaintext `"token"` field on the USB stick (`"token_imported_at"` receipt).
   - **Automatic & Manual USB Fallback (`auto_export_usb = true` by default)**: Whenever cloud upload is unconfigured, disabled, or offline with a FAT32/exFAT USB stick attached (or when `usb_backup_when_cloud = true`), atomically writes plain `BrakeIndicator/brake-dataset-<timestamp>.csv` and `BrakeIndicator/brake-summary-<timestamp>.txt` (`*.part` + `fsync` + `rename`) and clears the exported rows cleanly.

6. **Top-Bar Overlay & 3-Column Material 3 Widescreen Dashboard (`OverlayService` + `MainActivity`)**:
   - Renders an anti-aliased brake-lamp icon in a non-touchable `TYPE_SYSTEM_OVERLAY` window (`FLAG_NOT_FOCUSABLE | FLAG_NOT_TOUCHABLE | FLAG_LAYOUT_IN_SCREEN`) aligned with the top status bar row (`51×36` px default), visible across the Gen5W launcher, built-in navigation, and wireless projection without intercepting touches.
   - Provides a **Brake Indicator** tile in the Gen5W launcher (and an optional ★ custom button entry) styled in **Material 3 Dark**:
     1. **Left Control Sidebar**: Live `BRAKE ON/OFF` + `OBD` status card, `Mark Log`, **OBD Mode** toggle, `Test (3s)` | `Export USB`, `Upload Cloud` | `Test Cloud` 4-stage LTE/TLS/GitHub diagnostic, `Update Model` (long-press to revert to built-in), `Clear` | `BLE & Log`, and `Close`.
     2. **Center Regen Ground-Truth Training Card** (closest to driver): Large zero-scroll touch controls for **`i-Pedal`**, **`L3` | `L2`**, **`L1` | `L0`**, **`Auto Regen` | `SCC Cruise`**, **`Unset`**, and **`Auto-Hold: ON / OFF / ?`**, with live `CONFIRMED` vs `CARRY-FWD` styling and instant `Model ✓/✗` comparison.
     3. **Right Full-Height Slot**: Full-vertical-height driving-legible **Status Monitor** (`Real OBD` through `RAM Dataset Buffer` at `27 px` bold font) by default, swapping on demand via **`BLE & Log`** to the hidden **OBD-II BLE & Transition Log** panel (`Scan BLE`, `Reconnect`, `Unpair`, `Hide`, BLE pairing list, and scrollable transition log).

---

## One-Time Cloud Upload Provisioning (Optional)

To stream compressed `.tar.gz` drive logs over the car's built-in LTE modem or Wi-Fi without plugging in a USB drive after every drive:

1. Create a dedicated **private** GitHub repository (for example, `<owner>/brake-indicator-logs`).
2. Generate a **Fine-Grained Personal Access Token** (`GitHub Settings → Developer settings → Personal access tokens → Fine-grained tokens`) with:
   - **Repository access:** *Only select repositories* → your private logs repository
   - **Repository permissions:** `Contents: Read and write` (all other permissions `No access`)
3. Save `BrakeIndicator/cloud-upload.json` on a FAT32/exFAT USB drive:
   ```json
   {
     "enabled": true,
     "repo": "owner/brake-indicator-logs",
     "branch": "main",
     "token": "github_pat_YOUR_FINE_GRAINED_TOKEN_HERE",
     "allow_lte": true,
     "usb_backup_too": false,
     "daily_limit_kb": 5120
   }
   ```
4. Plug the USB stick into the head unit once (with USB ADB off) and tap **`Test Cloud`** in **Brake Indicator**. The app imports your credentials, scrubs `"token"` from the USB stick, and verifies `Network → DNS → TCP :443 → TLS 1.2 → GitHub API` write access.

> **Note:** OTA Brake Model updates (`auto_update_model`) fetch public ECDSA-signed `.jar` bundles from `lemoyneiv/brake-indicator-gen5w@ota` and work out-of-the-box over LTE/Wi-Fi **without** needing `cloud-upload.json`.

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
| `auto_update_model` | `true` | Automatically check for and verify ECDSA-signed OTA brake model updates (`ota` branch) when Parked over LTE/Wi-Fi |
| `auto_upload_cloud` | `true` | Automatically compress and upload drive logs (`.tar.gz`) to GitHub when `cloud-upload.json` credentials have been imported |
| `cloud_allow_lte` | `true` | Allow automatic cloud uploads and automatic Park OTA model checks over the head unit's built-in cellular/LTE modem (`false` restricts auto network sync to Wi-Fi / hotspot) |
| `usb_backup_when_cloud` | `false` | Also write plain `.csv` + `.txt` copies to a mounted USB drive even when cloud upload is active |
| `auto_export_usb` | `true` | Automatically export CSV + summary to `<usb>/BrakeIndicator/` on shifting into Park or reaching 95% RAM buffer capacity when a USB stick is attached (acts as automatic offline fallback when cloud upload is unconfigured or unreachable), then clear the buffer |
| `bridge_port` | `35000` | Localhost TCP ELM327 coexistence/fallback port (`127.0.0.1:<port>`, matching `obd-gen5w`) |

---

## Installation & Updating

Download the latest release assets from [Releases](https://github.com/lemoyneiv/brake-indicator-gen5w/releases):

- **With HMG-API (USB Updater):** Unzip `brake-indicator-gen5w-v0.10.3-hmgapi-usb.zip` to the root of a FAT32/exFAT USB drive (creates `HMGAPI/updates/brake-indicator-gen5w-v0.10.3.zip`), plug into the head unit's data USB port, and install/update from **HMG-API → Updates**.
- **With NEMESIS (USB Auto-Prompt):** Unzip `brake-indicator-gen5w-v0.10.3-nemesis-usb.zip` to the root of a USB drive (`NEMESIS.data` and `packages/hmgapi.brake-indicator-gen5w.zip`) and plug into the head unit once Android has booted. Tapping **Launch** on **Brake Light Indicator** in NEMESIS Manager opens the **Brake Indicator** app directly.
- **Direct Packages:** `brake-indicator-gen5w-v0.10.3.hmgapi.zip` (raw signed HMG-API mod package) and `brake-indicator-gen5w-v0.10.3-nemesis-package.zip` (NEMESIS Package Store / `nemesis install` archive).

> **Note on USB Provisioning / USB Export & USB Host Mode:** When importing `BrakeIndicator/cloud-upload.json`, importing an offline `BrakeIndicator/brake-model-*.jar`, or exporting calibration CSV datasets to USB, make sure **USB ADB** is turned off so the head unit's front data USB port operates in USB host mode (able to mount a FAT32/exFAT flash drive).

---

## Building & Signing Installable Packages & OTA Models

Requires Java 17+, Python 3, the Android SDK (`ANDROID_HOME` with `platforms/android-19` and `build-tools`), and an unpacked HMG-API Mod SDK (`HMGAPI_SDK` or `../HMG-API`):

```sh
./build.sh        # Full host APK + HMG-API & NEMESIS installable .zip packages into dist/
./build-model.sh  # Standalone ~16 KB ECDSA-signed OTA brake-model-<ver>.jar + latest.json into dist-model/
```

Pushing a `model-v*` tag (for example `model-v2026.11.19.1` after bumping `ota-model.json`) triggers `.github/workflows/publish-model.yml` to run the host unit test suite, sign `brake-model-<ver>.jar` with `MOD_KEY`, and publish `latest.json` + `models/brake-model-<ver>.jar` onto the `ota` branch for automatic in-car download.

## License

GNU Affero General Public License v3.0 ([LICENSE](LICENSE)), matching [Open-HMG/HMG-API](https://github.com/Open-HMG/HMG-API).
