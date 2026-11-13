package dev.lemoyne.brakeindicator;

import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Pure Java ELM327 / STN BLE & TCP framing, GATT characteristic resolver,
 * LNEX BVM02 hardware safety exclusion, ISO 15765-4 (ISO-TP) reassembler,
 * and Hyundai/Kia E-GMP IGMP (`770` -> `778`, UDS `22 BC06`) brake-lamp decoder.
 */
public final class Elm327BrakeProtocol {

    public static final Charset ASCII = Charset.forName("US-ASCII");

    // BLE UART profiles used across Veepeak OBDCheck BLE, Vgate, OBDLink, ISSC, and Nordic NUS
    public static final String UUID_ISSC_SERVICE  = "49535343-fe7d-4ae5-8fa9-9fafd205e455";
    public static final String UUID_ISSC_NOTIFY   = "49535343-1e4d-4bd9-ba61-23c647249616";
    public static final String UUID_ISSC_WRITE    = "49535343-8841-43f4-a8d4-ecbe34729bb3";
    public static final String UUID_ISSC_AIRPATCH = "49535343-aca3-481c-91ec-d85e28a60318";

    public static final String UUID_NUS_SERVICE   = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
    public static final String UUID_NUS_WRITE     = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
    public static final String UUID_NUS_NOTIFY    = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

    public static final String UUID_VLINK_SERVICE = "e7810a71-73ae-499d-8c15-faa9aef0c3f2";
    public static final String UUID_VLINK_RW_NOT  = "bef8d6c9-9c21-4c9e-b632-bd58c1009f9f";

    // BluetoothGattCharacteristic property bitmasks (API 19)
    public static final int PROP_READ          = 0x02;
    public static final int PROP_WRITE_NO_RESP = 0x04;
    public static final int PROP_WRITE         = 0x08;
    public static final int PROP_NOTIFY        = 0x10;
    public static final int PROP_INDICATE      = 0x20;

    /**
     * E-GMP IGMP (`770` -> `778`) fast brake-lamp UDS polling setup:
     * 11-bit 500 kbps ISO 15765-4 CAN, headers ON (`ATH1`), spaces OFF (`ATS0`),
     * `ATST19` (~100 ms timeout), targeting IGMP `770` with receive filter `778`.
     */
    public static final String[] IGMP_INIT_COMMANDS = {
        "ATE0",
        "ATL0",
        "ATS0",
        "ATH1",
        "ATSP6",
        "ATCAF1",
        "ATCFC1",
        "ATAT1",
        "ATST19",
        "ATSH770",
        "ATCRA778"
    };

    public static final String CMD_IGMP_BRAKE_LAMP = "22BC06";
    public static final int IGMP_RX_CAN_ID = 0x778;

    private static final String[] KNOWN_OBD_NAME_TOKENS = {
        "obdcheck",
        "veepeak",
        "obdii",
        "obd-ii",
        "obd2",
        "obdble",
        "ios-vlink",
        "v-link",
        "vgate",
        "icar",
        "elm327",
        "carista",
        "konnwei",
        "nexas",
        "viecar",
        "obdlink"
    };

    private static final String[] KNOWN_BVM02_NAME_TOKENS = {
        "bvm02",
        "bl-",
        "carcell",
        "batterysense",
        "batmonitor"
    };

    private Elm327BrakeProtocol() {}

    public static final class GattBinding {
        public final String profileName;
        public final String notifyUuid;
        public final String writeUuid;
        public final boolean writeWithoutResponse;
        public final int writeProps;

        public GattBinding(String profileName, String notifyUuid, String writeUuid,
                           boolean writeWithoutResponse, int writeProps) {
            this.profileName = profileName;
            this.notifyUuid = notifyUuid;
            this.writeUuid = writeUuid;
            this.writeWithoutResponse = writeWithoutResponse;
            this.writeProps = writeProps;
        }

        public boolean supportsAlternateWriteMode() {
            return (writeProps & (PROP_WRITE | PROP_WRITE_NO_RESP)) == (PROP_WRITE | PROP_WRITE_NO_RESP);
        }

        public GattBinding withWriteWithoutResponse(boolean wnr) {
            return new GattBinding(profileName, notifyUuid, writeUuid, wnr, writeProps);
        }
    }

    public static final class IsoTpMessage {
        public final int canId;
        public final byte[] payload;

        public IsoTpMessage(int canId, byte[] payload) {
            this.canId = canId;
            this.payload = payload;
        }
    }

    /**
     * Decoded Hyundai/Kia E-GMP IGMP (`770` -> `778`, `22 BC06`) brake-lamp status.
     *
     * Verified across 2022-2025 Kia EV6 and Hyundai Ioniq 5/6 signalsets & real-vehicle frames:
     * - `payload` = `[0x62, 0xBC, 0x06, A, B, C, D, E, ...]` (8 to 11+ bytes)
     * - `rawCode = payload[7] & 0xFF` (data byte E, bitOffset 32):
     *     - `0` (`0x00`)                   -> Brake lights OFF
     *     - `4` (`0x04`), `36` (`0x24`)    -> Brake lights ON via Regen / Smart Cruise Control (no pedal)
     *     - `40` (`0x28`), `41` (`0x29`),
     *       `44` (`0x2C`), `45` (`0x2D`)   -> Brake lights ON with brake pedal / brake switch active
     */
    public static final class IgmpBrakeReading {
        public final int rawCode;
        public final boolean brakeLampLit;
        public final boolean pedalBit;
        public final String rawPayloadHex;

        public IgmpBrakeReading(int rawCode, boolean brakeLampLit, boolean pedalBit, String rawPayloadHex) {
            this.rawCode = rawCode;
            this.brakeLampLit = brakeLampLit;
            this.pedalBit = pedalBit;
            this.rawPayloadHex = rawPayloadHex == null ? "" : rawPayloadHex;
        }
    }

    public static final class PromptBuffer {
        private static final int MAX_BUFFER_CHARS = 8192;
        private final StringBuilder sb = new StringBuilder(256);

        public synchronized List<String> feed(byte[] chunk) {
            return feed(chunk, 0, chunk == null ? 0 : chunk.length);
        }

        public synchronized List<String> feed(byte[] chunk, int off, int len) {
            List<String> completed = new ArrayList<String>();
            if (chunk == null || len <= 0) {
                return completed;
            }
            int end = Math.min(chunk.length, off + len);
            for (int i = Math.max(0, off); i < end; i++) {
                int b = chunk[i] & 0xFF;
                if (b == 0) {
                    continue;
                }
                char c = (char) b;
                if (c == '>') {
                    completed.add(sb.toString().trim());
                    sb.setLength(0);
                } else {
                    if (sb.length() >= MAX_BUFFER_CHARS) {
                        sb.delete(0, sb.length() - 1024);
                    }
                    sb.append(c);
                }
            }
            return completed;
        }

        public synchronized void reset() {
            sb.setLength(0);
        }
    }

    public static String normUuid(String raw) {
        if (raw == null) return "";
        String s = raw.trim().toLowerCase(Locale.US);
        if (s.startsWith("0x")) s = s.substring(2);
        if (s.length() == 36 && s.startsWith("0000") && s.endsWith("-0000-1000-8000-00805f9b34fb")) {
            return s.substring(4, 8);
        }
        return s;
    }

    public static String expandFullUuid(String normOrFull) {
        String n = normUuid(normOrFull);
        if (n.length() == 4) {
            return "0000" + n + "-0000-1000-8000-00805f9b34fb";
        }
        return n;
    }

    /**
     * Hardware safety check: identifies LNEX BVM02 12V battery sensors so we NEVER pair
     * or write to BVM02's `0xFFF1` firmware characteristic.
     */
    public static boolean isLikelyBvm02Device(String localName) {
        if (localName == null) {
            return false;
        }
        String lower = localName.trim().toLowerCase(Locale.US);
        for (int i = 0; i < KNOWN_BVM02_NAME_TOKENS.length; i++) {
            if (lower.contains(KNOWN_BVM02_NAME_TOKENS[i])) {
                return true;
            }
        }
        return false;
    }

    public static boolean isLikelyObdAdapter(String localName, List<String> serviceUuids) {
        if (isLikelyBvm02Device(localName)) {
            return false;
        }
        if (localName != null) {
            String lower = localName.trim().toLowerCase(Locale.US);
            for (int i = 0; i < KNOWN_OBD_NAME_TOKENS.length; i++) {
                if (lower.contains(KNOWN_OBD_NAME_TOKENS[i])) {
                    return true;
                }
            }
        }
        if (serviceUuids != null) {
            for (int i = 0; i < serviceUuids.size(); i++) {
                String norm = normUuid(serviceUuids.get(i));
                if (UUID_ISSC_SERVICE.equals(norm)
                        || UUID_NUS_SERVICE.equals(norm)
                        || UUID_VLINK_SERVICE.equals(norm)
                        || "18f0".equals(norm)
                        || "ffe0".equals(norm)
                        || "ffe5".equals(norm)
                        || "ae00".equals(norm)
                        || "ae30".equals(norm)
                        || "e010".equals(norm)
                        || "fff0".equals(norm)) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Resolves candidate ELM327 GATT notify/write characteristic pairs from `GATT_READY`,
     * strictly rejecting LNEX BVM02 battery sensors (`fff4` NOTIFY with read-only `fff1`).
     */
    public static List<GattBinding> resolveAdapterBindings(Map<String, Integer> charPropsByUuid) {
        if (charPropsByUuid == null || charPropsByUuid.isEmpty()) {
            return Collections.emptyList();
        }
        Map<String, Integer> normProps = new LinkedHashMap<String, Integer>();
        for (Map.Entry<String, Integer> e : charPropsByUuid.entrySet()) {
            if (e.getKey() != null && e.getValue() != null) {
                normProps.put(normUuid(e.getKey()), e.getValue());
            }
        }

        Integer pFff1 = normProps.get("fff1");
        Integer pFff4 = normProps.get("fff4");
        boolean hasBvmCmdNotify = pFff4 != null && (pFff4.intValue() & (PROP_NOTIFY | PROP_INDICATE)) != 0;
        if (hasBvmCmdNotify) {
            boolean fff1Active = pFff1 != null
                    && (pFff1.intValue() & (PROP_NOTIFY | PROP_INDICATE | PROP_WRITE | PROP_WRITE_NO_RESP)) != 0;
            if (!fff1Active) {
                return Collections.emptyList();
            }
        }

        List<GattBinding> out = new ArrayList<GattBinding>();
        Set<String> seenPairs = new HashSet<String>();

        addCandidateIfValid(out, seenPairs, normProps, "FFF0 (Veepeak BLE)", "fff1", "fff2");
        addCandidateIfValid(out, seenPairs, normProps, "18F0 (Dual-Mode BLE)", "2af0", "2af1");
        addCandidateIfValid(out, seenPairs, normProps, "18F0 Reversed", "2af1", "2af0");
        if (!hasBvmCmdNotify) {
            addCandidateIfValid(out, seenPairs, normProps, "FFF0 Reversed", "fff2", "fff1");
        }
        addCandidateIfValid(out, seenPairs, normProps, "ISSC UART", UUID_ISSC_NOTIFY, UUID_ISSC_WRITE);
        addCandidateIfValid(out, seenPairs, normProps, "ISSC Reversed", UUID_ISSC_WRITE, UUID_ISSC_NOTIFY);
        addCandidateIfValid(out, seenPairs, normProps, "ISSC AirPatch", UUID_ISSC_AIRPATCH, UUID_ISSC_AIRPATCH);
        addCandidateIfValid(out, seenPairs, normProps, "Nordic NUS", UUID_NUS_NOTIFY, UUID_NUS_WRITE);
        addCandidateIfValid(out, seenPairs, normProps, "FFE0/FFE5 UART", "ffe4", "ffe9");
        addCandidateIfValid(out, seenPairs, normProps, "FFE0 Dual-Char", "ffe1", "ffe2");
        addCandidateIfValid(out, seenPairs, normProps, "FFE0 Reversed", "ffe2", "ffe1");
        addCandidateIfValid(out, seenPairs, normProps, "FFE0 Single-Char", "ffe1", "ffe1");
        addCandidateIfValid(out, seenPairs, normProps, "Vlink BLE UART", UUID_VLINK_RW_NOT, UUID_VLINK_RW_NOT);
        if (!hasBvmCmdNotify) {
            addCandidateIfValid(out, seenPairs, normProps, "FFF0 Single-Char (FFF1)", "fff1", "fff1");
            addCandidateIfValid(out, seenPairs, normProps, "FFF0 Single-Char (FFF2)", "fff2", "fff2");
        }
        addCandidateIfValid(out, seenPairs, normProps, "18F0 Single-Char (2AF0)", "2af0", "2af0");
        addCandidateIfValid(out, seenPairs, normProps, "AE00 BLE UART", "ae02", "ae01");
        addCandidateIfValid(out, seenPairs, normProps, "AE00 Reversed", "ae01", "ae02");

        return Collections.unmodifiableList(out);
    }

    private static void addCandidateIfValid(
            List<GattBinding> out,
            Set<String> seenPairs,
            Map<String, Integer> normProps,
            String profileName,
            String notifyNorm,
            String writeNorm) {
        Integer pnObj = normProps.get(notifyNorm);
        Integer pwObj = normProps.get(writeNorm);
        if (pnObj == null || pwObj == null) return;
        int pn = pnObj.intValue();
        int pw = pwObj.intValue();
        if ((pn & (PROP_NOTIFY | PROP_INDICATE)) == 0) return;
        if ((pw & (PROP_WRITE | PROP_WRITE_NO_RESP)) == 0) return;
        if (!seenPairs.add(notifyNorm + "->" + writeNorm)) return;
        boolean wnr = (pw & PROP_WRITE_NO_RESP) != 0;
        out.add(new GattBinding(profileName, expandFullUuid(notifyNorm), expandFullUuid(writeNorm), wnr, pw));
    }

    /**
     * Strict read-only safety guardrail: blocks VIN queries (`0902` / `22F190`) and any UDS write/routine
     * service, allowing only `AT*` setup commands and read-only `22BC06` IGMP brake-lamp polling.
     */
    public static boolean isSafeCommand(String cmd) {
        if (cmd == null) return false;
        String norm = stripSpacesUpper(cmd);
        if (norm.isEmpty()) return false;
        if (norm.startsWith("0902") || norm.startsWith("22F190") || norm.startsWith("1A90")) {
            return false;
        }
        if (norm.startsWith("AT")) {
            return true;
        }
        return CMD_IGMP_BRAKE_LAMP.equals(norm);
    }

    public static List<byte[]> encodeCommandChunks(String command, int maxChunkBytes) {
        List<byte[]> chunks = new ArrayList<byte[]>();
        if (command == null) return chunks;
        byte[] raw = (command.trim() + "\r").getBytes(ASCII);
        int limit = Math.max(1, maxChunkBytes);
        for (int off = 0; off < raw.length; off += limit) {
            int len = Math.min(limit, raw.length - off);
            byte[] part = new byte[len];
            System.arraycopy(raw, off, part, 0, len);
            chunks.add(part);
        }
        return chunks;
    }

    public static String cleanAdapterVersion(String raw) {
        if (raw == null) return "";
        String s = raw.replace('\r', ' ').replace('\n', ' ').trim();
        while (s.regionMatches(true, 0, "ATI ", 0, 4) || s.regionMatches(true, 0, "ATZ ", 0, 4)) {
            s = s.substring(4).trim();
        }
        if ("ATI".equalsIgnoreCase(s) || "ATZ".equalsIgnoreCase(s)) {
            return "";
        }
        s = s.replaceAll("\\s{2,}", " ").trim();
        return s.length() > 28 ? s.substring(0, 28).trim() : s;
    }

    public static boolean isErrorOrNoData(String rawResponse) {
        if (rawResponse == null) return true;
        String up = rawResponse.toUpperCase(Locale.US);
        return up.contains("NO DATA")
                || up.contains("UNABLE TO CONNECT")
                || up.contains("CAN ERROR")
                || up.contains("BUS INIT: ...ERROR")
                || up.contains("STOPPED")
                || up.trim().equals("?");
    }

    /**
     * Decodes a raw ELM327 `22BC06` response block into an {@link IgmpBrakeReading},
     * or returns {@code null} if no valid positive `62 BC 06` payload is present.
     */
    public static IgmpBrakeReading decodeIgmpBrakeResponse(String rawResponse) {
        if (rawResponse == null || isErrorOrNoData(rawResponse)) {
            return null;
        }
        List<IsoTpMessage> msgs = parseIsoTpMessages(rawResponse, true);
        if (msgs.isEmpty()) {
            msgs = parseIsoTpMessages(rawResponse, false);
        }
        return decodeIgmpBrakeLamp(msgs);
    }

    public static IgmpBrakeReading decodeIgmpBrakeLamp(List<IsoTpMessage> msgs) {
        if (msgs == null || msgs.isEmpty()) {
            return null;
        }
        for (int i = 0; i < msgs.size(); i++) {
            IsoTpMessage m = msgs.get(i);
            if (m == null || m.payload == null || m.payload.length < 8) {
                continue;
            }
            if (m.canId != -1 && m.canId != IGMP_RX_CAN_ID) {
                continue;
            }
            byte[] u = m.payload;
            if ((u[0] & 0xFF) != 0x62 || (u[1] & 0xFF) != 0xBC || (u[2] & 0xFF) != 0x06) {
                continue;
            }
            // Reject asleep/placeholder IGMP frames filled with 0xAA padding (evDash IGMP guard)
            boolean allAa = true;
            for (int k = 3; k <= 7; k++) {
                if ((u[k] & 0xFF) != 0xAA) {
                    allAa = false;
                    break;
                }
            }
            if (allAa) {
                continue;
            }
            int rawCode = u[7] & 0xFF;
            boolean lit = (rawCode != 0);
            boolean pedalBit = (rawCode & 0x08) != 0;
            return new IgmpBrakeReading(rawCode, lit, pedalBit, bytesToHex(u));
        }
        return null;
    }

    private static final class MultiFrameBuilder {
        final int canId;
        final int expectedLen;
        final byte[] buf;
        int offset;

        MultiFrameBuilder(int canId, int expectedLen) {
            this.canId = canId;
            this.expectedLen = expectedLen;
            this.buf = new byte[expectedLen];
        }

        void append(byte[] src, int srcOff, int len) {
            int copy = Math.min(len, expectedLen - offset);
            if (copy > 0) {
                System.arraycopy(src, srcOff, buf, offset, copy);
                offset += copy;
            }
        }

        boolean isComplete() {
            return offset >= expectedLen;
        }
    }

    public static List<IsoTpMessage> parseIsoTpMessages(String rawResponse, boolean headersExpected) {
        List<IsoTpMessage> out = new ArrayList<IsoTpMessage>();
        if (rawResponse == null || rawResponse.trim().isEmpty()) {
            return out;
        }
        Map<Integer, MultiFrameBuilder> builders = new LinkedHashMap<Integer, MultiFrameBuilder>();
        String[] rawLines = rawResponse.split("[\\r\\n]+");
        for (int idx = 0; idx < rawLines.length; idx++) {
            String line = rawLines[idx].trim();
            if (line.isEmpty()) continue;
            String upper = line.toUpperCase(Locale.US);
            if (upper.startsWith("SEARCHING")
                    || upper.startsWith("AT")
                    || upper.startsWith("22BC06")
                    || upper.equals("OK")
                    || upper.contains("NO DATA")
                    || upper.contains("STOPPED")
                    || upper.contains("ERROR")
                    || upper.equals("?")) {
                continue;
            }

            String compact = stripSpacesUpper(upper);
            if (!headersExpected && compact.length() == 3 && isAllHex(compact) && line.indexOf(':') < 0) {
                int expected = Integer.parseInt(compact, 16);
                if (expected > 0 && expected <= 4095) {
                    builders.put(Integer.valueOf(-1), new MultiFrameBuilder(-1, expected));
                }
                continue;
            }

            int colon = compact.indexOf(':');
            if (colon > 0 && colon <= 2) {
                String hexPart = compact.substring(colon + 1);
                if (!isAllHex(hexPart) || (hexPart.length() & 1) != 0) continue;
                byte[] chunk = hexToBytes(hexPart);
                MultiFrameBuilder b = builders.get(Integer.valueOf(-1));
                if (b != null && chunk != null) {
                    b.append(chunk, 0, chunk.length);
                    if (b.isComplete()) {
                        out.add(new IsoTpMessage(-1, b.buf));
                        builders.remove(Integer.valueOf(-1));
                    }
                }
                continue;
            }

            if (!isAllHex(compact)) {
                continue;
            }

            int canId = -1;
            String frameHex = compact;
            if (headersExpected && compact.length() >= 5 && (compact.length() % 2 == 1)) {
                canId = Integer.parseInt(compact.substring(0, 3), 16);
                frameHex = compact.substring(3);
            } else if (!headersExpected && (compact.length() % 2 == 0)) {
                canId = -1;
                frameHex = compact;
            } else if (compact.length() >= 5 && (compact.length() % 2 == 1)) {
                canId = Integer.parseInt(compact.substring(0, 3), 16);
                frameHex = compact.substring(3);
            } else {
                continue;
            }

            byte[] frame = hexToBytes(frameHex);
            if (frame == null || frame.length == 0) continue;

            if (canId == -1 && !headersExpected) {
                int firstByte = frame[0] & 0xFF;
                if (firstByte >= 0x41 && firstByte <= 0x7F && frame.length <= 8) {
                    if (!isResponsePending(frame, 0, frame.length)) {
                        out.add(new IsoTpMessage(-1, frame));
                    }
                    continue;
                }
            }

            int pciType = (frame[0] & 0xF0) >>> 4;
            if (pciType == 0) {
                int sfLen = frame[0] & 0x0F;
                if (sfLen <= 0 || sfLen > frame.length - 1) continue;
                if (isResponsePending(frame, 1, sfLen)) continue;
                byte[] payload = new byte[sfLen];
                System.arraycopy(frame, 1, payload, 0, sfLen);
                out.add(new IsoTpMessage(canId, payload));
            } else if (pciType == 1) {
                if (frame.length < 3) continue;
                int totalLen = ((frame[0] & 0x0F) << 8) | (frame[1] & 0xFF);
                if (totalLen <= 0 || totalLen > 4095) continue;
                MultiFrameBuilder mfb = new MultiFrameBuilder(canId, totalLen);
                mfb.append(frame, 2, frame.length - 2);
                if (mfb.isComplete()) {
                    out.add(new IsoTpMessage(canId, mfb.buf));
                } else {
                    builders.put(Integer.valueOf(canId), mfb);
                }
            } else if (pciType == 2) {
                MultiFrameBuilder mfb = builders.get(Integer.valueOf(canId));
                if (mfb == null || frame.length < 2) continue;
                mfb.append(frame, 1, frame.length - 1);
                if (mfb.isComplete()) {
                    if (!isResponsePending(mfb.buf, 0, mfb.buf.length)) {
                        out.add(new IsoTpMessage(canId, mfb.buf));
                    }
                    builders.remove(Integer.valueOf(canId));
                }
            }
        }
        return out;
    }

    private static boolean isResponsePending(byte[] buf, int off, int len) {
        return len >= 3 && (buf[off] & 0xFF) == 0x7F && (buf[off + 2] & 0xFF) == 0x78;
    }

    public static String stripSpacesUpper(String s) {
        if (s == null) return "";
        StringBuilder sb = new StringBuilder(s.length());
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
                sb.append(Character.toUpperCase(c));
            }
        }
        return sb.toString();
    }

    public static boolean isAllHex(String s) {
        if (s == null || s.isEmpty()) return false;
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            boolean ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
            if (!ok) return false;
        }
        return true;
    }

    public static byte[] hexToBytes(String hex) {
        if (hex == null || (hex.length() & 1) != 0) return null;
        int len = hex.length() / 2;
        byte[] out = new byte[len];
        for (int i = 0; i < len; i++) {
            int hi = Character.digit(hex.charAt(i * 2), 16);
            int lo = Character.digit(hex.charAt(i * 2 + 1), 16);
            if (hi < 0 || lo < 0) return null;
            out[i] = (byte) ((hi << 4) | lo);
        }
        return out;
    }

    public static String bytesToHex(byte[] data) {
        if (data == null || data.length == 0) return "";
        char[] hexChars = "0123456789ABCDEF".toCharArray();
        char[] out = new char[data.length * 2];
        for (int i = 0; i < data.length; i++) {
            int v = data[i] & 0xFF;
            out[i * 2] = hexChars[v >>> 4];
            out[i * 2 + 1] = hexChars[v & 0x0F];
        }
        return new String(out);
    }
}
