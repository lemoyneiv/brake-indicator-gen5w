package dev.lemoyne.brakeindicator;

import android.net.LocalSocket;
import android.net.LocalSocketAddress;

import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.SocketTimeoutException;
import java.nio.charset.Charset;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Resilient {@code @hmgapid} {@code LFD1} vehicle-reading client.
 *
 * On updated HMG-API ({@code Hub.closeAccepted}), {@code Hub} closes its accepted socket
 * immediately and {@code hmgapid} sends both {@code 'O'} (readings) and {@code 'X'} (exit 0)
 * in {@code < 2 ms}, enabling true 20 Hz polling.
 *
 * On older HMG-API installs affected by Android 4.4 KitKat AOSP Bug 36917
 * ({@code LocalServerSocket.accept()} leaving {@code fdCreatedInternally == false}),
 * {@code hmgapid} delivers the complete {@code 'O'} readings frame in {@code < 2 ms}
 * and then stalls 3,000 ms waiting for {@code EOF} before returning exit code 1.
 * {@link #readVehicle} recovers the already-delivered {@code 'O'} readings after a short
 * post-data grace timeout ({@code 90 ms}) and signals {@code legacyHubStall = true} so the
 * caller can pace requests gently without starving {@code hmgapid}'s worker pool.
 */
public final class HmgVehiclePoller {

    private static final Charset UTF8 = Charset.forName("UTF-8");
    private static final String DEFAULT_SOCKET = "hmgapid";

    public interface TimeoutSetter {
        void setReadTimeoutMs(int timeoutMs) throws IOException;
    }

    public static final class PollResult {
        public final Map<String, String> readings;
        public final boolean ok;
        public final boolean legacyHubStall;

        public PollResult(Map<String, String> readings, boolean ok, boolean legacyHubStall) {
            this.readings = readings == null ? Collections.<String, String>emptyMap() : readings;
            this.ok = ok;
            this.legacyHubStall = legacyHubStall;
        }
    }

    private HmgVehiclePoller() {}

    public static PollResult readVehicle(String[] names, int initialTimeoutMs, int postOutputExitTimeoutMs) {
        return readVehicleFromSocket(DEFAULT_SOCKET, LocalSocketAddress.Namespace.ABSTRACT,
                names, initialTimeoutMs, postOutputExitTimeoutMs);
    }

    public static PollResult readVehicleFromSocket(
            String socketName,
            LocalSocketAddress.Namespace namespace,
            String[] names,
            int initialTimeoutMs,
            int postOutputExitTimeoutMs) {
        if (names == null || names.length == 0) {
            return new PollResult(Collections.<String, String>emptyMap(), false, false);
        }
        final LocalSocket s = new LocalSocket();
        try {
            s.connect(new LocalSocketAddress(socketName, namespace));
            s.setSoTimeout(Math.max(100, initialTimeoutMs));
            return readVehicleOverStreams(
                    s.getInputStream(),
                    s.getOutputStream(),
                    new TimeoutSetter() {
                        @Override
                        public void setReadTimeoutMs(int timeoutMs) throws IOException {
                            s.setSoTimeout(timeoutMs);
                        }
                    },
                    names,
                    postOutputExitTimeoutMs);
        } catch (IOException e) {
            return new PollResult(Collections.<String, String>emptyMap(), false, false);
        } finally {
            try { s.close(); } catch (Throwable ignored) {}
        }
    }

    public static PollResult readVehicleOverStreams(
            InputStream rawIn,
            OutputStream rawOut,
            TimeoutSetter timeoutSetter,
            String[] names,
            int postOutputExitTimeoutMs) {
        if (names == null || names.length == 0 || rawIn == null || rawOut == null) {
            return new PollResult(Collections.<String, String>emptyMap(), false, false);
        }
        ByteArrayOutputStream text = new ByteArrayOutputStream(256);
        boolean gotOutputFrame = false;
        boolean legacyStall = false;
        int exitCode = -1;

        try {
            byte[] req = buildLfd1VehicleRequest(names);
            rawOut.write(req);
            rawOut.flush();

            DataInputStream in = new DataInputStream(rawIn);
            while (true) {
                int type;
                try {
                    type = in.read();
                } catch (SocketTimeoutException ste) {
                    if (gotOutputFrame) {
                        legacyStall = true;
                        break;
                    }
                    throw ste;
                }
                if (type < 0) {
                    break;
                }
                byte[] lenBuf = new byte[4];
                in.readFully(lenBuf);
                int len = (lenBuf[0] & 0xff)
                        | ((lenBuf[1] & 0xff) << 8)
                        | ((lenBuf[2] & 0xff) << 16)
                        | ((lenBuf[3] & 0xff) << 24);
                if (len < 0 || len > (1 << 20)) {
                    throw new IOException("bad hmgapid frame len: " + len);
                }
                byte[] data = new byte[len];
                in.readFully(data);
                if (type == 'O') {
                    text.write(data);
                    if (text.size() > 0 && hasNewlineTerminatedReading(text.toByteArray())) {
                        gotOutputFrame = true;
                        if (timeoutSetter != null) {
                            timeoutSetter.setReadTimeoutMs(Math.max(25, postOutputExitTimeoutMs));
                        }
                    }
                } else if (type == 'X' && len == 4) {
                    exitCode = (data[0] & 0xff)
                            | ((data[1] & 0xff) << 8)
                            | ((data[2] & 0xff) << 16)
                            | ((data[3] & 0xff) << 24);
                    if (exitCode == 1 && gotOutputFrame) {
                        legacyStall = true;
                    }
                    break;
                }
            }
        } catch (IOException e) {
            if (!gotOutputFrame) {
                return new PollResult(Collections.<String, String>emptyMap(), false, false);
            }
            legacyStall = true;
        }

        String raw = new String(text.toByteArray(), UTF8);
        Map<String, String> parsed = parseReadingsText(raw);
        boolean ok = !parsed.isEmpty() && (exitCode == 0 || legacyStall);
        return new PollResult(parsed, ok, legacyStall);
    }

    public static byte[] buildLfd1VehicleRequest(String[] names) {
        ByteArrayOutputStream req = new ByteArrayOutputStream(192);
        req.write('L');
        req.write('F');
        req.write('D');
        req.write('1');
        writeU32(req, names.length + 1);
        writeArg(req, "vehicle");
        for (int i = 0; i < names.length; i++) {
            writeArg(req, names[i]);
        }
        return req.toByteArray();
    }

    public static Map<String, String> parseReadingsText(String raw) {
        if (raw == null || raw.isEmpty()) {
            return Collections.emptyMap();
        }
        Map<String, String> out = new LinkedHashMap<String, String>();
        String[] lines = raw.split("\n");
        for (int i = 0; i < lines.length; i++) {
            String l = lines[i].trim();
            int sp = l.indexOf(' ');
            if (sp > 0) {
                String key = l.substring(0, sp);
                String val = l.substring(sp + 1).trim();
                if (!key.contains(":") && !"the".equals(key) && !"not".equals(key) && !"bad".equals(key)) {
                    out.put(key, val);
                }
            }
        }
        return out;
    }

    private static boolean hasNewlineTerminatedReading(byte[] bytes) {
        if (bytes == null || bytes.length < 3) return false;
        return bytes[bytes.length - 1] == (byte) '\n';
    }

    private static void writeArg(ByteArrayOutputStream out, String arg) {
        byte[] b = (arg == null ? "" : arg).getBytes(UTF8);
        writeU32(out, b.length);
        out.write(b, 0, b.length);
    }

    private static void writeU32(ByteArrayOutputStream o, int v) {
        o.write(v & 0xff);
        o.write((v >> 8) & 0xff);
        o.write((v >> 16) & 0xff);
        o.write((v >> 24) & 0xff);
    }
}
