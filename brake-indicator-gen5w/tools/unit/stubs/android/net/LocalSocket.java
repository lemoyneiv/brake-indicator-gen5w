package android.net;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

/** Minimal host-JVM compile stub for unit tests (never packaged in APK). */
public class LocalSocket {
    public LocalSocket() {}

    public void connect(LocalSocketAddress endpoint) throws IOException {
        throw new IOException("Host JVM stub");
    }

    public void setSoTimeout(int n) throws IOException {}

    public InputStream getInputStream() throws IOException {
        throw new IOException("Host JVM stub");
    }

    public OutputStream getOutputStream() throws IOException {
        throw new IOException("Host JVM stub");
    }

    public void shutdownInput() throws IOException {}

    public void close() throws IOException {}
}
