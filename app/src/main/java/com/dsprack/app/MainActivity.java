package com.dsprack.app;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelUuid;
import android.provider.Settings;
import android.view.WindowManager;
import android.webkit.JavascriptInterface;
import android.webkit.PermissionRequest;
import android.webkit.WebChromeClient;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;

import org.json.JSONObject;

import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.Collections;
import java.util.List;
import java.util.UUID;

/**
 * DSP Rack: แอปควบคุม ESP32 DSP ผ่าน Bluetooth Low Energy (Nordic UART Service)
 * หน้าตาใช้ไฟล์ assets/index.html ส่วนบลูทูธทำในแอปเอง ไม่ต้องใช้ Chrome
 */
@SuppressWarnings("deprecation")
@SuppressLint({"MissingPermission", "SetJavaScriptEnabled"})
public class MainActivity extends Activity {
    static final UUID NUS = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    static final UUID RX = UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
    static final UUID TX = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
    static final UUID CCCD = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
    static final int REQ_PERM = 1, REQ_BT_ON = 2, REQ_MIC = 3;

    WebView web;
    final Handler ui = new Handler(Looper.getMainLooper());
    BluetoothAdapter adapter;
    BluetoothLeScanner scanner;
    BluetoothGatt gatt;
    BluetoothGattCharacteristic rxChar;
    final ArrayDeque<byte[]> queue = new ArrayDeque<>();
    final StringBuilder lineBuf = new StringBuilder();
    boolean writing = false, scanning = false, ready = false, wantConnect = false, userDisconnect = false;
    SharedPreferences prefs;
    PermissionRequest pendingMic;   // คำขอใช้ไมค์จากหน้าเว็บ (RTA) ที่รอผู้ใช้กดอนุญาต

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        prefs = getSharedPreferences("dsp", MODE_PRIVATE);
        BluetoothManager bm = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        adapter = bm != null ? bm.getAdapter() : null;

        web = new WebView(this);
        web.setBackgroundColor(0xFF14181E);
        WebSettings s = web.getSettings();
        s.setJavaScriptEnabled(true);
        s.setDomStorageEnabled(true);
        s.setMediaPlaybackRequiresUserGesture(false);
        web.addJavascriptInterface(new Bridge(), "AndroidBle");
        web.setWebChromeClient(new WebChromeClient() {
            // หน้าเว็บขอใช้ไมค์ (getUserMedia) สำหรับ RTA
            @Override
            public void onPermissionRequest(PermissionRequest req) {
                ui.post(() -> {
                    boolean wantsMic = false;
                    for (String r : req.getResources())
                        if (PermissionRequest.RESOURCE_AUDIO_CAPTURE.equals(r)) wantsMic = true;
                    if (!wantsMic) { req.deny(); return; }
                    if (checkSelfPermission(Manifest.permission.RECORD_AUDIO) == PackageManager.PERMISSION_GRANTED) {
                        req.grant(new String[]{PermissionRequest.RESOURCE_AUDIO_CAPTURE});
                    } else {
                        if (pendingMic != null) pendingMic.deny();
                        pendingMic = req;
                        requestPermissions(new String[]{Manifest.permission.RECORD_AUDIO}, REQ_MIC);
                    }
                });
            }

            @Override
            public void onPermissionRequestCanceled(PermissionRequest req) {
                ui.post(() -> { if (pendingMic == req) pendingMic = null; });
            }
        });
        web.setWebViewClient(new WebViewClient() {
            @Override
            public void onPageFinished(WebView v, String url) {
                // เคยต่อแล้ว และอนุญาตสิทธิ์ไว้แล้ว: ต่อให้อัตโนมัติตอนเปิดแอป
                if (prefs.getBoolean("auto", false) && hasPerms() && adapter != null && adapter.isEnabled())
                    startConnect();
            }
        });
        web.loadUrl("file:///android_asset/index.html");
        setContentView(web);
    }

    // ---------------- เรียกจากหน้าเว็บ ----------------
    class Bridge {
        @JavascriptInterface
        public void connect() { ui.post(MainActivity.this::startConnect); }

        @JavascriptInterface
        public void disconnect() {
            ui.post(() -> {
                userDisconnect = true;
                prefs.edit().putBoolean("auto", false).apply();
                stopScan();
                if (gatt != null) gatt.disconnect(); else js("bleOnDisconnected(true)");
            });
        }

        @JavascriptInterface
        public void write(String s) {
            byte[] all = s.getBytes(StandardCharsets.UTF_8);
            ui.post(() -> {
                for (int i = 0; i < all.length; i += 20) {
                    int n = Math.min(20, all.length - i);
                    byte[] part = new byte[n];
                    System.arraycopy(all, i, part, 0, n);
                    queue.add(part);
                }
                pump();
            });
        }
    }

    void js(String code) { ui.post(() -> web.evaluateJavascript(code, null)); }

    void status(String t, boolean bad) { js("bleOnStatus(" + JSONObject.quote(t) + "," + bad + ")"); }

    // ---------------- สิทธิ์ / เปิดบลูทูธ ----------------
    String[] neededPerms() {
        if (Build.VERSION.SDK_INT >= 31)
            return new String[]{Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT};
        return new String[]{Manifest.permission.ACCESS_FINE_LOCATION};
    }

    boolean hasPerms() {
        for (String p : neededPerms())
            if (checkSelfPermission(p) != PackageManager.PERMISSION_GRANTED) return false;
        return true;
    }

    void startConnect() {
        if (adapter == null) { status("มือถือนี้ไม่มีบลูทูธ", true); return; }
        if (!hasPerms()) { wantConnect = true; requestPermissions(neededPerms(), REQ_PERM); return; }
        if (!adapter.isEnabled()) {
            wantConnect = true;
            startActivityForResult(new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE), REQ_BT_ON);
            return;
        }
        if (Build.VERSION.SDK_INT < 31 && Build.VERSION.SDK_INT >= 28) {
            LocationManager lm = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
            if (lm != null && !lm.isLocationEnabled()) {
                status("Android รุ่นนี้ต้องเปิดตำแหน่ง (Location) ก่อนค้นหาบลูทูธ", true);
                startActivity(new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS));
                return;
            }
        }
        wantConnect = false;
        if (ready || scanning) return;
        startScan();
    }

    @Override
    public void onRequestPermissionsResult(int code, String[] perms, int[] res) {
        if (code == REQ_MIC) {
            if (pendingMic == null) return;
            if (res.length > 0 && res[0] == PackageManager.PERMISSION_GRANTED)
                pendingMic.grant(new String[]{PermissionRequest.RESOURCE_AUDIO_CAPTURE});
            else
                pendingMic.deny();
            pendingMic = null;
            return;
        }
        if (code != REQ_PERM) return;
        if (hasPerms()) { if (wantConnect) startConnect(); }
        else status("ต้องอนุญาตบลูทูธ/อุปกรณ์ใกล้เคียง ก่อนจึงจะเชื่อมต่อได้", true);
    }

    @Override
    protected void onActivityResult(int code, int result, Intent data) {
        if (code == REQ_BT_ON) {
            if (adapter.isEnabled() && wantConnect) startConnect();
            else status("ต้องเปิดบลูทูธก่อน", true);
        }
    }

    // ---------------- ค้นหา ----------------
    final ScanCallback scanCb = new ScanCallback() {
        @Override
        public void onScanResult(int type, ScanResult r) {
            if (!scanning) return;
            stopScan();
            connectDevice(r.getDevice());
        }

        @Override
        public void onScanFailed(int err) {
            scanning = false;
            status("ค้นหาไม่สำเร็จ (" + err + ") ลองปิด-เปิดบลูทูธ", true);
        }
    };

    final Runnable scanTimeout = () -> {
        if (!scanning) return;
        stopScan();
        status("ไม่พบ ESP32-DSP · เช็กว่าบอร์ดเปิดอยู่และใช้โค้ด USE_BLE 1", true);
        js("bleOnDisconnected(true)");
    };

    void startScan() {
        scanner = adapter.getBluetoothLeScanner();
        if (scanner == null) { status("เปิดบลูทูธก่อน", true); return; }
        List<ScanFilter> f = Collections.singletonList(
                new ScanFilter.Builder().setServiceUuid(new ParcelUuid(NUS)).build());
        ScanSettings st = new ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build();
        scanning = true;
        scanner.startScan(f, st, scanCb);
        status("กำลังค้นหา ESP32-DSP…", false);
        ui.postDelayed(scanTimeout, 12000);
    }

    void stopScan() {
        ui.removeCallbacks(scanTimeout);
        if (scanning && scanner != null) {
            try { scanner.stopScan(scanCb); } catch (Exception ignored) { }
        }
        scanning = false;
    }

    // ---------------- เชื่อมต่อ ----------------
    void connectDevice(BluetoothDevice d) {
        status("กำลังเชื่อมต่อ…", false);
        if (gatt != null) { gatt.close(); gatt = null; }
        userDisconnect = false;
        // TRANSPORT_LE สำคัญ: ESP32 เป็นทั้งลำโพงบลูทูธ (Classic) และ BLE ในตัวเดียว
        gatt = d.connectGatt(this, false, gattCb, BluetoothDevice.TRANSPORT_LE);
    }

    final BluetoothGattCallback gattCb = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt g, int st, int state) {
            if (state == BluetoothProfile.STATE_CONNECTED) {
                ui.post(g::discoverServices);
            } else if (state == BluetoothProfile.STATE_DISCONNECTED) {
                ui.post(() -> {
                    ready = false; rxChar = null; writing = false; queue.clear();
                    synchronized (lineBuf) { lineBuf.setLength(0); }
                    g.close();
                    if (gatt == g) gatt = null;
                    js("bleOnDisconnected(" + userDisconnect + ")");
                });
            }
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt g, int st) {
            ui.post(() -> {
                BluetoothGattService s = g.getService(NUS);
                if (s == null) { status("อุปกรณ์นี้ไม่ใช่ ESP32-DSP", true); g.disconnect(); return; }
                rxChar = s.getCharacteristic(RX);
                BluetoothGattCharacteristic tx = s.getCharacteristic(TX);
                if (rxChar == null || tx == null) { status("อ่านบริการไม่สำเร็จ", true); g.disconnect(); return; }
                g.setCharacteristicNotification(tx, true);
                BluetoothGattDescriptor d = tx.getDescriptor(CCCD);
                if (d != null) {
                    d.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                    g.writeDescriptor(d);
                } else {
                    onReady(g);
                }
            });
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt g, BluetoothGattDescriptor d, int st) {
            ui.post(() -> onReady(g));
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt g, BluetoothGattCharacteristic c, int st) {
            ui.post(() -> { writing = false; pump(); });
        }

        // Android 12 และต่ำกว่า
        @Override
        public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c) {
            onData(c.getValue());
        }

        // Android 13 ขึ้นไป
        @Override
        public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c, byte[] v) {
            onData(v);
        }
    };

    void onReady(BluetoothGatt g) {
        if (ready) return;
        ready = true;
        prefs.edit().putBoolean("auto", true).apply();
        String name = g.getDevice().getName();
        js("bleOnConnected(" + JSONObject.quote(name == null ? "ESP32-DSP" : name) + ")");
        pump();
    }

    void onData(byte[] v) {
        if (v == null) return;
        String s = new String(v, StandardCharsets.ISO_8859_1);
        synchronized (lineBuf) {
            lineBuf.append(s);
            int i;
            while ((i = lineBuf.indexOf("\n")) >= 0) {
                String line = lineBuf.substring(0, i);
                lineBuf.delete(0, i + 1);
                if (!line.isEmpty()) js("bleOnLine(" + JSONObject.quote(line) + ")");
            }
            if (lineBuf.length() > 8000) lineBuf.setLength(0);
        }
    }

    void pump() {
        if (writing || !ready || rxChar == null || gatt == null) return;
        byte[] b = queue.poll();
        if (b == null) return;
        rxChar.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
        rxChar.setValue(b);
        writing = gatt.writeCharacteristic(rxChar);
        if (!writing) {
            queue.addFirst(b);
            ui.postDelayed(this::pump, 30);
        }
    }

    @Override
    protected void onDestroy() {
        stopScan();
        if (gatt != null) { gatt.disconnect(); gatt.close(); gatt = null; }
        super.onDestroy();
    }
}
