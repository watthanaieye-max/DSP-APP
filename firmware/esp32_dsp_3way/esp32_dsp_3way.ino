/*
 * DSP Rack v2 — โมโน 3 ทาง (ซับ / กลาง / แหลม) + โหมดสเตอริโอ 2.1
 *
 * บอร์ด : ESP32-DevKitC WROVER (PSRAM 8MB)
 *         Arduino IDE → Board "ESP32 Wrover Module", Partition Scheme "Huge APP (3MB No OTA)"
 * Core  : esp32 (Espressif) 3.3.x เท่านั้น (ทดสอบไวยากรณ์กับ 3.3.13) — core 4.x ไม่มี BLEDevice.h
 * ไลบรารี: ESP32-A2DP (pschatzmann) v1.8.11
 *
 * เสียงเข้า : Bluetooth A2DP ชื่อ "ESP32-DSP" (44.1 kHz)
 * ควบคุม   : BLE Nordic UART Service (แอป DSP Rack)
 * DAC1     : PCM5102 BCK=26 LCK=25 DIN=22  ซ้าย=กลาง ขวา=แหลม  (สเตอริโอ: ซ้าย/ขวา)
 * DAC2     : PCM5102 BCK=27 LCK=32 DIN=33  ซับทั้งสองช่อง
 *
 * โปรโตคอล (บรรทัดละคำสั่ง จบด้วย \n, ส่งชิ้นละ 20 ไบต์):
 *   get                      → J{json}
 *   save                     → Ksave        (บันทึกค่าปัจจุบัน ใช้ตอนเปิดเครื่อง)
 *   reset                    → J{json}      (กลับค่าเริ่มต้น ไม่ลบพรีเซ็ต)
 *   k=v&k=v                  ตั้งค่า
 *   preset_load=n            → J{json}      (n = 0..4)
 *   preset_save=n&name=...   → Kpreset + J  (name เข้ารหัส URL)
 *   lock=1234 / unlock=1234  → J{json}
 *   ข้อผิดพลาด: Elocked, Ecode, Epreset
 *   มิเตอร์ทุก 150 ms: M{"in","sub","mid","hi","lim_sub","lim_mid","lim_hi","cpu"}
 */

#include <Arduino.h>
#if ESP_ARDUINO_VERSION_MAJOR >= 4
  // core 4.x เปลี่ยนไลบรารี BLE ใหม่ทั้งหมด (ไม่มี BLEDevice.h แล้ว)
  #error "ใช้บอร์ด esp32 (Espressif) เวอร์ชัน 3.3.x: Tools > Board > Boards Manager > esp32 > เลือก 3.3.13 > Install"
#endif
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/stream_buffer.h>
#include "BluetoothA2DPSink.h"

// ===================== ตัวเลือก =====================
#define USE_DAC2      1   // 0 = ทดสอบด้วย DAC1 ตัวเดียว (ยังไม่มี DAC2): ซับถูกรวมเข้า DAC1
#define BT_NAME       "ESP32-DSP"

#define DAC1_BCK 26
#define DAC1_LCK 25
#define DAC1_DIN 22
#define DAC2_BCK 27
#define DAC2_LCK 32
#define DAC2_DIN 33

#define FS        44100.0f
#define BLOCK     128                 // เฟรมต่อบล็อก (~2.9 ms)
#define SB_SIZE   (16 * 1024)         // บัฟเฟอร์รับเสียงจาก A2DP (ไบต์)
#define PREBUF    (4 * 1024)          // เริ่มเล่นเมื่อมีข้อมูลเท่านี้
#define MAX_DLY   20.0f               // ms
#define DLY_LEN   1024                // > 20 ms * 44.1 + ระยะ interpolation

#define NB  6     // แบนด์อีคิวต่อชุด
#define NCH 3     // 0=ซับ 1=กลาง(ซ้าย) 2=แหลม(ขวา)
#define NPRE 5
#define PARAM_VER 3

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #include <driver/i2s_std.h>
  #define NEW_I2S 1
#else
  #include <driver/i2s.h>
  #define NEW_I2S 0
#endif

// ===================== พารามิเตอร์ =====================
struct Band { float f, g, q; uint8_t t, on; };      // t: 0=peak 1=low shelf 2=high shelf
struct Chan {
  Band eq[NB]; uint8_t eqOn;
  float gain, dly;           // dB, ms
  uint8_t inv, mute, solo;
  float limT, limA, limR;    // dBFS, ms, ms
  uint8_t limOn;
};
struct Params {
  uint16_t ver;
  uint8_t mode;              // 0 = โมโน 3 ทาง, 1 = สเตอริโอ 2.1
  float vol, inG, bal;
  Band ieq[NB]; uint8_t ieqOn;
  uint8_t ssOn, ssSl; float ssF;     // subsonic: ssSl 0=12 dB 1=24 dB
  float x1, x2;                      // ซับ/กลาง, กลาง/แหลม
  uint8_t ft[4], fs[4];              // ตัวกรอง 0=ซับ LP 1=กลาง HP 2=กลาง LP 3=แหลม HP · ft 0=BW 1=LR · fs 0=12 1=24 2=48
  Chan ch[NCH];
  char name[33];
};

Params P;                    // ค่าที่แก้จากคำสั่ง (loop)
portMUX_TYPE pmux = portMUX_INITIALIZER_UNLOCKED;
volatile bool pDirty = true;
int8_t activePreset = -1;
char presetNames[NPRE][33];
uint16_t lockCode = 0; uint8_t locked = 0;   // lockCode = รหัส+1, 0 = ยังไม่ตั้งรหัส
Preferences prefs;

void defaults(Params &p) {
  memset(&p, 0, sizeof(p));
  p.ver = PARAM_VER; p.mode = 0; p.vol = -20; p.inG = 0; p.bal = 0;
  const float fi[NB] = {60, 150, 400, 1000, 3000, 8000};
  for (int b = 0; b < NB; b++) p.ieq[b] = {fi[b], 0, 1.0f, 0, 1};
  p.ieqOn = 1;
  p.ssOn = 1; p.ssSl = 1; p.ssF = 25;
  p.x1 = 80; p.x2 = 3000;
  for (int i = 0; i < 4; i++) { p.ft[i] = 1; p.fs[i] = 1; }   // LR 24 dB
  const float fc[NCH][NB] = {{25, 40, 63, 100, 160, 250},
                             {150, 300, 600, 1200, 2500, 5000},
                             {2500, 4000, 6300, 10000, 14000, 18000}};
  for (int c = 0; c < NCH; c++) {
    Chan &h = p.ch[c];
    for (int b = 0; b < NB; b++) h.eq[b] = {fc[c][b], 0, 1.4f, 0, 1};
    h.eqOn = 1; h.gain = 0; h.dly = 0; h.inv = h.mute = h.solo = 0;
    h.limT = -1; h.limA = 5; h.limR = 200; h.limOn = 1;
  }
  strcpy(p.name, "");
}

// ----- ตารางชื่อคีย์ ↔ ตัวแปร (ใช้ทั้ง k=v และ J{json}) -----
struct Key { char k[8]; void *p; uint8_t isF; float mn, mx; };
Key keys[200]; int nKeys = 0;
void addF(const char *k, float *p, float mn, float mx) { Key &e = keys[nKeys++]; strncpy(e.k, k, 7); e.k[7] = 0; e.p = p; e.isF = 1; e.mn = mn; e.mx = mx; }
void addB(const char *k, uint8_t *p, float mx) { Key &e = keys[nKeys++]; strncpy(e.k, k, 7); e.k[7] = 0; e.p = p; e.isF = 0; e.mn = 0; e.mx = mx; }
void addBand(const char *pre, Band *b, int n) {
  char k[8];
  for (int i = 0; i < NB; i++) {
    snprintf(k, 8, "%st%d", pre, i); addB(k, &b[i].t, 2);
    snprintf(k, 8, "%sf%d", pre, i); addF(k, &b[i].f, 15, 20000);
    snprintf(k, 8, "%sg%d", pre, i); addF(k, &b[i].g, -18, 12);
    snprintf(k, 8, "%sq%d", pre, i); addF(k, &b[i].q, 0.3f, 10);
    snprintf(k, 8, "%se%d", pre, i); addB(k, &b[i].on, 1);
  }
}
void buildKeys() {
  addB("mode", &P.mode, 1);
  addF("vol", &P.vol, -60, 0);
  addF("ing", &P.inG, -30, 12);
  addF("bal", &P.bal, -100, 100);
  addB("ieq", &P.ieqOn, 1);
  addBand("i", P.ieq, NB);
  addB("ss", &P.ssOn, 1); addB("sss", &P.ssSl, 1); addF("ssf", &P.ssF, 15, 60);
  addF("x1", &P.x1, 40, 300); addF("x2", &P.x2, 1000, 8000);
  char k[8];
  for (int i = 0; i < 4; i++) { snprintf(k, 8, "ft%d", i); addB(k, &P.ft[i], 1); snprintf(k, 8, "fs%d", i); addB(k, &P.fs[i], 2); }
  for (int c = 0; c < NCH; c++) {
    Chan &h = P.ch[c]; char pre[4]; snprintf(pre, 4, "c%d", c);
    addBand(pre, h.eq, NB);
    snprintf(k, 8, "c%deq", c); addB(k, &h.eqOn, 1);
    snprintf(k, 8, "c%dgn", c); addF(k, &h.gain, -30, 6);
    snprintf(k, 8, "c%ddl", c); addF(k, &h.dly, 0, MAX_DLY);
    snprintf(k, 8, "c%div", c); addB(k, &h.inv, 1);
    snprintf(k, 8, "c%dmu", c); addB(k, &h.mute, 1);
    snprintf(k, 8, "c%dso", c); addB(k, &h.solo, 1);
    snprintf(k, 8, "c%dlt", c); addF(k, &h.limT, -30, 0);
    snprintf(k, 8, "c%dla", c); addF(k, &h.limA, 0.1f, 50);
    snprintf(k, 8, "c%dlr", c); addF(k, &h.limR, 10, 1000);
    snprintf(k, 8, "c%dlo", c); addB(k, &h.limOn, 1);
  }
}

// ===================== DSP: biquad =====================
enum { T_PEAK = 0, T_LS = 1, T_HS = 2, T_LP = 3, T_HP = 4 };
// DF2T float, ค่าสัมประสิทธิ์ค่อยๆ เลื่อนเข้าหาเป้าหมายทุกบล็อก (กันเสียงป๊อก)
struct BQ {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;      // ใช้งานจริง
  float t0 = 1, t1 = 0, t2 = 0, u1 = 0, u2 = 0;      // เป้าหมาย
  float z1 = 0, z2 = 0;
  bool idle = true;                                   // ผ่านตรง ไม่ต้องคำนวณ
  void set(float B0, float B1, float B2, float A1, float A2) { t0 = B0; t1 = B1; t2 = B2; u1 = A1; u2 = A2; idle = false; }
  void bypass() { set(1, 0, 0, 0, 0); }
  void glide(float k) {
    if (idle) return;
    b0 += (t0 - b0) * k; b1 += (t1 - b1) * k; b2 += (t2 - b2) * k; a1 += (u1 - a1) * k; a2 += (u2 - a2) * k;
    if (t0 == 1 && t1 == 0 && t2 == 0 && u1 == 0 && u2 == 0 &&
        fabsf(b0 - 1) + fabsf(b1) + fabsf(b2) + fabsf(a1) + fabsf(a2) < 1e-6f) { b0 = 1; b1 = b2 = a1 = a2 = 0; idle = true; }
  }
  void snap() { b0 = t0; b1 = t1; b2 = t2; a1 = u1; a2 = u2; }
  void clear() { z1 = z2 = 0; }
  inline float run(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
  void runBlock(float *x, int n) { if (idle) return; for (int i = 0; i < n; i++) x[i] = run(x[i]); }
  // ออกแบบ biquad (RBJ cookbook) แล้วตั้งเป็นค่าเป้าหมาย
  void design(int t, float f, float g, float Q) {
    f = fminf(f, FS * 0.45f);
    float A = powf(10, g / 40), w = 2 * PI * f / FS, cw = cosf(w), sw = sinf(w), al = sw / (2 * Q);
    float b0, b1, b2, a0, a1, a2;
    switch (t) {
      case T_PEAK: b0 = 1 + al * A; b1 = -2 * cw; b2 = 1 - al * A; a0 = 1 + al / A; a1 = -2 * cw; a2 = 1 - al / A; break;
      case T_LS: { float s = 2 * sqrtf(A) * al;
        b0 = A * ((A + 1) - (A - 1) * cw + s); b1 = 2 * A * ((A - 1) - (A + 1) * cw); b2 = A * ((A + 1) - (A - 1) * cw - s);
        a0 = (A + 1) + (A - 1) * cw + s; a1 = -2 * ((A - 1) + (A + 1) * cw); a2 = (A + 1) + (A - 1) * cw - s; } break;
      case T_HS: { float s = 2 * sqrtf(A) * al;
        b0 = A * ((A + 1) + (A - 1) * cw + s); b1 = -2 * A * ((A - 1) + (A + 1) * cw); b2 = A * ((A + 1) + (A - 1) * cw - s);
        a0 = (A + 1) - (A - 1) * cw + s; a1 = 2 * ((A - 1) - (A + 1) * cw); a2 = (A + 1) - (A - 1) * cw - s; } break;
      case T_LP: b0 = (1 - cw) / 2; b1 = 1 - cw; b2 = (1 - cw) / 2; a0 = 1 + al; a1 = -2 * cw; a2 = 1 - al; break;
      default:   b0 = (1 + cw) / 2; b1 = -(1 + cw); b2 = (1 + cw) / 2; a0 = 1 + al; a1 = -2 * cw; a2 = 1 - al; break;
    }
    set(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0);
  }
};


// ค่า Q ของแต่ละชั้น: [ชนิด 0=BW 1=LR][ความชัน 0=12 1=24 2=48]
const float XQ[2][3][4] = {
  {{0.70711f, 0, 0, 0}, {0.54120f, 1.30656f, 0, 0}, {0.50980f, 0.60134f, 0.89998f, 2.56292f}},
  {{0.5f, 0, 0, 0},     {0.70711f, 0.70711f, 0, 0}, {0.54120f, 1.30656f, 0.54120f, 1.30656f}}};
const int XN[3] = {1, 2, 4};

struct XF {               // ตัวกรองครอสโอเวอร์ สูงสุด 4 ชั้น (48 dB/oct)
  BQ s[4];
  void design(int lpHp, float f, int type, int slope) {
    for (int i = 0; i < 4; i++) {
      if (i < XN[slope]) s[i].design(lpHp, f, 0, XQ[type][slope][i]); else s[i].bypass();
    }
  }
  void off() { for (auto &q : s) q.bypass(); }
  void glide(float k) { for (auto &q : s) q.glide(k); }
  void snap() { for (auto &q : s) q.snap(); }
  void clear() { for (auto &q : s) q.clear(); }
  void runBlock(float *x, int n) { for (auto &q : s) q.runBlock(x, n); }
};

struct EQ6 {
  BQ b[NB];
  void design(const Band *bd, bool on) {
    for (int i = 0; i < NB; i++) {
      if (on && bd[i].on && (fabsf(bd[i].g) > 0.01f)) b[i].design(bd[i].t, bd[i].f, bd[i].g, bd[i].q); else b[i].bypass();
    }
  }
  void glide(float k) { for (auto &q : b) q.glide(k); }
  void snap() { for (auto &q : b) q.snap(); }
  void clear() { for (auto &q : b) q.clear(); }
  void runBlock(float *x, int n) { for (auto &q : b) q.runBlock(x, n); }
};

// ----- สถานะ DSP (เป็นของ task DSP เท่านั้น) -----
struct InPath { EQ6 eq; XF ss; };
struct OutCh {
  XF hp, lp; EQ6 eq;
  float g = 0, gT = 0;                 // เกนเชิงเส้น (รวม vol, เฟส, ปิดเสียง)
  float dl = 0, dlT = 0;               // ดีเลย์ (sample)
  float buf[DLY_LEN] = {0}; int w = 0;
  float limTh = 1, limA = 0, limR = 0, lg = 1; bool limOn = true, mute = false;
  float peak = 0, grMax = 0;
};
InPath inp[2];
OutCh oc[NCH];
Params D;                              // สำเนาที่ DSP ใช้อยู่
float inGain = 1, inGainT = 1, fade = 1, fadeT = 1;
float inPeak = 0;
volatile float cpuLoad = 0;
bool pendingMode = false;

static inline float db2lin(float d) { return powf(10, d / 20); }

void applyParams(bool snap) {
  // อินพุต
  inGainT = db2lin(D.inG);
  for (int k = 0; k < 2; k++) {
    inp[k].eq.design(D.ieq, D.ieqOn);
    if (D.ssOn) inp[k].ss.design(T_HP, D.ssF, 0, D.ssSl ? 1 : 0); else inp[k].ss.off();
  }
  bool stereo = D.mode == 1;
  // ครอสโอเวอร์
  oc[0].hp.off();
  oc[0].lp.design(T_LP, D.x1, D.ft[0], D.fs[0]);
  oc[1].hp.design(T_HP, D.x1, D.ft[1], D.fs[1]);
  if (stereo) oc[1].lp.off(); else oc[1].lp.design(T_LP, D.x2, D.ft[2], D.fs[2]);
  if (stereo) oc[2].hp.design(T_HP, D.x1, D.ft[1], D.fs[1]); else oc[2].hp.design(T_HP, D.x2, D.ft[3], D.fs[3]);
  oc[2].lp.off();
  // ช่องออก
  bool anySolo = false;
  for (int c = 0; c < NCH; c++) anySolo |= D.ch[c].solo;
  float vol = db2lin(D.vol);
  for (int c = 0; c < NCH; c++) {
    Chan &h = D.ch[c]; OutCh &o = oc[c];
    o.eq.design(h.eq, h.eqOn);
    float g = db2lin(h.gain) * vol * (h.inv ? -1 : 1);
    if (stereo && c == 1 && D.bal > 0) g *= 1 - D.bal / 100;
    if (stereo && c == 2 && D.bal < 0) g *= 1 + D.bal / 100;
    o.gT = g;
    o.dlT = constrain(h.dly, 0, MAX_DLY) * FS / 1000;
    o.limTh = db2lin(h.limT); o.limOn = h.limOn;
    o.limA = 1 - expf(-1 / (fmaxf(h.limA, 0.1f) * FS / 1000));
    o.limR = 1 - expf(-1 / (fmaxf(h.limR, 10) * FS / 1000));
    o.mute = h.mute || (anySolo && !h.solo);
  }
  if (snap) {
    for (auto &i : inp) { i.eq.snap(); i.ss.snap(); }
    for (auto &o : oc) { o.hp.snap(); o.lp.snap(); o.eq.snap(); o.dl = o.dlT; }
  }
}

// ===================== I2S =====================
#if NEW_I2S
i2s_chan_handle_t tx1 = nullptr, tx2 = nullptr;
void i2sOpen(i2s_port_t port, int bck, int lck, int din, i2s_chan_handle_t *h) {
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(port, I2S_ROLE_MASTER);
  cc.dma_desc_num = 6; cc.dma_frame_num = 256; cc.auto_clear = true;
  ESP_ERROR_CHECK(i2s_new_channel(&cc, h, nullptr));
  i2s_std_config_t sc;
  memset(&sc, 0, sizeof(sc));
  sc.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100);
  sc.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  sc.gpio_cfg.mclk = I2S_GPIO_UNUSED; sc.gpio_cfg.bclk = (gpio_num_t)bck; sc.gpio_cfg.ws = (gpio_num_t)lck;
  sc.gpio_cfg.dout = (gpio_num_t)din; sc.gpio_cfg.din = I2S_GPIO_UNUSED;
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(*h, &sc));
  ESP_ERROR_CHECK(i2s_channel_enable(*h));
}
void i2sWrite(int n, const int32_t *buf, size_t bytes) {
  size_t w; i2s_channel_write(n == 1 ? tx1 : tx2, buf, bytes, &w, portMAX_DELAY);
}
#else
void i2sOpen(i2s_port_t port, int bck, int lck, int din) {
  i2s_config_t c = {};
  c.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  c.sample_rate = 44100; c.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  c.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT; c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  c.intr_alloc_flags = 0; c.dma_buf_count = 6; c.dma_buf_len = 256; c.use_apll = false; c.tx_desc_auto_clear = true;
  i2s_driver_install(port, &c, 0, nullptr);
  i2s_pin_config_t p = {};
  p.mck_io_num = I2S_PIN_NO_CHANGE; p.bck_io_num = bck; p.ws_io_num = lck; p.data_out_num = din; p.data_in_num = I2S_PIN_NO_CHANGE;
  i2s_set_pin(port, &p);
}
void i2sWrite(int n, const int32_t *buf, size_t bytes) {
  size_t w; i2s_write(n == 1 ? I2S_NUM_0 : I2S_NUM_1, buf, bytes, &w, portMAX_DELAY);
}
#endif

// ===================== รับเสียงจาก A2DP =====================
BluetoothA2DPSink a2dp;
StreamBufferHandle_t sb;
void onAudio(const uint8_t *data, uint32_t len) {
  // ถ้าบัฟเฟอร์ล้น (นาฬิกาสองฝั่งไม่ตรงกัน) ทิ้งข้อมูลเก่าบางส่วน
  if (xStreamBufferSpacesAvailable(sb) < len) {
    uint8_t junk[256]; size_t drop = len;
    while (drop > 0) { size_t n = xStreamBufferReceive(sb, junk, drop > 256 ? 256 : drop, 0); if (!n) break; drop -= n; }
  }
  xStreamBufferSend(sb, data, len, 0);
}

// ===================== งาน DSP (คอร์ 1) =====================
static inline int32_t toI2S(float v) {
  if (v > 0.999999f) v = 0.999999f; else if (v < -0.999999f) v = -0.999999f;
  return (int32_t)(v * 2147483647.0f);
}

void dspTask(void *) {
  static int16_t raw[BLOCK * 2];
  static float xl[BLOCK], xr[BLOCK], y[NCH][BLOCK];
  static int32_t o1[BLOCK * 2];
#if USE_DAC2
  static int32_t o2[BLOCK * 2];
#endif
  bool playing = false;
  const float glideK = 0.3f, blockUs = BLOCK * 1e6f / FS;
  bool first = true;

  for (;;) {
    // ---- รับค่าที่เปลี่ยน ----
    if (pDirty) {
      Params n;
      portENTER_CRITICAL(&pmux); n = P; pDirty = false; portEXIT_CRITICAL(&pmux);
      if (!first && n.mode != D.mode) { pendingMode = true; fadeT = 0; }   // เปลี่ยนโหมด: ค่อยๆ เงียบก่อน
      if (pendingMode) { uint8_t m = D.mode; D = n; D.mode = m; } else D = n;
      applyParams(first); first = false;
    }
    if (pendingMode && fade < 0.001f) {
      portENTER_CRITICAL(&pmux); D.mode = P.mode; portEXIT_CRITICAL(&pmux);
      applyParams(true);
      for (auto &i : inp) { i.eq.clear(); i.ss.clear(); }
      for (auto &o : oc) { o.hp.clear(); o.lp.clear(); o.eq.clear(); memset(o.buf, 0, sizeof(o.buf)); o.lg = 1; }
      pendingMode = false; fadeT = 1;
    }

    // ---- อ่านเสียง ----
    size_t need = sizeof(raw), got = 0;
    if (!playing) {
      if (xStreamBufferBytesAvailable(sb) >= PREBUF) playing = true;
    }
    if (playing) {
      got = xStreamBufferReceive(sb, raw, need, pdMS_TO_TICKS(30));
      if (got < need) { memset((uint8_t *)raw + got, 0, need - got); if (got == 0) playing = false; }
    } else memset(raw, 0, need);

    uint32_t t0 = micros();
    bool stereo = D.mode == 1;
    // ---- อินพุต ----
    float ig0 = inGain, ig1 = inGainT, pk = inPeak;
    for (int i = 0; i < BLOCK; i++) {
      float g = ig0 + (ig1 - ig0) * (i + 1) / BLOCK;
      float l = raw[2 * i] * (1.0f / 32768) * g, r = raw[2 * i + 1] * (1.0f / 32768) * g;
      float a = fmaxf(fabsf(l), fabsf(r)); if (a > pk) pk = a;
      if (stereo) { xl[i] = l; xr[i] = r; } else xl[i] = (l + r) * 0.5f;
    }
    inGain = ig1; inPeak = pk;
    inp[0].ss.runBlock(xl, BLOCK); inp[0].eq.runBlock(xl, BLOCK);
    if (stereo) { inp[1].ss.runBlock(xr, BLOCK); inp[1].eq.runBlock(xr, BLOCK); }

    // ---- แยกช่อง ----
    if (stereo) {
      for (int i = 0; i < BLOCK; i++) { y[0][i] = (xl[i] + xr[i]) * 0.5f; y[1][i] = xl[i]; y[2][i] = xr[i]; }
    } else {
      for (int i = 0; i < BLOCK; i++) y[0][i] = y[1][i] = y[2][i] = xl[i];
    }

    // fade ทั้งระบบ (ตอนสลับโหมด) ~30 ms: ขยับได้ 0.1 ต่อบล็อก
    float f0 = fade, f1 = fade + constrain(fadeT - fade, -0.1f, 0.1f);
    if (f1 < 0.001f) f1 = 0;
    for (int c = 0; c < NCH; c++) {
      OutCh &o = oc[c]; float *x = y[c];
      o.hp.runBlock(x, BLOCK); o.lp.runBlock(x, BLOCK); o.eq.runBlock(x, BLOCK);
      float gT = o.mute ? 0 : o.gT, g0 = o.g, pk2 = o.peak, gr = o.grMax;
      float d0 = o.dl, d1 = o.dlT;
      if (fabsf(d1 - d0) > 2) d1 = d0 + (d1 > d0 ? 2 : -2);     // เลื่อนดีเลย์ช้าๆ
      for (int i = 0; i < BLOCK; i++) {
        float t = (i + 1) * (1.0f / BLOCK);
        float v = x[i] * (g0 + (gT - g0) * t);
        // ดีเลย์แบบเศษส่วน (linear interpolation)
        o.buf[o.w] = v;
        float d = d0 + (d1 - d0) * t;
        float rp = o.w - d; if (rp < 0) rp += DLY_LEN;
        int i0 = (int)rp; float fr = rp - i0; int i1 = i0 + 1; if (i1 >= DLY_LEN) i1 = 0;
        v = o.buf[i0] + (o.buf[i1] - o.buf[i0]) * fr;
        o.w = (o.w + 1) % DLY_LEN;
        // ลิมิตเตอร์ (peak)
        if (o.limOn) {
          float a = fabsf(v), want = a > o.limTh ? o.limTh / a : 1;
          o.lg += (want - o.lg) * (want < o.lg ? o.limA : o.limR);
          v *= o.lg;
          if (1 - o.lg > gr) gr = 1 - o.lg;
        } else o.lg = 1;
        v *= f0 + (f1 - f0) * t;
        float a = fabsf(v); if (a > pk2) pk2 = a;
        x[i] = v;
      }
      o.g = gT; o.dl = d1; o.peak = pk2; o.grMax = gr;
    }
    fade = f1;

    // ---- เลื่อนค่าสัมประสิทธิ์ ----
    for (auto &i : inp) { i.eq.glide(glideK); i.ss.glide(glideK); }
    for (auto &o : oc) { o.hp.glide(glideK); o.lp.glide(glideK); o.eq.glide(glideK); }

    // ---- ส่งออก ----
    for (int i = 0; i < BLOCK; i++) {
#if USE_DAC2
      if (stereo) { o1[2 * i] = toI2S(y[1][i]); o1[2 * i + 1] = toI2S(y[2][i]); }
      else        { o1[2 * i] = toI2S(y[1][i]); o1[2 * i + 1] = toI2S(y[2][i]); }
      int32_t s = toI2S(y[0][i]); o2[2 * i] = s; o2[2 * i + 1] = s;
#else
      // ไม่มี DAC2: รวมซับเข้า DAC1 (โมโน: ช่องซ้าย, สเตอริโอ: ทั้งสองช่อง)
      if (stereo) { o1[2 * i] = toI2S(y[1][i] + y[0][i]); o1[2 * i + 1] = toI2S(y[2][i] + y[0][i]); }
      else        { o1[2 * i] = toI2S(y[1][i] + y[0][i]); o1[2 * i + 1] = toI2S(y[2][i]); }
#endif
    }
    uint32_t us = micros() - t0;
    cpuLoad += (us / blockUs * 100 - cpuLoad) * 0.02f;
    i2sWrite(1, o1, sizeof(o1));
#if USE_DAC2
    i2sWrite(2, o2, sizeof(o2));
#endif
  }
}

// ===================== NVS =====================
bool loadBlob(const char *k, Params &p) {
  if (prefs.getBytesLength(k) != sizeof(Params)) return false;
  Params t; prefs.getBytes(k, &t, sizeof(t));
  if (t.ver != PARAM_VER) return false;
  p = t; return true;
}
void loadPresetNames() {
  for (int i = 0; i < NPRE; i++) {
    char k[4]; snprintf(k, 4, "p%d", i); Params t;
    if (loadBlob(k, t)) { strncpy(presetNames[i], t.name, 32); presetNames[i][32] = 0; }
    else presetNames[i][0] = 0;
  }
}

// ===================== BLE (Nordic UART) =====================
#define NUS_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_UUID  "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
BLECharacteristic *txC;
volatile bool bleConn = false;
QueueHandle_t lineQ;
String rxBuf;

class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConn = true; }
  void onDisconnect(BLEServer *s) override { bleConn = false; s->getAdvertising()->start(); }
};
class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = String(c->getValue().c_str());
    for (size_t i = 0; i < v.length(); i++) {
      char ch = v[i];
      if (ch == '\n' || ch == '\r') {
        if (rxBuf.length()) { char *s = strdup(rxBuf.c_str()); if (xQueueSend(lineQ, &s, 0) != pdTRUE) free(s); rxBuf = ""; }
      } else if (rxBuf.length() < 600) rxBuf += ch;
    }
  }
};

bool serReply = false;   // กำลังตอบคำสั่งที่มาจาก Serial Monitor
uint8_t serMeter = 0;    // 1 = พิมพ์มิเตอร์ลง Serial ต่อเนื่อง

void bleSend(const String &s) {
  if (serReply) Serial.println(s);
  if (!bleConn) return;
  String m = s + "\n";
  for (size_t i = 0; i < m.length(); i += 20) {
    size_t n = min((size_t)20, m.length() - i);
    txC->setValue((uint8_t *)m.c_str() + i, n);
    txC->notify();
    delay(4);
  }
}

String urlEnc(const char *s) {
  String o; const char *hex = "0123456789ABCDEF";
  for (; *s; s++) {
    uint8_t c = *s;
    if (isalnum(c) || c == '-' || c == '_' || c == '.') o += (char)c;
    else { o += '%'; o += hex[c >> 4]; o += hex[c & 15]; }
  }
  return o;
}
String urlDec(const String &s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '%' && i + 2 < s.length()) { o += (char)strtol(s.substring(i + 1, i + 3).c_str(), nullptr, 16); i += 2; }
    else if (c == '+') o += ' ';
    else o += c;
  }
  return o;
}

void sendState() {
  String j = "J{";
  char b[40];
  Params p;
  portENTER_CRITICAL(&pmux); p = P; portEXIT_CRITICAL(&pmux);
  for (int i = 0; i < nKeys; i++) {
    Key &k = keys[i];
    // อ่านจากสำเนา p ด้วยตำแหน่งเดียวกับใน P
    void *q = (uint8_t *)&p + ((uint8_t *)k.p - (uint8_t *)&P);
    if (k.isF) snprintf(b, sizeof(b), "\"%s\":%.6g,", k.k, *(float *)q);
    else snprintf(b, sizeof(b), "\"%s\":%d,", k.k, *(uint8_t *)q);
    j += b;
  }
  for (int i = 0; i < NPRE; i++) { j += "\"pn" + String(i) + "\":\"" + urlEnc(presetNames[i]) + "\","; }
  j += "\"pre\":" + String(activePreset) + ",\"lk\":" + String(locked) + ",\"lkset\":" + String(lockCode ? 1 : 0) + ",\"dac2\":" + String(USE_DAC2) + "}";
  bleSend(j);
}

int findKey(const String &k) {
  for (int i = 0; i < nKeys; i++) if (k == keys[i].k) return i;
  return -1;
}

void handleLine(String line) {
  line.trim();
  if (!line.length()) return;
  if (line == "get") { sendState(); return; }
  if (line.startsWith("unlock=")) {
    if (!lockCode || (uint16_t)(line.substring(7).toInt() + 1) == lockCode) { locked = 0; prefs.putUChar("lkd", 0); sendState(); }
    else bleSend("Ecode");
    return;
  }
  if (locked) { bleSend("Elocked"); return; }
  if (line.startsWith("lock=")) {
    int c = line.substring(5).toInt();
    if (c < 0 || c > 9999 || line.length() != 9) { bleSend("Ecode"); return; }
    lockCode = c + 1; locked = 1; prefs.putUShort("lk", lockCode); prefs.putUChar("lkd", 1); sendState(); return;
  }
  if (line == "save") { Params p; portENTER_CRITICAL(&pmux); p = P; portEXIT_CRITICAL(&pmux); prefs.putBytes("cur", &p, sizeof(p)); prefs.putChar("pre", activePreset); bleSend("Ksave"); return; }
  if (line == "reset") { portENTER_CRITICAL(&pmux); defaults(P); pDirty = true; portEXIT_CRITICAL(&pmux); activePreset = -1; sendState(); return; }
  if (line.startsWith("preset_load=")) {
    int n = line.substring(12).toInt(); char k[4]; snprintf(k, 4, "p%d", n); Params t;
    if (n < 0 || n >= NPRE || !loadBlob(k, t)) { bleSend("Epreset"); return; }
    portENTER_CRITICAL(&pmux); P = t; pDirty = true; portEXIT_CRITICAL(&pmux);
    activePreset = n; sendState(); return;
  }
  if (line.startsWith("preset_save=")) {
    int amp = line.indexOf('&'); int n = line.substring(12, amp < 0 ? line.length() : amp).toInt();
    if (n < 0 || n >= NPRE) { bleSend("Epreset"); return; }
    String name = "";
    int ni = line.indexOf("name="); if (ni >= 0) name = urlDec(line.substring(ni + 5));
    if (!name.length()) name = "Preset " + String(n + 1);
    Params p; const char *nm = name.c_str();
    portENTER_CRITICAL(&pmux); strncpy(P.name, nm, 32); P.name[32] = 0; p = P; portEXIT_CRITICAL(&pmux);
    char k[4]; snprintf(k, 4, "p%d", n); prefs.putBytes(k, &p, sizeof(p));
    strncpy(presetNames[n], p.name, 33); activePreset = n;
    bleSend("Kpreset"); sendState(); return;
  }
  // k=v&k=v : แยกค่าก่อน (มีการจองหน่วยความจำ) แล้วค่อยเขียนลง P ในช่วงล็อก
  int idx[64]; float val[64]; int n = 0, s = 0;
  while (s < (int)line.length() && n < 64) {
    int e = line.indexOf('&', s); if (e < 0) e = line.length();
    int eq = line.indexOf('=', s);
    if (eq > s && eq < e) {
      int i = findKey(line.substring(s, eq));
      if (i >= 0) { idx[n] = i; val[n] = constrain(line.substring(eq + 1, e).toFloat(), keys[i].mn, keys[i].mx); n++; }
    }
    s = e + 1;
  }
  if (!n) return;
  portENTER_CRITICAL(&pmux);
  for (int j = 0; j < n; j++) {
    Key &k = keys[idx[j]];
    if (k.isF) *(float *)k.p = val[j]; else *(uint8_t *)k.p = (uint8_t)lroundf(val[j]);
  }
  pDirty = true;
  portEXIT_CRITICAL(&pmux);
}

// ===================== setup / loop =====================
void setup() {
  Serial.begin(115200);
  defaults(P);
  buildKeys();
  prefs.begin("dsp", false);
  Params t;
  if (loadBlob("cur", t)) P = t;
  activePreset = prefs.getChar("pre", -1);
  lockCode = prefs.getUShort("lk", 0); locked = prefs.getUChar("lkd", 0) && lockCode;
  loadPresetNames();
  pDirty = true;

  sb = xStreamBufferCreate(SB_SIZE, 1);
  lineQ = xQueueCreate(16, sizeof(char *));

#if NEW_I2S
  i2sOpen(I2S_NUM_0, DAC1_BCK, DAC1_LCK, DAC1_DIN, &tx1);
#if USE_DAC2
  i2sOpen(I2S_NUM_1, DAC2_BCK, DAC2_LCK, DAC2_DIN, &tx2);
#endif
#else
  i2sOpen(I2S_NUM_0, DAC1_BCK, DAC1_LCK, DAC1_DIN);
#if USE_DAC2
  i2sOpen(I2S_NUM_1, DAC2_BCK, DAC2_LCK, DAC2_DIN);
#endif
#endif

  // A2DP ก่อน แล้วจึง BLE (ใช้คอนโทรลเลอร์โหมด BTDM ร่วมกัน)
  a2dp.set_default_bt_mode(ESP_BT_MODE_BTDM);
  a2dp.set_stream_reader(onAudio, false);
  a2dp.set_auto_reconnect(true);
  a2dp.start(BT_NAME);

  BLEDevice::init(BT_NAME);
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new SrvCb());
  BLEService *svc = srv->createService(NUS_UUID);
  txC = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txC->addDescriptor(new BLE2902());
  BLECharacteristic *rx = svc->createCharacteristic(RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCb());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_UUID);
  adv->setScanResponse(true);
  adv->start();

  xTaskCreatePinnedToCore(dspTask, "dsp", 8192, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
  Serial.println(F("\nDSP Rack v2 พร้อมทำงาน · Bluetooth: \"" BT_NAME "\""));
  serHelp();
}

static float todb(float a) { return a > 1e-5f ? 20 * log10f(a) : -100; }

// ===================== Serial Monitor (115200, ส่งแบบ Newline) =====================
void serHelp() {
  Serial.println(F("\n=== DSP Rack v2 · คำสั่ง Serial Monitor (115200 baud, Newline) ==="));
  Serial.println(F("help            แสดงคำสั่งทั้งหมด"));
  Serial.println(F("status          สถานะ: A2DP, BLE, CPU, บัฟเฟอร์, หน่วยความจำ"));
  Serial.println(F("show            ค่าที่ตั้งไว้ทั้งหมดแบบอ่านง่าย"));
  Serial.println(F("meter           เปิด/ปิด มิเตอร์ต่อเนื่อง (ทุก 0.5 วินาที)"));
  Serial.println(F("get             ค่าทั้งหมดแบบ JSON (เหมือนที่ส่งให้แอป)"));
  Serial.println(F("k=v&k=v         ตั้งค่า เช่น vol=-20  หรือ  x1=90&x2=3000  หรือ  c0gn=-3"));
  Serial.println(F("save / reset    บันทึกลงเครื่อง / กลับค่าเริ่มต้น"));
  Serial.println(F("preset_load=0   โหลดพรีเซ็ต (0-4) · preset_save=0&name=Home"));
  Serial.println(F("lock=1234 / unlock=1234"));
  Serial.println(F("คีย์: mode vol ing bal ieq ss ssf sss x1 x2 ft0-3 fs0-3"));
  Serial.println(F("     ช่อง c0=ซับ c1=กลาง/ซ้าย c2=แหลม/ขวา: gn dl iv mu so lt la lr lo eq"));
  Serial.println(F("     แบนด์อีคิว: i(ขาเข้า)/c0/c1/c2 + t f g q e + เลขแบนด์ 0-5 เช่น c1g2=-3"));
}

void serStatus() {
  Serial.printf("\n--- สถานะ (uptime %lu s) ---\n", (unsigned long)(millis() / 1000));
  Serial.printf("A2DP : %s, sample rate %u Hz, บัฟเฟอร์ %u/%u ไบต์\n",
                a2dp.is_connected() ? "เชื่อมต่อแล้ว" : "ยังไม่เชื่อมต่อ", (unsigned)a2dp.sample_rate(),
                (unsigned)xStreamBufferBytesAvailable(sb), (unsigned)SB_SIZE);
  Serial.printf("BLE  : %s\n", bleConn ? "แอปเชื่อมต่ออยู่" : "ไม่มีแอปเชื่อมต่อ");
  Serial.printf("CPU  : %.0f%% (คอร์ 1)\n", (float)cpuLoad);
  Serial.printf("RAM  : ว่าง %u ไบต์, PSRAM %u ไบต์\n", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getPsramSize());
  Serial.printf("DAC2 : %s\n", USE_DAC2 ? "ใช้งาน" : "ปิด (ซับรวมเข้า DAC1)");
  Serial.printf("ล็อก : %s · พรีเซ็ต: %d %s\n", locked ? "ล็อกอยู่" : "ไม่ได้ล็อก", activePreset + 1,
                activePreset >= 0 ? presetNames[activePreset] : "(ไม่ได้เลือก)");
}

void serShow() {
  Params p; portENTER_CRITICAL(&pmux); p = P; portEXIT_CRITICAL(&pmux);
  const char *SL[3] = {"12", "24", "48"}, *TY[2] = {"BW", "LR"}, *BT[3] = {"peak", "lowshelf", "highshelf"};
  bool st = p.mode == 1;
  const char *CN[3] = {"ซับ", st ? "ซ้าย" : "กลาง", st ? "ขวา" : "แหลม"};
  Serial.printf("\n--- ค่าปัจจุบัน ---\nโหมด: %s · วอลุ่ม %.1f dB · เกนขาเข้า %.1f dB%s\n", st ? "สเตอริโอ 2.1" : "โมโน 3 ทาง",
                p.vol, p.inG, st ? (" · บาลานซ์ " + String(p.bal, 0)).c_str() : "");
  Serial.printf("Subsonic: %s %.0f Hz %s dB\n", p.ssOn ? "ON" : "OFF", p.ssF, p.ssSl ? "24" : "12");
  Serial.printf("ครอส: x1 %.0f Hz", p.x1);
  if (!st) Serial.printf(" · x2 %.0f Hz", p.x2);
  Serial.println();
  const char *FN[4] = {"ซับ LP", "กลาง HP", "กลาง LP", "แหลม HP"};
  for (int i = 0; i < (st ? 2 : 4); i++) Serial.printf("  %-8s %s %s dB/oct\n", FN[i], TY[p.ft[i]], SL[p.fs[i]]);
  Serial.printf("อีคิวขาเข้า (%s):", p.ieqOn ? "ON" : "OFF");
  for (int b = 0; b < NB; b++) if (p.ieq[b].on && fabsf(p.ieq[b].g) > 0.01f) Serial.printf(" [%d %s %.0fHz %+.1fdB Q%.2f]", b + 1, BT[p.ieq[b].t], p.ieq[b].f, p.ieq[b].g, p.ieq[b].q);
  Serial.println();
  for (int c = 0; c < NCH; c++) {
    Chan &h = p.ch[c];
    Serial.printf("ช่อง %d %s: เกน %+.1f dB · ดีเลย์ %.2f ms · เฟส %s%s%s · ลิมิต %s %.1f dBFS A%.1f R%.0f ms\n", c, CN[c],
                  h.gain, h.dly, h.inv ? "กลับ" : "ปกติ", h.mute ? " · MUTE" : "", h.solo ? " · SOLO" : "",
                  h.limOn ? "ON" : "OFF", h.limT, h.limA, h.limR);
    Serial.printf("  อีคิว (%s):", h.eqOn ? "ON" : "OFF");
    for (int b = 0; b < NB; b++) if (h.eq[b].on && fabsf(h.eq[b].g) > 0.01f) Serial.printf(" [%d %s %.0fHz %+.1fdB Q%.2f]", b + 1, BT[h.eq[b].t], h.eq[b].f, h.eq[b].g, h.eq[b].q);
    Serial.println();
  }
}

void serialPoll() {
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\n' && ch != '\r') { if (buf.length() < 600) buf += ch; continue; }
    buf.trim();
    if (!buf.length()) continue;
    Serial.print("> "); Serial.println(buf);
    if (buf == "help" || buf == "?") serHelp();
    else if (buf == "status") serStatus();
    else if (buf == "show") serShow();
    else if (buf == "meter") { serMeter = !serMeter; Serial.println(serMeter ? "มิเตอร์: เปิด (พิมพ์ meter อีกครั้งเพื่อปิด)" : "มิเตอร์: ปิด"); }
    else {
      serReply = true;
      bool isSet = buf.indexOf('=') > 0 && !buf.startsWith("preset") && !buf.startsWith("lock") && !buf.startsWith("unlock");
      handleLine(buf);
      if (isSet) Serial.println(locked ? "ไม่ได้ตั้งค่า: เครื่องถูกล็อก" : "OK");
      serReply = false;
    }
    buf = "";
  }
}

void loop() {
  char *s;
  while (xQueueReceive(lineQ, &s, 0) == pdTRUE) { handleLine(String(s)); free(s); }
  serialPoll();

  static uint32_t lastM = 0;
  if (millis() - lastM >= 150) {
    lastM = millis();
    // อ่านแล้วรีเซ็ตค่าสูงสุดของรอบนี้ (DSP เขียนทับต่อเอง ค่าผิดเล็กน้อยได้ ไม่เป็นไร)
    float in = inPeak; inPeak = 0;
    float pk[NCH], gr[NCH];
    for (int c = 0; c < NCH; c++) { pk[c] = oc[c].peak; oc[c].peak = 0; gr[c] = oc[c].grMax; oc[c].grMax = 0; }
    if (bleConn) {
      char m[200];
      snprintf(m, sizeof(m), "M{\"in\":%.1f,\"sub\":%.1f,\"mid\":%.1f,\"hi\":%.1f,\"lim_sub\":%.1f,\"lim_mid\":%.1f,\"lim_hi\":%.1f,\"cpu\":%.0f}",
               todb(in), todb(pk[0]), todb(pk[1]), todb(pk[2]),
               -todb(1 - gr[0]), -todb(1 - gr[1]), -todb(1 - gr[2]), (float)cpuLoad);
      bleSend(m);
    }
    static uint8_t n = 0;
    if (serMeter && ++n >= 3) {   // ~0.5 วินาที
      n = 0;
      Serial.printf("IN %6.1f | SUB %6.1f | %s %6.1f | %s %6.1f dBFS | LIM %4.1f %4.1f %4.1f dB | CPU %3.0f%% | A2DP %s\n",
                    todb(in), todb(pk[0]), P.mode ? "L  " : "MID", todb(pk[1]), P.mode ? "R " : "HI", todb(pk[2]),
                    -todb(1 - gr[0]), -todb(1 - gr[1]), -todb(1 - gr[2]), (float)cpuLoad, a2dp.is_connected() ? "on" : "off");
    }
  }
  delay(5);
}
