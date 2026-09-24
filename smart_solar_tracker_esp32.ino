/*
  SMART SOLAR PANEL TRACKER - ESP32 + Firebase (Versi Lengkap)
  ================================================================
  Fitur:
  - Auto tracking matahari 2-axis (azimuth timur-barat & elevasi)
  - Monitoring & kontrol via Firebase Realtime Database
  - Mode AUTO / MANUAL, tombol AKTIF/NONAKTIF panel (kembali ke standby)
  - Sensor ARUS & DAYA (INA219): tegangan, arus, watt, energi harian & total (Wh)
  - Panel STATIS pembanding (opsional, sensor INA219 kedua) untuk membandingkan
    hasil tracker vs panel diam
  - Pencatatan riwayat tegangan & daya per 5 menit (untuk grafik & ekspor CSV)
  - Kalibrasi jarak jauh: ambang cahaya, batas servo, posisi standby, jadwal
    parkir malam, ambang alert tegangan rendah -- semua bisa diubah dari
    dashboard tanpa upload ulang kode
  - Jadwal otomatis: panel otomatis parkir di jam tertentu tiap malam
  - Notifikasi Telegram (opsional): panel on/off, tegangan rendah, saat online
  - Status "last_seen" agar dashboard tahu kalau ESP32 sedang offline

  LIBRARY YANG DIPERLUKAN (install via Library Manager):
  1. Firebase ESP Client by Mobizt
  2. ESP32Servo by Kevin Harrington
  3. Adafruit INA219 by Adafruit (+ dependensi Adafruit BusIO)
  4. UniversalTelegramBot by Brian Lough (hanya perlu kalau TELEGRAM_ENABLED true)

  WIRING:
  - LDR Kiri-Atas / Kanan-Atas / Kiri-Bawah / Kanan-Bawah -> GPIO34/35/32/33
  - Servo Azimuth (Timur-Barat) -> GPIO13
  - Servo Elevasi  (Atas-Bawah)  -> GPIO12
  - Relay pemutus panel (opsional) -> GPIO14
  - INA219 panel utama  -> I2C: SDA=GPIO21, SCL=GPIO22, alamat default 0x40
  - INA219 panel statis (opsional, pembanding) -> I2C sama, alamat 0x41
    (ubah dengan solder jumper "A0" di modul INA219)
  - Servo power: gunakan power supply eksternal 5V terpisah, GND disatukan!
*/

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ESP32Servo.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// ===================== KONFIGURASI WIFI & FIREBASE =====================
#define WIFI_SSID       "NAMA_WIFI_KAMU"
#define WIFI_PASSWORD   "PASSWORD_WIFI_KAMU"

#define API_KEY         "ISI_API_KEY_FIREBASE"
#define DATABASE_URL    "https://NAMA-PROJECT-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define USER_EMAIL      "email_akun_firebase@example.com"
#define USER_PASSWORD   "password_akun_firebase"

FirebaseData   fbdo;
FirebaseAuth   auth;
FirebaseConfig config;

unsigned long lastFirebaseSend = 0;
const unsigned long firebaseInterval = 2000; // kirim data tiap 2 detik

// ===================== KONFIGURASI NOTIFIKASI TELEGRAM (OPSIONAL) =====================
#define TELEGRAM_ENABLED false // ganti true untuk mengaktifkan notifikasi
#define TELEGRAM_BOT_TOKEN "ISI_TOKEN_BOT_TELEGRAM"
#define TELEGRAM_CHAT_ID   "ISI_CHAT_ID_TELEGRAM_KAMU"

WiFiClientSecure telegramClient;
UniversalTelegramBot bot(TELEGRAM_BOT_TOKEN, telegramClient);
unsigned long lastAlertTime = 0;
const unsigned long alertCooldown = 30UL * 60UL * 1000UL; // jeda 30 menit antar alert sejenis

void sendTelegramAlert(String message) {
  if (!TELEGRAM_ENABLED) return;
  bot.sendMessage(TELEGRAM_CHAT_ID, message, "");
}

// ===================== SENSOR ARUS & DAYA (INA219) =====================
Adafruit_INA219 ina219Main;          // panel utama (tracker), alamat default 0x40
#define ENABLE_STATIC_PANEL false     // ganti true kalau kamu pasang panel statis pembanding
Adafruit_INA219 ina219Static(0x41);  // panel statis (opsional)

// ===================== PENYIMPANAN ENERGI (tersimpan walau ESP32 restart) =====================
Preferences prefs;
float energyTotalWh       = 0; // akumulasi sepanjang waktu
float energyTodayWh       = 0; // reset tiap pergantian hari
float energyTodayStaticWh = 0;
int   lastLoggedDay       = -1; // tm_yday terakhir, dipakai deteksi pergantian hari

// ===================== KONFIGURASI PENCATATAN & NTP =====================
const char* NTP_SERVER   = "pool.ntp.org";
const long  GMT_OFFSET   = 7 * 3600;  // WIB = UTC+7. Ganti 8*3600 (WITA) atau 9*3600 (WIT) sesuai lokasi
const int   DST_OFFSET   = 0;

unsigned long lastVoltageLog = 0;
const unsigned long voltageLogInterval = 5UL * 60UL * 1000UL; // catat tiap 5 menit

// ===================== PIN DEFINISI =====================
#define LDR_TOP_LEFT     34
#define LDR_TOP_RIGHT    35
#define LDR_BOTTOM_LEFT  32
#define LDR_BOTTOM_RIGHT 33

#define SERVO_AZIMUTH_PIN  13
#define SERVO_ELEVASI_PIN  12

#define RELAY_PIN 14
#define RELAY_ACTIVE_LOW true // ganti false kalau modul relay kamu aktif-HIGH

Servo servoAzimuth;
Servo servoElevasi;

// ===================== VARIABEL TRACKING =====================
int posAzimuth = 90;
int posElevasi = 90;
const int SERVO_STEP = 1; // derajat pergerakan tiap update auto-track

bool modeAuto      = true;
bool panelPower     = true;
bool wasPoweredOff  = false; // dipakai mendeteksi transisi aktif<->standby sekali saja

unsigned long lastTrackUpdate = 0;
const unsigned long trackInterval = 200;

// ===================== KONFIGURASI YANG BISA DIUBAH DARI DASHBOARD =====================
// Nilai default ini dipakai kalau belum ada isinya di Firebase; setelah itu ESP32
// akan mengikuti nilai yang tersimpan di /solar_tracker/config/... setiap sinkron.
int   lightThreshold    = 40;
int   servoMinCfg       = 10;
int   servoMaxCfg       = 170;
int   standbyAzimuthCfg = 90;
int   standbyElevasiCfg = 90;
bool  scheduleEnabled   = false;
int   autoParkHour      = 18; // jam 18:00 mulai parkir malam (kalau scheduleEnabled true)
int   autoWakeHour      = 6;  // jam 06:00 aktif lagi
float lowVoltageAlert   = 3.0;

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Sensor arus/daya
  if (!ina219Main.begin()) {
    Serial.println("Peringatan: INA219 panel utama tidak terdeteksi, cek wiring I2C.");
  }
  if (ENABLE_STATIC_PANEL && !ina219Static.begin()) {
    Serial.println("Peringatan: INA219 panel statis tidak terdeteksi.");
  }

  // Penyimpanan energi permanen
  prefs.begin("solartrkr", false);
  energyTotalWh = prefs.getFloat("energyTotal", 0.0);

  // Setup servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoAzimuth.setPeriodHertz(50);
  servoElevasi.setPeriodHertz(50);
  servoAzimuth.attach(SERVO_AZIMUTH_PIN, 500, 2400);
  servoElevasi.attach(SERVO_ELEVASI_PIN, 500, 2400);
  servoAzimuth.write(posAzimuth);
  servoElevasi.write(posElevasi);

  pinMode(RELAY_PIN, OUTPUT);
  setRelay(true);

  analogReadResolution(12);

  // Koneksi WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Menghubungkan ke WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi Terhubung: " + WiFi.localIP().toString());

  // Sinkronkan jam
  configTime(GMT_OFFSET, DST_OFFSET, NTP_SERVER);
  Serial.print("Menyinkronkan waktu (NTP)");
  time_t nowT = time(nullptr);
  while (nowT < 8 * 3600 * 2) {
    delay(300);
    Serial.print(".");
    nowT = time(nullptr);
  }
  Serial.println(" OK");

  // Konfigurasi Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.token_status_callback = tokenStatusCallback;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  if (Firebase.ready()) {
    Firebase.RTDB.setBool(&fbdo, "/solar_tracker/mode_auto", modeAuto);
    Firebase.RTDB.setBool(&fbdo, "/solar_tracker/power", panelPower);
    Firebase.RTDB.setString(&fbdo, "/solar_tracker/status", "AKTIF");

    // Isi nilai default konfigurasi kalau belum pernah diset dari dashboard
    initConfigDefaultInt("/solar_tracker/config/light_threshold", lightThreshold);
    initConfigDefaultInt("/solar_tracker/config/servo_min", servoMinCfg);
    initConfigDefaultInt("/solar_tracker/config/servo_max", servoMaxCfg);
    initConfigDefaultInt("/solar_tracker/config/standby_azimuth", standbyAzimuthCfg);
    initConfigDefaultInt("/solar_tracker/config/standby_elevasi", standbyElevasiCfg);
    initConfigDefaultBool("/solar_tracker/config/auto_park_enabled", scheduleEnabled);
    initConfigDefaultInt("/solar_tracker/config/auto_park_hour", autoParkHour);
    initConfigDefaultInt("/solar_tracker/config/auto_wake_hour", autoWakeHour);
    initConfigDefaultFloat("/solar_tracker/config/low_voltage_alert", lowVoltageAlert);
  }

  if (TELEGRAM_ENABLED) {
    telegramClient.setInsecure(); // lewati verifikasi sertifikat (praktik umum di proyek hobi)
    sendTelegramAlert("🔆 Solar tracker online dan siap tracking.");
  }
}

// ===================== HELPER KONFIGURASI DEFAULT =====================
void initConfigDefaultInt(const char* path, int defaultValue) {
  if (!Firebase.RTDB.getInt(&fbdo, path)) Firebase.RTDB.setInt(&fbdo, path, defaultValue);
}
void initConfigDefaultFloat(const char* path, float defaultValue) {
  if (!Firebase.RTDB.getFloat(&fbdo, path)) Firebase.RTDB.setFloat(&fbdo, path, defaultValue);
}
void initConfigDefaultBool(const char* path, bool defaultValue) {
  if (!Firebase.RTDB.getBool(&fbdo, path)) Firebase.RTDB.setBool(&fbdo, path, defaultValue);
}

// ===================== BACA KONFIGURASI TERBARU DARI DASHBOARD =====================
void readConfig() {
  if (Firebase.RTDB.getInt(&fbdo, "/solar_tracker/config/light_threshold")) lightThreshold = fbdo.intData();
  if (Firebase.RTDB.getInt(&fbdo, "/solar_tracker/config/servo_min")) servoMinCfg = fbdo.intData();
  if (Firebase.RTDB.getInt(&fbdo, "/solar_tracker/config/servo_max")) servoMaxCfg = fbdo.intData();
  if (Firebase.RTDB.getInt(&fbdo, "/solar_tracker/config/standby_azimuth")) standbyAzimuthCfg = fbdo.intData();
  if (Firebase.RTDB.getInt(&fbdo, "/solar_tracker/config/standby_elevasi")) standbyElevasiCfg = fbdo.intData();
  if (Firebase.RTDB.getBool(&fbdo, "/solar_tracker/config/auto_park_enabled")) scheduleEnabled = fbdo.boolData();
  if (Firebase.RTDB.getInt(&fbdo, "/solar_tracker/config/auto_park_hour")) autoParkHour = fbdo.intData();
  if (Firebase.RTDB.getInt(&fbdo, "/solar_tracker/config/auto_wake_hour")) autoWakeHour = fbdo.intData();
  if (Firebase.RTDB.getFloat(&fbdo, "/solar_tracker/config/low_voltage_alert")) lowVoltageAlert = fbdo.floatData();
}

// ===================== JADWAL PARKIR MALAM OTOMATIS =====================
bool isNightSchedule() {
  if (!scheduleEnabled) return false;
  time_t nowT = time(nullptr);
  if (nowT < 8 * 3600 * 2) return false;
  struct tm *tmNow = localtime(&nowT);
  int hr = tmNow->tm_hour;
  if (autoParkHour <= autoWakeHour) {
    return (hr >= autoParkHour || hr < autoWakeHour);
  }
  return (hr >= autoParkHour && hr < autoWakeHour);
}

// ===================== RELAY HELPER =====================
void setRelay(bool on) {
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(RELAY_PIN, level ? HIGH : LOW);
}

// ===================== BACA LDR =====================
struct LightReading {
  int topLeft, topRight, bottomLeft, bottomRight;
  int avgTop, avgBottom, avgLeft, avgRight;
};

LightReading readLDR() {
  LightReading r;
  r.topLeft     = analogRead(LDR_TOP_LEFT);
  r.topRight    = analogRead(LDR_TOP_RIGHT);
  r.bottomLeft  = analogRead(LDR_BOTTOM_LEFT);
  r.bottomRight = analogRead(LDR_BOTTOM_RIGHT);

  r.avgTop    = (r.topLeft + r.topRight) / 2;
  r.avgBottom = (r.bottomLeft + r.bottomRight) / 2;
  r.avgLeft   = (r.topLeft + r.bottomLeft) / 2;
  r.avgRight  = (r.topRight + r.bottomRight) / 2;
  return r;
}

// ===================== LOGIKA TRACKING OTOMATIS =====================
void autoTrack() {
  LightReading light = readLDR();

  int diffLR = light.avgLeft - light.avgRight;
  if (abs(diffLR) > lightThreshold) {
    if (diffLR > 0 && posAzimuth > servoMinCfg) posAzimuth -= SERVO_STEP;
    else if (diffLR < 0 && posAzimuth < servoMaxCfg) posAzimuth += SERVO_STEP;
    servoAzimuth.write(posAzimuth);
  }

  int diffTB = light.avgTop - light.avgBottom;
  if (abs(diffTB) > lightThreshold) {
    if (diffTB > 0 && posElevasi < servoMaxCfg) posElevasi += SERVO_STEP;
    else if (diffTB < 0 && posElevasi > servoMinCfg) posElevasi -= SERVO_STEP;
    servoElevasi.write(posElevasi);
  }
}

// ===================== GERAK HALUS SERVO =====================
void moveServoSmooth(Servo &servo, int fromAngle, int toAngle) {
  int step = (toAngle > fromAngle) ? 1 : -1;
  for (int a = fromAngle; a != toAngle; a += step) {
    servo.write(a);
    delay(12);
  }
  servo.write(toAngle);
}

// ===================== PANEL KE POSISI STANDBY =====================
void goToStandbyPosition(String statusText) {
  Serial.println("Panel ke posisi standby (" + statusText + ")");

  if (!servoAzimuth.attached()) servoAzimuth.attach(SERVO_AZIMUTH_PIN, 500, 2400);
  if (!servoElevasi.attached()) servoElevasi.attach(SERVO_ELEVASI_PIN, 500, 2400);

  moveServoSmooth(servoAzimuth, posAzimuth, standbyAzimuthCfg);
  moveServoSmooth(servoElevasi, posElevasi, standbyElevasiCfg);
  posAzimuth = standbyAzimuthCfg;
  posElevasi = standbyElevasiCfg;

  delay(300);
  servoAzimuth.detach();
  servoElevasi.detach();

  setRelay(false);

  if (Firebase.ready()) {
    Firebase.RTDB.setString(&fbdo, "/solar_tracker/status", statusText);
  }
}

// ===================== PANEL DIAKTIFKAN KEMBALI =====================
void wakeUpPanel() {
  Serial.println("Panel diaktifkan kembali");
  if (!servoAzimuth.attached()) servoAzimuth.attach(SERVO_AZIMUTH_PIN, 500, 2400);
  if (!servoElevasi.attached()) servoElevasi.attach(SERVO_ELEVASI_PIN, 500, 2400);
  setRelay(true);
  if (Firebase.ready()) {
    Firebase.RTDB.setString(&fbdo, "/solar_tracker/status", "AKTIF");
  }
}

// ===================== ALERT TEGANGAN RENDAH (TELEGRAM) =====================
void checkLowVoltageAlert(float voltage) {
  if (!TELEGRAM_ENABLED || !panelPower) return;
  time_t nowT = time(nullptr);
  if (nowT < 8 * 3600 * 2) return;
  struct tm *tmNow = localtime(&nowT);
  int hr = tmNow->tm_hour;
  if (hr < 6 || hr >= 18) return; // hanya cek di jam siang, malam wajar tegangannya rendah

  if (voltage < lowVoltageAlert && millis() - lastAlertTime > alertCooldown) {
    sendTelegramAlert("⚠️ Tegangan panel surya rendah: " + String(voltage, 2) +
                       " V (ambang: " + String(lowVoltageAlert, 2) + " V). Cek posisi/kabel panel.");
    lastAlertTime = millis();
  }
}

// ===================== BACA & KIRIM DAYA/ENERGI =====================
void readAndPublishPower() {
  float busVoltage  = ina219Main.getBusVoltage_V();
  float current_mA  = ina219Main.getCurrent_mA();
  float power_W     = busVoltage * (current_mA / 1000.0);
  if (!panelPower) power_W = 0;

  Firebase.RTDB.setFloat(&fbdo, "/solar_tracker/voltage", busVoltage);
  Firebase.RTDB.setFloat(&fbdo, "/solar_tracker/current", current_mA / 1000.0);
  Firebase.RTDB.setFloat(&fbdo, "/solar_tracker/power_watt", power_W);

  float hours = firebaseInterval / 3600000.0; // interval sinkron dalam satuan jam
  energyTodayWh += power_W * hours;
  Firebase.RTDB.setFloat(&fbdo, "/solar_tracker/energy_today_wh", energyTodayWh);
  Firebase.RTDB.setFloat(&fbdo, "/solar_tracker/energy_total_wh", energyTotalWh + energyTodayWh);

  if (ENABLE_STATIC_PANEL) {
    float sV = ina219Static.getBusVoltage_V();
    float sI = ina219Static.getCurrent_mA();
    float sP = sV * (sI / 1000.0);
    energyTodayStaticWh += sP * hours;
    Firebase.RTDB.setFloat(&fbdo, "/solar_tracker/static_panel/voltage", sV);
    Firebase.RTDB.setFloat(&fbdo, "/solar_tracker/static_panel/current", sI / 1000.0);
    Firebase.RTDB.setFloat(&fbdo, "/solar_tracker/static_panel/power_watt", sP);
    Firebase.RTDB.setFloat(&fbdo, "/solar_tracker/static_panel/energy_today_wh", energyTodayStaticWh);
  }

  checkLowVoltageAlert(busVoltage);
}

// ===================== DETEKSI PERGANTIAN HARI -> SIMPAN LOG HARIAN =====================
void checkDayRollover() {
  time_t nowT = time(nullptr);
  if (nowT < 8 * 3600 * 2) return;
  struct tm *tmNow = localtime(&nowT);

  if (lastLoggedDay == -1) {
    lastLoggedDay = tmNow->tm_yday;
    return;
  }

  if (tmNow->tm_yday != lastLoggedDay) {
    time_t yesterday = nowT - 86400;
    struct tm *tmYesterday = localtime(&yesterday);
    char dateStr[11];
    strftime(dateStr, sizeof(dateStr), "%Y-%m-%d", tmYesterday);

    Firebase.RTDB.setFloat(&fbdo, ("/solar_tracker/energy_daily_log/" + String(dateStr)).c_str(), energyTodayWh);
    if (ENABLE_STATIC_PANEL) {
      Firebase.RTDB.setFloat(&fbdo, ("/solar_tracker/energy_daily_log_static/" + String(dateStr)).c_str(), energyTodayStaticWh);
    }

    energyTotalWh += energyTodayWh;
    prefs.putFloat("energyTotal", energyTotalWh);

    energyTodayWh = 0;
    energyTodayStaticWh = 0;
    lastLoggedDay = tmNow->tm_yday;
  }
}

// ===================== CATAT RIWAYAT TEGANGAN & DAYA =====================
void logReadings() {
  time_t nowT = time(nullptr);
  if (nowT < 8 * 3600 * 2) return;

  float busVoltage = ina219Main.getBusVoltage_V();
  float current_mA = ina219Main.getCurrent_mA();
  float power_W    = panelPower ? busVoltage * (current_mA / 1000.0) : 0;

  String base = String((unsigned long)nowT);
  Firebase.RTDB.setFloat(&fbdo, ("/solar_tracker/voltage_log/" + base).c_str(), busVoltage);
  Firebase.RTDB.setFloat(&fbdo, ("/solar_tracker/power_log/" + base).c_str(), power_W);
}

// ===================== KIRIM & AMBIL DATA FIREBASE =====================
void syncFirebase() {
  if (!Firebase.ready()) return;

  LightReading light = readLDR();
  Firebase.RTDB.setInt(&fbdo, "/solar_tracker/ldr/top_left", light.topLeft);
  Firebase.RTDB.setInt(&fbdo, "/solar_tracker/ldr/top_right", light.topRight);
  Firebase.RTDB.setInt(&fbdo, "/solar_tracker/ldr/bottom_left", light.bottomLeft);
  Firebase.RTDB.setInt(&fbdo, "/solar_tracker/ldr/bottom_right", light.bottomRight);
  Firebase.RTDB.setInt(&fbdo, "/solar_tracker/servo/azimuth", posAzimuth);
  Firebase.RTDB.setInt(&fbdo, "/solar_tracker/servo/elevasi", posElevasi);

  readAndPublishPower();
  checkDayRollover();

  time_t nowT = time(nullptr);
  if (nowT > 8 * 3600 * 2) {
    Firebase.RTDB.setInt(&fbdo, "/solar_tracker/last_seen", (int)nowT);
  }

  readConfig();

  if (Firebase.RTDB.getBool(&fbdo, "/solar_tracker/mode_auto")) {
    modeAuto = fbdo.boolData();
  }

  bool previousPower = panelPower;
  if (Firebase.RTDB.getBool(&fbdo, "/solar_tracker/power")) {
    panelPower = fbdo.boolData();
  }
  if (panelPower != previousPower) {
    sendTelegramAlert(panelPower ? "🔆 Panel diaktifkan dari dashboard." : "⏸️ Panel dinonaktifkan dari dashboard.");
  }

  if (panelPower && !modeAuto) {
    if (Firebase.RTDB.getInt(&fbdo, "/solar_tracker/manual/azimuth")) {
      int manualAz = fbdo.intData();
      if (manualAz >= servoMinCfg && manualAz <= servoMaxCfg) {
        posAzimuth = manualAz;
        servoAzimuth.write(posAzimuth);
      }
    }
    if (Firebase.RTDB.getInt(&fbdo, "/solar_tracker/manual/elevasi")) {
      int manualEl = fbdo.intData();
      if (manualEl >= servoMinCfg && manualEl <= servoMaxCfg) {
        posElevasi = manualEl;
        servoElevasi.write(posElevasi);
      }
    }
  }
}

// ===================== LOOP UTAMA =====================
void loop() {
  unsigned long now = millis();
  bool nightSchedule  = isNightSchedule();
  bool effectiveActive = panelPower && !nightSchedule;

  if (!effectiveActive) {
    if (!wasPoweredOff) {
      goToStandbyPosition(nightSchedule ? "STANDBY (jadwal malam)" : "STANDBY (manual)");
      wasPoweredOff = true;
    }
  } else {
    if (wasPoweredOff) {
      wakeUpPanel();
      wasPoweredOff = false;
    }
    if (modeAuto && now - lastTrackUpdate >= trackInterval) {
      lastTrackUpdate = now;
      autoTrack();
    }
  }

  if (now - lastFirebaseSend >= firebaseInterval) {
    lastFirebaseSend = now;
    syncFirebase();
  }

  if (now - lastVoltageLog >= voltageLogInterval) {
    lastVoltageLog = now;
    logReadings();
  }
}
