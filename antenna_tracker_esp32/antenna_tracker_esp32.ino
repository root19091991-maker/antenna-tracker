// ESP32: поворотна платформа антени (пан + тильт, 2 серво).
// Команди з ПК: по USB-serial (115200) І по мережі UDP (Ethernet W5500 або Wi-Fi).
//
// Протокол (рядки закінчуються \n, по UDP можна і без \n):
//   T,az,el     азимут 0..360 (0 = північ), кут місця 0..90  -> антена наводиться
//   P,pan,tilt  прямі кути серво 0..180 (для калібрування)
//   C           центр: pan=90, tilt=90
//   ?           вивести стан
//
// РЕЖИМИ: USE_TILT 0 = тільки поворот по осі на PAN_RANGE_DEG (напр. 270°, одне серво на 270°);
//         USE_TILT 1 = pan + tilt на серво 180° (механіка нижче).
//
// Механіка для USE_TILT 1 (серво на 180°): "через верх". Pan 0..180 покриває півколо азимутів, а коли ціль
// у протилежній півкулі, pan = az-180 і тильт перевертається (tilt = 180 - el).
//   tilt 0   = антена дивиться горизонтально в бік pan
//   tilt 90  = вертикально вгору
//   tilt 180 = горизонтально у протилежний бік
//
// Підключення: сигнал pan -> PIN_PAN, тильт -> PIN_TILT, GND серво -> GND ESP32.
// Серво живіть ОКРЕМО від 5 В (не від ESP32!), мінус спільний з ESP32.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ETH.h>
#include <SPI.h>
#include <WebServer.h>

// ---------- Мережа ----------
#define NET_MODE 2                 // 0 = Ethernet W5500, 1 = Wi-Fi клієнт (до вашого роутера), 2 = власна Wi-Fi мережа трекера
const char* AP_SSID = "ANTENNA-TRACKER";      // NET_MODE 2: ім'я мережі, яку створює плата
const char* AP_PASS = "Qwert12345)";             // NET_MODE 2: пароль (мінімум 8 символів)
// У режимі 2: підключіться до цієї мережі, відкрийте http://192.168.4.1/ (ручне керування), UDP-команди на 192.168.4.1:5005
const char* MY_WIFI_SSID = "ВАШ_WIFI";        // потрібно тільки для NET_MODE 1
const char* MY_WIFI_PASS = "ВАШ_ПАРОЛЬ";
IPAddress TRK_IP(192, 168, 1, 50);       // IP трекера (вільний у вашій мережі!)
IPAddress TRK_GW(192, 168, 1, 1);         // адреса роутера
IPAddress TRK_MASK(255, 255, 255, 0);
IPAddress TRK_DNS(192, 168, 1, 1);
const uint16_t CMD_PORT = 5005;            // порт команд

// Піни модуля W5500 (SPI). Живлення модуля тільки 3.3 В!
const int W5500_SCK  = 25;
const int W5500_MISO = 26;
const int W5500_MOSI = 27;
const int W5500_CS   = 32;
const int W5500_INT  = 33;
const int W5500_RST  = 4;

// ---------- Налаштування ----------
#if defined(CONFIG_IDF_TARGET_ESP32C3)
const int PIN_PAN  = 4;
const int PIN_TILT = 5;
#else
const int PIN_PAN  = 18;
const int PIN_TILT = 19;
#endif

#define USE_TILT 0                  // 0 = тільки поворот по осі (без нахилу), 1 = pan + tilt
const float PAN_RANGE_DEG = 270.0;  // повний хід pan-серво: 270 (серво на 270°) або 180 (звичайне серво)
const float PAN_ZERO_AZ  = 0.0;     // азимут, у який дивиться антена при pan=0 (0 = північ, 90 = схід)
const bool  PAN_REVERSE  = false;   // true, якщо pan крутиться у зворотний бік
const bool  TILT_REVERSE = false;   // true, якщо тильт крутиться у зворотний бік
const float PAN_TRIM     = 0.0;     // підгонка нуля pan, градуси
const float TILT_TRIM    = 0.0;     // підгонка нуля tilt, градуси
const int   SERVO_MIN_US = 500;     // імпульс для 0 градусів (залежить від серво: 500..1000)
const int   SERVO_MAX_US = 2500;    // імпульс для повного ходу (PAN_RANGE_DEG): 2000..2500
const float MAX_SPEED_DPS = 60.0;   // макс. швидкість серво, градусів/с (плавний рух)
// -----------------------------------

const int PWM_FREQ = 50;
const int PWM_RES  = 14;            // 16384 тіків на період 20000 мкс

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmBegin() {
  ledcAttach(PIN_PAN, PWM_FREQ, PWM_RES);
#if USE_TILT
  ledcAttach(PIN_TILT, PWM_FREQ, PWM_RES);
#endif
}
void pwmDuty(int pin, uint32_t d) { ledcWrite(pin, d); }
#else
void pwmBegin() {
  ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_PAN, 0);
#if USE_TILT
  ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_TILT, 1);
#endif
}
void pwmDuty(int pin, uint32_t d) { ledcWrite(pin == PIN_PAN ? 0 : 1, d); }
#endif

float curPan = PAN_RANGE_DEG / 2, curTilt = 90;      // текуче положення, градуси
float tgtPan = PAN_RANGE_DEG / 2, tgtTilt = 90;      // ціль, градуси
String line;
unsigned long lastStep = 0;

void writeServo(int pin, float deg, float range) {
  deg = constrain(deg, 0.0f, range);
  float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / range;
  pwmDuty(pin, (uint32_t)(us * 16384.0f / 20000.0f));
}

// az/el (градуси) -> кути серво pan/tilt
void aimToServos(float az, float el, float &pan, float &tilt) {
  el = constrain(el, 0.0f, 90.0f);
  float rel = fmodf(az - PAN_ZERO_AZ, 360.0f);
  if (rel < 0) rel += 360.0f;
#if USE_TILT
  // pan + tilt на звичайних серво 180°: "через верх"
  if (rel <= 180.0f) { pan = rel;          tilt = el; }
  else               { pan = rel - 180.0f; tilt = 180.0f - el; }
  if (PAN_REVERSE)  pan  = 180.0f - pan;
  if (TILT_REVERSE) tilt = 180.0f - tilt;
  pan  = constrain(pan + PAN_TRIM, 0.0f, 180.0f);
  tilt = constrain(tilt + TILT_TRIM, 0.0f, 180.0f);
#else
  // тільки поворот по осі на PAN_RANGE_DEG (напр. 270°). Якщо азимут у "мертвій зоні",
  // антена стає на найближчий край діапазону.
  if (rel <= PAN_RANGE_DEG) pan = rel;
  else pan = ((rel - PAN_RANGE_DEG) < (360.0f - rel)) ? PAN_RANGE_DEG : 0.0f;
  if (PAN_REVERSE) pan = PAN_RANGE_DEG - pan;
  pan = constrain(pan + PAN_TRIM, 0.0f, PAN_RANGE_DEG);
  tilt = 90;
#endif
}

WiFiUDP udp;
bool udpUp = false;

bool netUp() {
#if NET_MODE == 0
  return ETH.linkUp();
#elif NET_MODE == 2
  return true;
#else
  return WiFi.status() == WL_CONNECTED;
#endif
}

void netBegin() {
#if NET_MODE == 0
  ETH.begin(ETH_PHY_W5500, 1, W5500_CS, W5500_INT, W5500_RST, SPI3_HOST, W5500_SCK, W5500_MISO, W5500_MOSI);
  ETH.config(TRK_IP, TRK_GW, TRK_MASK, TRK_DNS);
#elif NET_MODE == 2
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
#else
  WiFi.mode(WIFI_STA);
  WiFi.config(TRK_IP, TRK_GW, TRK_MASK, TRK_DNS);
  WiFi.begin(MY_WIFI_SSID, MY_WIFI_PASS);
#endif
}

// Обробляє один рядок команди, повертає відповідь
String handleLine(String s) {
  s.trim();
  if (s.length() == 0) return "";
  char buf[96];
  char cmd = s[0];
  if (cmd == 'T' || cmd == 'P') {
    int c1 = s.indexOf(',');
    int c2 = s.indexOf(',', c1 + 1);
    if (c1 < 0 || c2 < 0) return "ERR format";
    float a = s.substring(c1 + 1, c2).toFloat();
    float b = s.substring(c2 + 1).toFloat();
    if (cmd == 'T') aimToServos(a, b, tgtPan, tgtTilt);
    else { tgtPan = constrain(a, 0.0f, PAN_RANGE_DEG); tgtTilt = constrain(b, 0.0f, 180.0f); }
    snprintf(buf, sizeof(buf), "OK pan=%.1f tilt=%.1f", tgtPan, tgtTilt);
    return String(buf);
  } else if (cmd == 'C') {
    tgtPan = PAN_RANGE_DEG / 2; tgtTilt = 90;
    return "OK center";
  } else if (cmd == '?') {
    snprintf(buf, sizeof(buf), "pan=%.1f->%.1f tilt=%.1f->%.1f net=%s", curPan, tgtPan, curTilt, tgtTilt, netUp() ? "up" : "down");
    return String(buf);
  }
  return "ERR unknown";
}

#if NET_MODE == 2
WebServer web(80);
const char PAGE[] PROGMEM =
"<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'><title>Трекер</title>"
"<body style='font-family:sans-serif;max-width:420px;margin:20px auto'><h2>Антена трекер</h2>"
"<p>Азимут (град): <b id=a>0</b></p><input id=s type=range min=0 max=360 value=0 style='width:100%'>"
"<p>Висота (град): <b id=e>0</b></p><input id=h type=range min=0 max=90 value=0 style='width:100%'>"
"<p><button onclick=\"c('C')\">Центр</button></p><pre id=r></pre>"
"<script>function c(x){fetch('/cmd?c='+encodeURIComponent(x)).then(r=>r.text()).then(t=>r.textContent=t)}"
"function t(){a.textContent=s.value;e.textContent=h.value;c('T,'+s.value+','+h.value)}"
"s.oninput=t;h.oninput=t;</script>";
void webBegin() {
  web.on("/", []() { web.send_P(200, "text/html; charset=utf-8", PAGE); });
  web.on("/cmd", []() { web.send(200, "text/plain; charset=utf-8", handleLine(web.arg("c"))); });
  web.begin();
}
#endif

void setup() {
  Serial.begin(115200);
  pwmBegin();
  writeServo(PIN_PAN, curPan, PAN_RANGE_DEG);
#if USE_TILT
  writeServo(PIN_TILT, curTilt, 180.0f);
#endif
  netBegin();
#if NET_MODE == 2
  webBegin();
  Serial.println(String("Wi-Fi: ") + AP_SSID + "  http://" + WiFi.softAPIP().toString() + "/");
#endif
  Serial.println("Antenna tracker ready");
  lastStep = millis();
}

void loop() {
#if NET_MODE == 2
  web.handleClient();
#endif
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { String r = handleLine(line); if (r.length()) Serial.println(r); line = ""; }
    else if (c != '\r' && line.length() < 64) line += c;
  }

  // UDP-команди по мережі
  if (!udpUp && netUp()) {
    udp.begin(CMD_PORT);
    udpUp = true;
    Serial.printf("UDP listening on %s:%u\n", TRK_IP.toString().c_str(), CMD_PORT);
  }
  if (udpUp) {
    int n = udp.parsePacket();
    if (n > 0) {
      char pkt[96];
      int len = udp.read(pkt, sizeof(pkt) - 1);
      if (len < 0) len = 0;
      pkt[len] = 0;
      String r = handleLine(String(pkt));
      if (r.length()) {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.print(r + "\n");
        udp.endPacket();
      }
    }
  }

  unsigned long now = millis();
  if (now - lastStep >= 20) {                 // шаг 20 мс: плавне наведення
    float dt = (now - lastStep) / 1000.0f;
    lastStep = now;
    float maxStep = MAX_SPEED_DPS * dt;
    float dp = tgtPan - curPan;
    float dt2 = tgtTilt - curTilt;
    curPan  += constrain(dp,  -maxStep, maxStep);
    curTilt += constrain(dt2, -maxStep, maxStep);
    writeServo(PIN_PAN, curPan, PAN_RANGE_DEG);
#if USE_TILT
    writeServo(PIN_TILT, curTilt, 180.0f);
#endif
  }
}
