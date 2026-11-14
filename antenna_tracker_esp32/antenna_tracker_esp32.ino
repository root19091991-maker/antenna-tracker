// ESP32: поворотна платформа антени (пан + тильт, 2 серво). Команди з ПК по USB-serial.
//
// Протокол (115200, рядки закінчуються \n):
//   T,az,el     азимут 0..360 (0 = північ), кут місця 0..90  -> антена наводиться
//   P,pan,tilt  прямі кути серво 0..180 (для калібрування)
//   C           центр: pan=90, tilt=90
//   ?           вивести стан
//
// Механіка (серво на 180°): "через верх". Pan 0..180 покриває півколо азимутів, а коли ціль
// у протилежній півкулі, pan = az-180 і тильт перевертається (tilt = 180 - el).
//   tilt 0   = антена дивиться горизонтально в бік pan
//   tilt 90  = вертикально вгору
//   tilt 180 = горизонтально у протилежний бік
//
// Підключення: сигнал pan -> PIN_PAN, тильт -> PIN_TILT, GND серво -> GND ESP32.
// Серво живіть ОКРЕМО від 5 В (не від ESP32!), мінус спільний з ESP32.

// ---------- Налаштування ----------
#if defined(CONFIG_IDF_TARGET_ESP32C3)
const int PIN_PAN  = 4;
const int PIN_TILT = 5;
#else
const int PIN_PAN  = 18;
const int PIN_TILT = 19;
#endif

const float PAN_ZERO_AZ  = 0.0;     // азимут, у який дивиться антена при pan=0 (0 = північ, 90 = схід)
const bool  PAN_REVERSE  = false;   // true, якщо pan крутиться у зворотний бік
const bool  TILT_REVERSE = false;   // true, якщо тильт крутиться у зворотний бік
const float PAN_TRIM     = 0.0;     // підгонка нуля pan, градуси
const float TILT_TRIM    = 0.0;     // підгонка нуля tilt, градуси
const int   SERVO_MIN_US = 500;     // імпульс для 0 градусів (залежить від серво: 500..1000)
const int   SERVO_MAX_US = 2500;    // імпульс для 180 градусів (2000..2500)
const float MAX_SPEED_DPS = 60.0;   // макс. швидкість серво, градусів/с (плавний рух)
// -----------------------------------

const int PWM_FREQ = 50;
const int PWM_RES  = 14;            // 16384 тіків на період 20000 мкс

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmBegin()                 { ledcAttach(PIN_PAN, PWM_FREQ, PWM_RES); ledcAttach(PIN_TILT, PWM_FREQ, PWM_RES); }
void pwmDuty(int pin, uint32_t d) { ledcWrite(pin, d); }
#else
void pwmBegin()                 { ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_PAN, 0); ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_TILT, 1); }
void pwmDuty(int pin, uint32_t d) { ledcWrite(pin == PIN_PAN ? 0 : 1, d); }
#endif

float curPan = 90, curTilt = 90;      // текуче положення, градуси
float tgtPan = 90, tgtTilt = 90;      // ціль, градуси
String line;
unsigned long lastStep = 0;

void writeServo(int pin, float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f;
  pwmDuty(pin, (uint32_t)(us * 16384.0f / 20000.0f));
}

// az/el (градуси) -> кути серво pan/tilt
void aimToServos(float az, float el, float &pan, float &tilt) {
  el = constrain(el, 0.0f, 90.0f);
  float rel = fmodf(az - PAN_ZERO_AZ, 360.0f);
  if (rel < 0) rel += 360.0f;
  if (rel <= 180.0f) { pan = rel;          tilt = el; }
  else               { pan = rel - 180.0f; tilt = 180.0f - el; }
  if (PAN_REVERSE)  pan  = 180.0f - pan;
  if (TILT_REVERSE) tilt = 180.0f - tilt;
  pan  = constrain(pan + PAN_TRIM, 0.0f, 180.0f);
  tilt = constrain(tilt + TILT_TRIM, 0.0f, 180.0f);
}

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  char cmd = s[0];
  if (cmd == 'T' || cmd == 'P') {
    int c1 = s.indexOf(',');
    int c2 = s.indexOf(',', c1 + 1);
    if (c1 < 0 || c2 < 0) { Serial.println("ERR format"); return; }
    float a = s.substring(c1 + 1, c2).toFloat();
    float b = s.substring(c2 + 1).toFloat();
    if (cmd == 'T') aimToServos(a, b, tgtPan, tgtTilt);
    else { tgtPan = constrain(a, 0.0f, 180.0f); tgtTilt = constrain(b, 0.0f, 180.0f); }
    Serial.printf("OK pan=%.1f tilt=%.1f\n", tgtPan, tgtTilt);
  } else if (cmd == 'C') {
    tgtPan = 90; tgtTilt = 90;
    Serial.println("OK center");
  } else if (cmd == '?') {
    Serial.printf("pan=%.1f->%.1f tilt=%.1f->%.1f\n", curPan, tgtPan, curTilt, tgtTilt);
  } else {
    Serial.println("ERR unknown");
  }
}

void setup() {
  Serial.begin(115200);
  pwmBegin();
  writeServo(PIN_PAN, curPan);
  writeServo(PIN_TILT, curTilt);
  Serial.println("Antenna tracker ready");
  lastStep = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handleLine(line); line = ""; }
    else if (c != '\r' && line.length() < 64) line += c;
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
    writeServo(PIN_PAN, curPan);
    writeServo(PIN_TILT, curTilt);
  }
}
