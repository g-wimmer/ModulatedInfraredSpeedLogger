#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "HT16K33.h"

HT16K33 seg(0x70);
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();

const int CARRIER_PIN = 9;

const uint32_t CARRIER_HZ    = 30000;
const uint16_t WINDOW_US     = 200;
const uint8_t  CLEAR_MIN     = 3;       // more than 3 edges means unblocked
const uint8_t  BLOCK_MAX     = 1;       // <=1 edges means spoke is blocking
const uint8_t  BLOCK_CONFIRM = 2;       // consecutive low windows needed to call it blocked
const uint32_t MIN_EVENT_US  = 3000;    // ignore spoke events closer than this (40 mph ~ 6400 us)
const uint32_t TIMEOUT_US    = 1000000; // no spoke seen for a second means stopped
const uint16_t REPORT_MS     = 250;

const float WHEEL_DIAMETER_IN = 7.2;
const uint8_t SPOKES          = 5;
const float INCHES_PER_EVENT  = 3.14159265f * WHEEL_DIAMETER_IN / SPOKES;  //wheel rotation per spoke

uint32_t nextWin;
uint16_t lastCount;
bool     clearState = false;
uint8_t  lowStreak  = 0;
bool     hasEvent   = false;
uint32_t lastEventT = 0;
uint32_t lastDt     = 0;
uint32_t dtSum      = 0;
uint16_t dtN        = 0;
uint32_t lastReportMs;

void setup() {
  const char fringe[] = "    FRINGE     ";
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(38400);
  Serial.println("Starting logo sweep");
  Wire.begin();
  Wire.setClock(100000);
  seg.begin();
  alpha4.begin(0x70);
  seg.setBrightness(1);

  for(int i = 0; i<strlen(fringe) -4; i++){
    alpha4.writeDigitAscii(0, fringe[i]);
    alpha4.writeDigitAscii(1, fringe[i+1]);
    alpha4.writeDigitAscii(2, fringe[i+2]);
    alpha4.writeDigitAscii(3, fringe[i+3]);
    alpha4.writeDisplay();
    delay(180);
  }
  delay(200);
  alpha4.writeDigitAscii(0, 'M');
  alpha4.writeDigitAscii(1, 'I');
  alpha4.writeDigitAscii(2, 'S');
  alpha4.writeDigitAscii(3, 'L');
  alpha4.writeDisplay();
  delay(440);
  alpha4.writeDigitAscii(0, ' ');
  alpha4.writeDigitAscii(1, 'B');
  alpha4.writeDigitAscii(2, 'Y');
  alpha4.writeDigitAscii(3, ' ');
  alpha4.writeDisplay();
  delay(340);
  alpha4.writeDigitAscii(0, 'G');
  alpha4.writeDigitAscii(1, 'a');
  alpha4.writeDigitAscii(2, 'b');
  alpha4.writeDigitAscii(3, 'e');
  alpha4.writeDisplay();
  delay(440);
  alpha4.writeDigitAscii(0, ' ');
  alpha4.writeDigitAscii(1, ' ');
  alpha4.writeDigitAscii(2, ' ');
  alpha4.writeDigitAscii(3, ' ');
  alpha4.writeDisplay();

  pinMode(5, INPUT); //comparator input

  //timer 1 counting edges
  TCCR1A = 0;
  TCCR1B = (1 << CS12) | (1 << CS11) | (1 << CS10);    //external clock on T1, rising edge
  TCNT1 = 0;
  Serial.println("Starting Carrier Frequency");
  pinMode(3,OUTPUT);
  //
  TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << WGM22) | (1 << CS21);
  OCR2A = (uint8_t)(F_CPU / 8UL / CARRIER_HZ - 1);
  OCR2B = OCR2A / 2;

  nextWin = micros() + WINDOW_US;
  lastCount = TCNT1;
  lastReportMs = millis();
  digitalWrite(LED_BUILTIN, LOW);
}

void spokeEvent(uint32_t t) {
  if (hasEvent) {
    uint32_t dt = t - lastEventT;
    if (dt < MIN_EVENT_US) return;      //too soon to be a real spoke
    lastDt = dt;
    dtSum += dt;
    dtN++;
  }
  hasEvent = true;
  lastEventT = t;
}

void processWindow(uint8_t n, uint32_t t) {
  if (n >= CLEAR_MIN) {
    clearState = true;
    lowStreak = 0;
  } else if (n <= BLOCK_MAX) {
    if (clearState && ++lowStreak >= BLOCK_CONFIRM) {
      clearState = false;
      lowStreak = 0;
      spokeEvent(t);                    //carrier just disappeared: spoke leading edge
    }
  } else {
    lowStreak = 0;                      //exactly 2 edges: ambiguous, hold state
  }
}

void report() {
  float mph = 0;
  if (hasEvent && (dtN || lastDt)) {
    uint32_t since = micros() - lastEventT;
    float dt = dtN ? (float)dtSum / dtN : (float)lastDt;
    if (since > dt) dt = since;         // speed can't exceed spacing / time since last spoke
    if (since <= TIMEOUT_US && dt > 0) mph = INCHES_PER_EVENT / (dt * 1e-6f) / 17.6f;
  }
  dtSum = 0;
  dtN = 0;
  char buf[8];
  dtostrf(mph, 4, 1, buf);
  if (buf[0] == ' ') buf[0] = '0';
  alpha4.writeDigitAscii(0, ' ');
  alpha4.writeDigitAscii(1, buf[0]);
  alpha4.writeDigitAscii(2, buf[1], true);
  alpha4.writeDigitAscii(3, buf[3]);
  alpha4.writeDisplay();
  Serial.print(F("mph: "));
  Serial.println(mph, 1);
  

}


void loop() {
  int32_t late = (int32_t)(micros() - nextWin);
  if (late >= 0) {
    if (late > (int32_t)WINDOW_US) {    // fell behind (e.g. during a Serial print): resync
      nextWin = micros() + WINDOW_US;
      lastCount = TCNT1;
      return;
    }
    uint32_t winEnd = nextWin;
    nextWin += WINDOW_US;
    uint16_t now = TCNT1;
    uint16_t d = now - lastCount;
    lastCount = now;
    uint8_t n = (d > 255) ? 255 : (uint8_t)d;
    processWindow(n, winEnd);
  }

  if (millis() - lastReportMs >= REPORT_MS) {
    lastReportMs += REPORT_MS;
    report();
  }
}
