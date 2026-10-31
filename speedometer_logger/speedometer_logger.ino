// Modulated Infrared Speed Logger
// Timer0 note: the Arduino core runs Timer0 in fast-PWM mode, where OCR0A is double-buffered
// and can only fire once per overflow. setup() switches Timer0 to normal mode so OCR0A can be
// advanced inside the ISR. The overflow period (millis/micros/delay) is unchanged, but
// analogWrite() on D5/D6 will no longer work (this sketch doesn't use it).


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "SdFat.h"
#include <util/atomic.h>

const uint8_t PIN_BUTTON     = 2;
const uint8_t PIN_CARRIER    = 3;    // fixed by hardware (Timer2 OC2B)
const uint8_t PIN_COMPARATOR = 5;    // fixed by hardware (Timer1 T1)
const uint8_t PIN_SD_CS      = 10;


//Set to a speed (ex 40) to have the interrupt simulate a wheel spinning at higher speeds. 
#define SIM_MPH 0


const uint32_t CARRIER_HZ    = 30000;
const uint16_t WINDOW_US     = 200;
static_assert(WINDOW_US == 200, "log timestamps assume 200 us ticks");
const uint8_t  CLEAR_MIN     = 3;       // >= 3 edges in a window  -> carrier present (beam clear)
const uint8_t  BLOCK_MAX     = 1;       // <= 1 edge in a window   -> carrier absent (spoke)
const uint8_t  BLOCK_CONFIRM = 2;       // consecutive low windows needed to call it blocked
const uint32_t MIN_EVENT_US  = 3000;    // ignore spoke events closer than this (40 mph ~ 6400 us)
const uint32_t TIMEOUT_US    = 1000000; // no spoke for a second -> stopped

//200 us expressed in Timer0 ticks (25 at 8 MHz with the core's /64 prescaler)
const uint8_t  WINDOW_TICKS      = (uint8_t)((uint32_t)WINDOW_US * (F_CPU / 1000000UL) / 64UL);
const uint32_t MIN_EVENT_TICKS   = MIN_EVENT_US / WINDOW_US;
const uint32_t TIMEOUT_TICKS     = TIMEOUT_US / WINDOW_US;

//Display-speed plausibility filter. A spoke interval shorter than this percentage of the previous
//interval (a >2.5x speed-up in one spoke) is treated as the beam flickering at a spoke edge
//(common at very low speeds) and ignored for speed. It is still logged, flagged used=0.
//After REJECT_LIMIT rejects in a row the filter assumes the wheel really did speed up and resyncs.
const uint8_t  MIN_INTERVAL_PCT = 40;
const uint8_t  REJECT_LIMIT     = 3;

//wheel dimensions
const float   WHEEL_DIAMETER_IN = 7.2;
const uint8_t SPOKES            = 5;
const float   INCHES_PER_EVENT  = 3.14159265f * WHEEL_DIAMETER_IN / SPOKES;  // rim travel per spoke

#if SIM_MPH > 0
const float    SENSOR_RADIUS_IN = 2.1;//axle to beam
const float    SPOKE_WIDTH_IN   = 0.6;//spoke width at the beam
const float    SIM_V_IPS        = SIM_MPH * 17.6f;//rim speed, inches per second
const uint32_t SIM_PERIOD_US    = (uint32_t)(INCHES_PER_EVENT / SIM_V_IPS * 1e6f);
const uint32_t SIM_ABSENT_US    = (uint32_t)((SPOKE_WIDTH_IN + 0.03f) / (SIM_V_IPS * SENSOR_RADIUS_IN / (WHEEL_DIAMETER_IN / 2)) * 1e6f);
uint32_t simPhase = 0;
#endif

//Display timing
const uint16_t REPORT_MS   = 250;
const uint16_t SYNC_MS     = 2000;//flush the SD file this often while recording
const uint16_t TOP_HOLD_MS = 10000;// how long the high top speed is displayed
const uint16_t DEBOUNCE_MS = 250;

//ISR <-> main shared state
const uint8_t EV_BUF = 64;//must be a power of two
volatile uint32_t winTicks = 0; //200 us ticks since boot
volatile uint32_t evBuf[EV_BUF];
volatile uint8_t  evHead = 0;
volatile uint8_t  evTail = 0;
volatile uint16_t evDropped = 0;

// ISR-private state
uint16_t lastCount;
bool     clearState = false;
uint8_t  lowStreak  = 0;
bool     isrHasEvent = false;
uint32_t isrLastEvent = 0;

ISR(TIMER0_COMPA_vect) {
  OCR0A += WINDOW_TICKS; //next sample exactly one window later
  winTicks++;

  uint16_t now = TCNT1;
  uint16_t d = now - lastCount;
  lastCount = now;
  uint8_t n = (d > 255) ? 255 : (uint8_t)d;

  if (n >= CLEAR_MIN) {
    clearState = true;
    lowStreak = 0;
  } else if (n <= BLOCK_MAX) {
    if (clearState && ++lowStreak >= BLOCK_CONFIRM) {
      clearState = false;
      lowStreak = 0;
      uint32_t t = winTicks;  //carrier just disappeared: spoke leading edge
      if (!isrHasEvent || (t - isrLastEvent) >= MIN_EVENT_TICKS) {
        isrHasEvent = true;
        isrLastEvent = t;
        uint8_t next = (evHead + 1) & (EV_BUF - 1);
        if (next != evTail) {
          evBuf[evHead] = t;
          evHead = next;
        } else {
          evDropped++;
        }
      }
    }
  } else {
    lowStreak = 0;   //exactly 2 edges: ambiguous, hold state
  }

#if SIM_MPH > 0
  simPhase += WINDOW_US;
  if (simPhase >= SIM_PERIOD_US) simPhase -= SIM_PERIOD_US;
  if (simPhase < SIM_ABSENT_US) {
    TCCR2A &= ~(1 << COM2B1);//carrier off, hold D3 low
    PORTD &= ~(1 << 3);
  } else {
    TCCR2A |= (1 << COM2B1); //carrier on
  }
#endif
}

bool popEvent(uint32_t &t) {
  if (evTail == evHead) return false;
  t = evBuf[evTail];
  evTail = (evTail + 1) & (EV_BUF - 1);
  return true;
}

uint32_t ticksNow() {
  uint32_t t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = winTicks; }
  return t;
}

// button
volatile bool btnFlag = false;

void onButton() {
  static uint32_t last = 0;
  uint32_t ms = millis();
  if (ms - last >= DEBOUNCE_MS) {
    last = ms;
    btnFlag = true;
  }
}

//display
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();

void showText(const char *s) {
  for (uint8_t i = 0; i < 4; i++) alpha4.writeDigitAscii(i, s[i]);
  alpha4.writeDisplay();
}

void showSpeed(float mph, bool rec) {
  if (mph > 99.9f) mph = 99.9f;
  char buf[8];
  dtostrf(mph, 4, 1, buf);
  if (buf[0] == ' ') buf[0] = '0';
  alpha4.writeDigitAscii(0, rec ? 'R' : ' ');   // 'R' = recording
  alpha4.writeDigitAscii(1, buf[0]);
  alpha4.writeDigitAscii(2, buf[1], true);
  alpha4.writeDigitAscii(3, buf[3]);
  alpha4.writeDisplay();
}

//SD
SdFat32 sd;
File32  file;
bool     sdOk = false;
uint16_t runNum = 1;

enum Mode : uint8_t { IDLE, RECORDING, SHOW_TOP };
Mode     mode = IDLE;
uint32_t sessionStart = 0;       // ticks
uint32_t spokeIdx = 0;
uint32_t prevRel = 0;
bool     hasPrevRel = false;
float    topMph = 0;
uint32_t topShownAt = 0;
uint32_t noticeUntil = 0;
const char *noticeText = "    ";

bool initSd() {
  return sd.begin(SdSpiConfig(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(4)));
}

//1 tick = 200 us, printed as milliseconds with one decimal (e.g. 1234.6)
void printTicksMs(Print &p, uint32_t ticks) {
  p.print(ticks / 5UL);
  p.write('.');
  p.print((uint8_t)((ticks % 5UL) * 2));
}

bool startRecording() {
  if (!sdOk) sdOk = initSd();
  if (!sdOk) return false;

  char name[13];
  while (runNum < 1000) {
    snprintf(name, sizeof(name), "RUN%03u.CSV", (unsigned)runNum);
    if (!sd.exists(name)) break;
    runNum++;
  }
  if (runNum >= 1000) return false; //card is likely full, or something went wrong
  if (!file.open(name, O_WRONLY | O_CREAT | O_EXCL)) {
    sdOk = false;   //card probably removed: re-init on next press
    return false;
  }
  file.println(F("spoke,time_ms,dt_ms,used"));
  runNum++;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    evTail = evHead;   //discard events from before the session
    sessionStart = winTicks;
  }
  spokeIdx = 0;
  hasPrevRel = false;
  topMph = 0;
  return true;
}

void stopRecording() {
  file.close();     //close() also syncs
}

//Writes a spoke event to the file
void logEvent(uint32_t t, bool used) {
  uint32_t rel = t - sessionStart;
  file.print(++spokeIdx);
  file.write(',');
  printTicksMs(file, rel);
  file.write(',');
  if (hasPrevRel) printTicksMs(file, rel - prevRel);
  file.write(',');
  file.write(used ? '1' : '0');
  file.write('\n');
  prevRel = rel;
  hasPrevRel = true;
}


bool     hasEv = false;
uint32_t lastEvTicks = 0;
uint32_t lastDtTicks = 0;
uint32_t dtSumTicks = 0;
uint16_t dtN = 0;

uint8_t rejectStreak = 0;

void handleSpoke(uint32_t t) {
  bool used = true;
  bool resync = false;
  uint32_t dt = hasEv ? (t - lastEvTicks) : 0;

  if (!hasEv || dt > TIMEOUT_TICKS) { 
    resync = true; // first spoke, or restarting after a stop
  } else if (lastDtTicks && dt * 100UL < lastDtTicks * MIN_INTERVAL_PCT) {//checks to see if the wheel's speedup is unrealistically fast
    if (++rejectStreak >= REJECT_LIMIT){
      resync = true;
    }   //wheel really did speed up
    else used = false; //probably edge flicker
  } else {
    rejectStreak = 0;
  }

  if (resync) {
    lastDtTicks = 0;
    rejectStreak = 0;
  } else if (used) {
    lastDtTicks = dt;
    dtSumTicks += dt;
    dtN++;
  }

  if (used) {
    hasEv = true;
    lastEvTicks = t;
  }
  if (mode == RECORDING && (int32_t)(t - sessionStart) >= 0){
    logEvent(t, used);
  }
}

float currentMph() {
  float mph = 0;
  if (hasEv && (dtN || lastDtTicks)) {
    uint32_t sinceTicks = ticksNow() - lastEvTicks;
    if (sinceTicks <= TIMEOUT_TICKS) {
      float dtTicks = dtN ? (float)dtSumTicks / dtN : (float)lastDtTicks;
      if (sinceTicks > dtTicks) dtTicks = sinceTicks;   //can't be faster than spacing / time since last spoke
      mph = INCHES_PER_EVENT / (dtTicks * WINDOW_US * 1e-6f) / 17.6f;
    }
  }
  dtSumTicks = 0;
  dtN = 0;
  return mph;
}

void onButtonPress() {
  uint32_t ms = millis();
  if (mode == RECORDING) {
    stopRecording();
    mode = SHOW_TOP;//display top speed of recording
    topShownAt = ms;
  } else if (startRecording()) {
    mode = RECORDING;
  } else {
    noticeText = "NOSD";
    noticeUntil = ms + 1200;
  }
}

void updateDisplay(float mph, uint32_t ms) {
  if (ms < noticeUntil) {
    showText(noticeText);
  } else if (mode == SHOW_TOP) {
    if (ms - topShownAt >= TOP_HOLD_MS) {
      mode = IDLE;
      showSpeed(mph, false);
    } else if (((ms - topShownAt) / 1000UL) & 1) {
      showSpeed(topMph, false);
    } else {
      showText("HI  ");
    }
  } else {
    showSpeed(mph, mode == RECORDING);
  }
}


uint32_t lastReportMs = 0;
uint32_t lastSyncMs = 0;

void setup() {
  const char fringe[] = "    FRINGE     ";
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(38400);
  Serial.println(F("Starting logo sweep"));
  Wire.begin();
  Wire.setClock(100000);
  alpha4.begin(0x70);
  alpha4.setBrightness(1);

  for (uint8_t i = 0; i < strlen(fringe) - 4; i++) {
    showText(fringe + i);
    delay(180);
  }
  delay(200);
  showText("MISL");
  delay(440);
  showText(" BY ");
  delay(340);
  showText("Gabe");
  delay(440);
  showText("    ");

  sdOk = initSd();
  if (!sdOk) {
    Serial.println(F("SD init failed"));
    showText("NOSD");
    delay(1200);
    showText("    ");
  }

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), onButton, FALLING);

  //Timer1 counting rising edges with hardwire counter
  pinMode(PIN_COMPARATOR, INPUT);
  TCCR1A = 0;
  TCCR1B = (1 << CS12) | (1 << CS11) | (1 << CS10);
  TCNT1 = 0;

  //Timer2 fast PWM, TOP = OCR2A, output on OC2B (D3), prescaler 8
  Serial.println(F("Starting carrier"));
  pinMode(PIN_CARRIER, OUTPUT);
  TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << WGM22) | (1 << CS21);
  OCR2A = (uint8_t)(F_CPU / 8UL / CARRIER_HZ - 1);
  OCR2B = OCR2A / 2;

  //Timer0 normal mode + compare-match A every WINDOW_TICKS
  TCCR0A &= ~((1 << WGM01) | (1 << WGM00));
  lastCount = TCNT1;
  OCR0A = TCNT0 + WINDOW_TICKS;
  TIFR0 = (1 << OCF0A);
  TIMSK0 |= (1 << OCIE0A);

  lastReportMs = millis();
  lastSyncMs = lastReportMs;
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  uint32_t t;
  while (popEvent(t)) handleSpoke(t);

  if (btnFlag) {
    btnFlag = false;
    onButtonPress();
  }

  uint32_t ms = millis();

  if (mode == RECORDING && ms - lastSyncMs >= SYNC_MS) {
    lastSyncMs = ms;
    file.sync();
  }

  if (ms - lastReportMs >= REPORT_MS) {//update display every 250ms
    lastReportMs = ms;
    float mph = currentMph();
    if (mode == RECORDING && mph > topMph) topMph = mph;
    updateDisplay(mph, ms);
    Serial.print(F("mph: "));
    Serial.println(mph, 1);
    if (evDropped) {
      Serial.print(F("dropped events: "));
      Serial.println(evDropped);
    }
  }
}
