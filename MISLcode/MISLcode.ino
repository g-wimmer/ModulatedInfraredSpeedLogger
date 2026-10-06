#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "HT16K33.h"

HT16K33 seg(0x70);
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();
const int CARRIER_PIN = 9;
const uint32_t CARRIER_HZ  = 30000;
const uint16_t WINDOW_US   = 200;
const uint16_t REPORT_MS   = 5000;

uint16_t hist[16];

void setup() {
  const char fringe[] = "    FRINGE     ";
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
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
  TCCR1A = 0;
  TCCR1B = (1 << CS12) | (1 << CS11) | (1 << CS10);    //external clock on T1, rising edge
  TCNT1 = 0;
  Serial.println("Starting Carrier Frequency");
  tone(CARRIER_PIN, CARRIER_HZ); //start the ir pulse at 30khz
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

  memset(hist, 0, sizeof(hist));
  uint16_t last = TCNT1;
  uint32_t nextWin = micros() + WINDOW_US;
  uint32_t endMs = millis() + REPORT_MS;

  while ((int32_t)(millis() - endMs) < 0) {
    int32_t late = (int32_t)(micros() - nextWin);
    if (late >= 0) {
      if (late > WINDOW_US) {                          //fell behind resync, skip this sample
        nextWin = micros() + WINDOW_US;
        last = TCNT1;
        continue;
      }
      nextWin += WINDOW_US;
      uint16_t now = TCNT1;
      uint16_t d = now - last;
      last = now;
      if (d > 15) d = 15;
      hist[d]++;
    }
  }

  Serial.println(F("edges/window : windows"));
  for (uint8_t i = 0; i < 16; i++) {
    Serial.print(i);
    Serial.print(i == 15 ? F("+ : ") : F("  : "));
    Serial.println(hist[i]);
  }
  Serial.println();

}
