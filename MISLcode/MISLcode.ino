#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "HT16K33.h"

HT16K33 seg(0x70);
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();
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
    delay(200);
  }
  delay(200);
  alpha4.writeDigitAscii(0, 'M');
  alpha4.writeDigitAscii(1, 'I');
  alpha4.writeDigitAscii(2, 'S');
  alpha4.writeDigitAscii(3, 'L');
  alpha4.writeDisplay();
  delay(400);
  alpha4.writeDigitAscii(0, ' ');
  alpha4.writeDigitAscii(1, 'B');
  alpha4.writeDigitAscii(2, 'Y');
  alpha4.writeDigitAscii(3, ' ');
  alpha4.writeDisplay();
  delay(300);
  alpha4.writeDigitAscii(0, 'G');
  alpha4.writeDigitAscii(1, 'a');
  alpha4.writeDigitAscii(2, 'b');
  alpha4.writeDigitAscii(3, 'e');
  alpha4.writeDisplay();
  delay(400);
  alpha4.writeDigitAscii(0, ' ');
  alpha4.writeDigitAscii(1, ' ');
  alpha4.writeDigitAscii(2, ' ');
  alpha4.writeDigitAscii(3, ' ');
  alpha4.writeDisplay();



  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {



}
