#include <SoftwareSerial.h>

#define PIN_BLUETX 2 // 말 그대로 블루투스 TX
#define PIN_BLUERX 3 // 말 그대로 블루투스 RX

SoftwareSerial btSerial(PIN_BLUETX, PIN_BLUERX);

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
}

void loop() {
 if (btSerial.available()) Serial.write(btSerial.read());
 if (Serial.available()) btSerial.write(Serial.read());
}
