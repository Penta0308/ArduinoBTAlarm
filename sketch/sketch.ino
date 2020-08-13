#include <Servo.h>
#include <TimeLib.h>
#include <DS1302.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define PIN_BLUETX 2
#define PIN_BLUERX 3

#define PIN_RTC_CE 8
#define PIN_RTC_IO 7
#define PIN_RTC_SCLK 6

#define PIN_SERVO 9

#define PIN_SWITCH_ON 12
#define PIN_SWITCH_OFF 13

#define EEP_SERVO_ON 0x00
#define EEP_SERVO_OFF 0x01

/*
 * enum Day {
    kSunday    = 1,
    kMonday    = 2,
    kTuesday   = 3,
    kWednesday = 4,
    kThursday  = 5,
    kFriday    = 6,
    kSaturday  = 7
  };
 */

SoftwareSerial btSerial(PIN_BLUETX, PIN_BLUERX);
DS1302 rtc(PIN_RTC_CE, PIN_RTC_IO, PIN_RTC_SCLK);
Servo servo;

byte command = 0;
char * arglinebuf;
time_t alarmtime = 0;

void setup() {
  Serial.begin(9600); //시리얼모니터
  btSerial.begin(9600); //블루투스 시리얼
  pinMode(PIN_SWITCH_ON, INPUT_PULLUP);
  pinMode(PIN_SWITCH_OFF, INPUT_PULLUP);
  delay(500);
  servo.attach(9);
  delay(500);
  servo.write(EEPROM.read(EEP_SERVO_ON));
  btSerial.write("AT+NAME=ALARM");
  arglinebuf = new char(64);
}

void loop() {
  char arglinebufPos = 0;
  if (btSerial.available()) {
    byte b = btSerial.read();
    Serial.write(btSerial.read());//블루투스측 내용을 시리얼모니터에 출력
    if (command == 0) command = b;
    else if (arglinebufPos < 64 && b != '\n') {
      arglinebuf[arglinebufPos++] = b;
    }
    if (b == '\n') {
      arglinebuf[arglinebufPos] = '\0';
      //DO command
      switch(command) {
        case 's': { //SET EEP : sADDR(4-digit Decimal)DATA(3-digit Decimal)
          byte eepdata = atoi(arglinebuf + 5);
          arglinebuf[4] = '\0';
          short eepaddr = atoi(arglinebuf);
          EEPROM.update(eepaddr, eepdata);
          break;
        }
        case 'e': { //EVALUATE SERVO : eANGLE
          servo.write(atoi(arglinebuf));
          break;
        }
        case 'a': { //ALARM SET : aUTCSTAMP
          alarmtime = atol(arglinebuf);
          break;
        }
        case 'r': { //RTC SET : rUTCSTAMP
          Time * t = parseTimeStamp(atol(arglinebuf));
          rtc.writeProtect(false);
          rtc.halt(false);
          rtc.time(*t);
          delete t;
          break;
        }
      }
      command = 0;
      return;
    }
  }
  if (Serial.available()) {
    btSerial.write(Serial.read());//시리얼모니터 내용을 블루추스 측에 출력
  }
  Time _temptime = rtc.time();
  if(alarmtime > getTimeStamp(&_temptime)) { // YEAHHH
    
  }
}

/*Time * parseTime(char * arglinebuf) {
  short* rtcdata = new short(7); // YEAR MONTH DATE HOUR MIN SEC DAY
  rtcdata[0] = atoi(strtok(arglinebuf, ","));
  for(byte n = 1; n < 7; n++) rtcdata[n] = atoi(strtok(NULL, ","));
  Time* t = new Time(rtcdata[0], rtcdata[1], rtcdata[2], rtcdata[3], rtcdata[4], rtcdata[5], (Time::Day)rtcdata[6]);
  delete rtcdata;
}*/

time_t getTimeStamp(Time * t) {
  tmElements_t te;
  te.Year = t->yr - 1970;
  te.Month = t->mon;
  te.Day = t->date;
  te.Hour = t->hr;
  te.Minute = t->min;
  te.Second = t->sec;
  return makeTime(te);
}

Time * parseTimeStamp(time_t ts) {
  tmElements_t te;
  breakTime(ts, te);
  return new Time(te.Year + 1970, te.Month, te.Day, te.Hour, te.Minute, te.Second, Time::Day::kSunday);
}
