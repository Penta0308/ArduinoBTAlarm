//#define DEBUG

#include <NeoSWSerial.h>
#include <TimeLib.h>
#include <DS1302.h>
#include <EEPROM.h>

#define PIN_BLUETX 2 // 말 그대로 블루투스 TX
#define PIN_BLUERX 3 // 말 그대로 블루투스 RX
// AT+BAUD1 1200bps

#define PIN_RTC_CE 8
#define PIN_RTC_IO 7
#define PIN_RTC_SCLK 6

#define PIN_SERVO_PULSE 9

#define PIN_SWITCH_ON 5
#define PIN_SWITCH_OFF 4
#define PIN_SWITCH_PULSE 10

#define EEP_SERVO_ON 0x00
#define EEP_SERVO_OFF 0x01
#define EEP_SERVO_OFFSET_us 0x02

#define SERVO_GRADUALSTEPCOUNT 10 // 50/sec
#define SERVO_PULSECOUNT 30 // 50/sec
const float SERVO_ANGLEMUL = 5.5555553436279296875f;

#define CTOUCH_PULSEDELAY_us 400
#define CTOUCH_READDELAY_us 100

#define BUFFERLEN_ARGLINE 64
#define BUFFERLEN_COMMANDSET 8
#define BUFFERLEN_ALARMSET 10

NeoSWSerial btSerial(PIN_BLUETX, PIN_BLUERX);
DS1302 rtc(PIN_RTC_CE, PIN_RTC_IO, PIN_RTC_SCLK);

typedef union _u8i8 {
  uint8_t u;
  int8_t i;
} u8i8;

typedef class _CommandSet {
private:
  byte arglinebufsize;
public:
  _CommandSet(const char _command, const char* _arglinebuf);
  ~_CommandSet();
  char command;
  char* arglinebuf;
} CommandSet;

_CommandSet::_CommandSet(const char _command, const char* _arglinebuf) {
  command = _command;
  arglinebufsize = strlen(_arglinebuf) + 1;
  arglinebuf = (char*)malloc(arglinebufsize);
  memcpy(arglinebuf, _arglinebuf, arglinebufsize);
  #ifdef DEBUG
  Serial.println(arglinebufsize, DEC);
  Serial.println(_arglinebuf);
  Serial.println(arglinebuf);
  #endif
}

_CommandSet::~_CommandSet() {
  free(arglinebuf);
}

typedef class _AlarmSet {
private:
  byte commandSetWorking;
  CommandSet** commandSetList;
  byte commandSetCount;
public:
  _AlarmSet(time_t _alarmtime);
  ~_AlarmSet();
  void push(CommandSet* _commandSet);
  CommandSet* pop();
  time_t alarmtime = 0;
  boolean popped;
} AlarmSet;

_AlarmSet::_AlarmSet(time_t _alarmtime) {
  popped = false;
  commandSetWorking = 0;
  commandSetCount = 0;
  alarmtime = _alarmtime;
  commandSetList = (CommandSet**)malloc(sizeof(CommandSet*) * BUFFERLEN_COMMANDSET);
}

_AlarmSet::~_AlarmSet() {
  for(; commandSetWorking < commandSetCount; commandSetWorking++)
    delete commandSetList[commandSetWorking];
  free(commandSetList);
}

void _AlarmSet::push(CommandSet* _commandSet) {
  commandSetList[commandSetCount] = _commandSet;
  #ifdef DEBUG
  Serial.print("PUSHING");
  Serial.println(commandSetCount, DEC);
  #endif
  commandSetCount++;
}

CommandSet* _AlarmSet::pop() {
  popped = true;
  #ifdef DEBUG
  Serial.print("POPPING");
  Serial.println(commandSetWorking, DEC);
  #endif
  if(commandSetWorking >= commandSetCount) return NULL;
  CommandSet* ret = commandSetList[commandSetWorking];
  commandSetWorking++;
  #ifdef DEBUG
  Serial.println("POPPED");
  #endif
  return ret;
}

boolean CTouch_read(byte sensepin, byte pulsepin) { // SENSE: INPUT, PULSE: OUTPUT
  //pinMode(pulsepin, OUTPUT);
  digitalWrite(pulsepin, HIGH);
  delayMicroseconds(CTOUCH_PULSEDELAY_us);
  digitalWrite(pulsepin, LOW);
  delayMicroseconds(CTOUCH_READDELAY_us);
  return digitalRead(sensepin);
}

typedef class _StaticServo {
private:
  byte pin = -1;
  short nd = -1;
public:
  void attach(byte _pin);
  short write(float _angle);
  short write(float _angle, byte _count);
} StaticServo;

u8i8 SERVO_OFFSET_us;

void _StaticServo::attach(byte _pin) {
  pin = _pin;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

short _StaticServo::write(float _angle) { return write(_angle, 1); }

short _StaticServo::write(float _angle, byte _count) {
  noInterrupts();
  pinMode(PIN_SWITCH_ON, OUTPUT);
  pinMode(PIN_SWITCH_OFF, OUTPUT);
  short td = (short)(_angle * SERVO_ANGLEMUL) + 1000 + SERVO_OFFSET_us.i;
  byte sc = 0;
staticservowriteloop:
  //btSerial.println(td, DEC);
  digitalWrite(pin, HIGH);
  delayMicroseconds(td);
  digitalWrite(pin, LOW);
  delayMicroseconds(16000);
  delayMicroseconds(4000 - td);
  if(++sc == _count) {
    nd = td;
    interrupts();
    pinMode(PIN_SWITCH_ON, INPUT);
    pinMode(PIN_SWITCH_OFF, INPUT);
    return nd;
  }
  goto staticservowriteloop;
}

time_t getTimeStamp(Time* t) {
  tmElements_t te;
  te.Year = t->yr;
  te.Month = t->mon;
  te.Day = t->date;
  te.Hour = t->hr;
  te.Minute = t->min;
  te.Second = t->sec;
  return makeTime(te);
}

Time* parseTimeStamp(time_t ts) {
  tmElements_t te;
  breakTime(ts, te);
  return new Time(te.Year, te.Month, te.Day, te.Hour, te.Minute, te.Second, Time::Day::kSunday);
}

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
/*
 * Commands:
 *  sADDR(4-digit Decimal)DATA(3-digit Decimal) : EEP WRITE
 *  eANGLE(Decimal) : SERVO SET
 *  en              : SERVO ON  : EEP {EEP_SERVO_ON}
 *  ef              : SERVO OFF : EEP {EEP_SERVO_OFF}
 *  tUTCSTAMP : RTC SET
 *  g : RTC GET
 *  rALARMID(1-digit Decimal, 0~9)DeltaUTCSTAMP : ALARM SET - RECORDING ON
 *  f : RECORDING OFF
 *  dSECOND : Async Pause Action
 *  qMS : Sync Pause Action
 */

StaticServo servo;

int8_t recording = -1;
uint8_t command = 0;
char arglinebuf[BUFFERLEN_ARGLINE];
AlarmSet* alarmSetList[BUFFERLEN_ALARMSET];

time_t timestampoffset = 0;
time_t lastrefresh = 0;

uint8_t arglinebufPos = 0;

uint8_t sensedtct = 0;

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  pinMode(PIN_SWITCH_ON, INPUT);
  pinMode(PIN_SWITCH_OFF, INPUT);
  pinMode(PIN_SWITCH_PULSE, OUTPUT);
  delay(500);
  servo.attach(9);
  //arglinebuf = new char(BUFFERLEN_ARGLINE);
  //alarmSetList = malloc(sizeof(AlarmSet*) * 10);
  memset(alarmSetList, 0x0, 10);
  SERVO_OFFSET_us.u = EEPROM.read(EEP_SERVO_OFFSET_us);
  Time _temptime = rtc.time();
  timestampoffset = getTimeStamp(&_temptime);
  #ifdef DEBUG
  Serial.println("SETUP END");
  #endif
}

uint8_t beforeswitchstat = 0;
uint8_t switchstat = 0;

void loop() {
  #ifdef DEBUG
  //Serial.println("l");
  #endif
  if(millis() - lastrefresh > 3600000) { // 60Min.
    Time _temptime = rtc.time();
    time_t temptimestamp = getTimeStamp(&_temptime);
    timestampoffset = temptimestamp - millis() / 1000;
    lastrefresh = millis();
    #ifdef DEBUG
    Serial.println("REFRESHING RTC");
    #endif
  }
  time_t timestamp = timestampoffset + millis() / 1000;
  for(uint8_t i = 0; i < BUFFERLEN_ALARMSET; i++) {
    if(alarmSetList[i]) {
      if(alarmSetList[i]->alarmtime < timestamp) { // Trigger Alarm i
        #ifdef DEBUG
        Serial.print("TRIGGERED");
        Serial.println(i, DEC);
	      #endif
	      CommandSet* p = alarmSetList[i]->pop();
        #ifdef DEBUG
        Serial.println((int)p, DEC);
        #endif
  		  if(p) { // DO IT
          char command = p->command;
          char* arglinebuf = p->arglinebuf;
          #ifdef DEBUG
          Serial.print(command);
          Serial.println(arglinebuf);
          #endif
          if(command == 'd') alarmSetList[i]->alarmtime += atol(arglinebuf);
          else doLine(command, arglinebuf, false);
          delete p;
        } else { // FREE THEM
          #ifdef DEBUG
          Serial.print("FREEING");
          Serial.println(i, DEC);
          #endif
          delete alarmSetList[i];
          alarmSetList[i] = 0;
          #ifdef DEBUG
          Serial.print("FREED");
          #endif
        }
        #ifdef DEBUG
        //Serial.println("n");
        #endif
      }
    }
  }
  if(sensedtct) { // 1
    sensedtct = 0;
    if(CTouch_read(PIN_SWITCH_ON, PIN_SWITCH_PULSE)) switchstat = 0b00000001;
  } else { // 0
    sensedtct = 1;
    if(CTouch_read(PIN_SWITCH_OFF, PIN_SWITCH_PULSE)) switchstat |= 0b00000010;
  }

  if(switchstat) {
    if(beforeswitchstat == 0) {
      if(switchstat == 0b00000001) { // ON
        byte b = EEPROM.read(EEP_SERVO_ON);
        //btSerial.println(b, DEC);
        servo.write(b, SERVO_PULSECOUNT);
        //delay(100);
      } else if(switchstat == 0b00000010) { // OFF
        byte b = EEPROM.read(EEP_SERVO_OFF);
        //btSerial.println(b, DEC);
        servo.write(b, SERVO_PULSECOUNT);
        //delay(100);
      }
    }
    for(uint8_t i = 0; i < BUFFERLEN_ALARMSET; i++) {
      if(alarmSetList[i]) {
        if(alarmSetList[i]->popped) {
          #ifdef DEBUG
          Serial.print("ALARMINT");
          Serial.println(i, DEC);
          #endif
          delete alarmSetList[i];
		      alarmSetList[i] = 0;
        }
      }
    }
  }

  switchstat = beforeswitchstat;
  
  if (btSerial.available()) {
    byte b = btSerial.read();
    Serial.write(b);//블루투스측 내용을 시리얼모니터에 출력
    if (command == 0) command = b;
    else if (arglinebufPos < BUFFERLEN_ARGLINE && b != '\n') {
      arglinebuf[arglinebufPos] = b;
      arglinebufPos++;
    }else if (b == '\n' && command != 0) {
      arglinebuf[arglinebufPos] = '\0';
      arglinebufPos = 0;
      //DO command
      if (command == '+' || command == 'O') goto btlineworkend;
      else if(command == 'f') {
        //btSerial.println(alarmSetList[recording]->commandSetCount, DEC);
        recording = -1;
        btSerial.println("OK");
      } else if(command == 'r') { //ALARM SET: rALARMID(1-digit Decimal, 0~9)UTCSTAMP
        time_t alarmtime = atol(arglinebuf + 1) + timestampoffset + millis() / 1000;
        arglinebuf[1] = '\0';
        recording = atoi(arglinebuf);
        if(alarmSetList[recording]) delete alarmSetList[recording];
        alarmSetList[recording] = new AlarmSet(alarmtime);
        #ifdef DEBUG
        Serial.print("RECORDING");
        Serial.println(recording, DEC);
        //Serial.println(alarmSetList[recording]->alarmtime - getTimeStamp(&_temptime), DEC);
        #endif
        btSerial.println("OK");
      } else if (recording != -1)
        alarmSetList[recording]->push(new CommandSet(command, arglinebuf));
      else doLine(command, arglinebuf, true);
btlineworkend:
      command = 0;
    }
  }
  if (Serial.available()) {
    btSerial.write(Serial.read());//시리얼모니터 내용을 블루추스 측에 출력
  }
}

/*Time * parseTime(char * arglinebuf) {
  short* rtcdata = new short(7); // YEAR MONTH DATE HOUR MIN SEC DAY
  rtcdata[0] = atoi(strtok(arglinebuf, ","));
  for(byte n = 1; n < 7; n++) rtcdata[n] = atoi(strtok(NULL, ","));
  Time* t = new Time(rtcdata[0], rtcdata[1], rtcdata[2], rtcdata[3], rtcdata[4], rtcdata[5], (Time::Day)rtcdata[6]);
  delete rtcdata;
}*/

void doLine(const char command, char* arglinebuf, boolean replying) {
  if(command == 's') { //SET EEP : sADDR(4-digit Decimal)DATA(3-digit Decimal)
    byte eepdata = atoi(arglinebuf + 4);
    arglinebuf[4] = '\0';
    short eepaddr = atoi(arglinebuf);
    if(replying) btSerial.println(eepaddr, DEC);
    if(replying) btSerial.println(eepdata, DEC);
    EEPROM.update(eepaddr, eepdata);
    if(eepdata == EEPROM.read(eepaddr)) if(replying) btSerial.write("OK");
    else if(replying)btSerial.write("ERROR");
  } else if(command == 'e') { //EVALUATE SERVO : eANGLE
    if(isdigit(arglinebuf[0])) {
      float angle = atof(arglinebuf);
      short sv = servo.write(angle, SERVO_PULSECOUNT);
      if(replying) btSerial.println(sv, DEC);
      if(replying) btSerial.println("OK");
    } else if(arglinebuf[0] == 'n') {
      byte angle = EEPROM.read(EEP_SERVO_ON);
      short sv = servo.write(angle, SERVO_PULSECOUNT);
      if(replying) btSerial.println(angle, DEC);
      if(replying) btSerial.println("OK");
    } else if(arglinebuf[0] == 'f') {
      byte angle = EEPROM.read(EEP_SERVO_OFF);
      short sv = servo.write(angle, SERVO_PULSECOUNT);
      if(replying) btSerial.println(angle, DEC);
      if(replying) btSerial.println("OK");
    } else {
      if(replying) btSerial.println("ERROR");
    }
  } else if(command == 't') { //RTC SET : tUTCSTAMP
    Time* t = parseTimeStamp(atol(arglinebuf));
    rtc.writeProtect(false);
    rtc.halt(false);
    rtc.time(*t);
    delete t;
    Time _temptime = rtc.time();
    time_t temptimestamp = getTimeStamp(&_temptime);
    timestampoffset = temptimestamp - millis() / 1000;
    if(replying) btSerial.println("OK");
  } else if(command == 'g') { //RTC GET : g
    if(replying) btSerial.println("OK");
    Time _temptime = rtc.time();
    if(replying) btSerial.println(getTimeStamp(&_temptime), DEC);
  } else if(command == 'q') {
    delay(atoi(arglinebuf));
    if(replying) btSerial.write("OK");
  }
}
