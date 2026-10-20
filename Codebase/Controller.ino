#include <Wire.h>
#include <Adafruit_INA219.h>
#include <avr/wdt.h>
#include <avr/io.h>

void earlyWatchdogOff(void) __attribute__((naked, section(".init3")));
void earlyWatchdogOff(void) { MCUSR=0; wdt_disable(); }
#include <math.h>
#include <stdlib.h>
#include <string.h>

const byte BUCK_PIN=9, LOAD_PIN=10, SERVO_PIN=11;
const byte SHUNT_PIN=A1, LDR_L=A2, LDR_R=A3;
Adafruit_INA219 pv(0x40), battery(0x41);

const float CHARGE_LIMIT=0.30f, PRECHARGE_LIMIT=0.05f;
const float PRECHARGE_BELOW=3.00f, CV_TARGET=4.15f;
const float OVERVOLT_TRIP=4.19f, OVERCURRENT_TRIP=0.36f;
const float TERMINATE_BELOW=0.045f;
const float LOAD_CUTOFF=3.20f, LOAD_RESTART=3.45f;
const float PV_HEADROOM=0.7f;
const float SHUNT_OHMS=0.50f, ADC_VREF=5.00f;
const byte MAX_BUCK_DUTY=160;
const unsigned long SENSOR_PERIOD=100, TRACK_PERIOD=300;
const unsigned long TELEMETRY_PERIOD=1000, STALE_LIMIT=500;
const unsigned long TERMINATION_HOLD=30000UL;
const unsigned long CHARGE_TIMEOUT=12UL*3600UL*1000UL;

float pvV=NAN, pvI=NAN, pvP=NAN, batV=NAN, chargeI=NAN, loadI=NAN;
float targetLoadA=0.0f;
int leftLDR=0, rightLDR=0, angle=90;
byte buckDuty=0;
int8_t mpptDirection=1;
float previousPower=NAN;

const byte MPPT_SETTLE_TICKS=5;
byte mpptWait=0;
byte badReadings=0;
unsigned long overcurrentRetryAt=0;
unsigned long tripTimes[3]={0,0,0};
byte tripCount=0;
const unsigned long RETRY_DELAY=3000UL, TRIP_WINDOW=60000UL;
const float CC_HYSTERESIS=0.025f;
const float CV_HYSTERESIS=0.015f;
bool chargeEnabled=false, loadEnabled=false, trackEnabled=true;
bool fault=false, full=false, loadLocked=false;
unsigned long lastGood=0, lastSense=0, lastTrack=0, lastReport=0;
unsigned long cvLowCurrentSince=0, chargeTime=0, lastChargeTick=0;
unsigned long servoFrame=0, servoStart=0;
bool servoHigh=false;
const char* mode="OFF";

void setupPWM() {
  digitalWrite(BUCK_PIN,HIGH); pinMode(BUCK_PIN,OUTPUT);
  digitalWrite(LOAD_PIN,LOW); pinMode(LOAD_PIN,OUTPUT);
  TCCR1A=_BV(WGM10); TCCR1B=_BV(CS10);
  OCR1A=0; OCR1B=0;
}
void buckOff() {
  TCCR1A &= (byte)~(_BV(COM1A1)|_BV(COM1A0));
  digitalWrite(BUCK_PIN,HIGH);
  buckDuty=0;
  previousPower=NAN;
  mpptDirection=1;
  mpptWait=0;
}
void setBuck(byte onDuty) {
  if (!onDuty) { buckOff(); return; }
  buckDuty=min(onDuty,MAX_BUCK_DUTY);
  OCR1A=buckDuty;
  TCCR1A=(TCCR1A & (byte)~(_BV(COM1A1)|_BV(COM1A0)))
         | _BV(COM1A1) | _BV(COM1A0); 
}
void stepBuck(int step) {
  setBuck((byte)constrain((int)buckDuty+step,0,(int)MAX_BUCK_DUTY));
}
void setLoad(float amps) {
  amps=constrain(amps,0.0f,0.50f);
  if (amps<=0) {
    TCCR1A &= (byte)~(_BV(COM1B1)|_BV(COM1B0));
    digitalWrite(LOAD_PIN,LOW); OCR1B=0; return;
  }
  byte duty=(byte)constrain((int)(amps*510.0f+0.5f),1,255);
  if (duty==255) {
    TCCR1A &= (byte)~(_BV(COM1B1)|_BV(COM1B0));
    digitalWrite(LOAD_PIN,HIGH);
  } else {
    OCR1B=duty;
    TCCR1A=(TCCR1A & (byte)~(_BV(COM1B1)|_BV(COM1B0))) | _BV(COM1B1);
  }
}
void shutdown(const __FlashStringHelper* reason) {
  buckOff(); setLoad(0); chargeEnabled=false; loadEnabled=false;
  fault=true; mode="FAULT";
  Serial.print(F("# FAULT: ")); Serial.println(reason);
}

void servoPulse() {
  unsigned long now=micros();
  if (servoHigh) {
    if (now-servoStart >= (unsigned long)map(angle,0,180,1000,2000)) {
      digitalWrite(SERVO_PIN,LOW); servoHigh=false;
    }
  } else if (now-servoFrame>=20000UL) {
    servoFrame=now; servoStart=now;
    digitalWrite(SERVO_PIN,HIGH); servoHigh=true;
  }
}
bool quietWindow() {
  unsigned long elapsed=micros()-servoFrame;
  return !servoHigh && elapsed>=2500UL && elapsed<13000UL;
}
void trackSun() {
  if (!trackEnabled) return;
  int difference=leftLDR-rightLDR;
  if (abs(difference)>35)
    angle=constrain(angle+(difference>0?1:-1),15,165);
}

int readAverageADC(byte pin) {
  analogRead(pin); 
  long sum=0;
  for (byte i=0;i<8;i++) sum+=analogRead(pin);
  return (int)((sum+4)/8);
}
bool readSensors() {
  float pShunt=pv.getShuntVoltage_mV();
  float pV=pv.getBusVoltage_V()+pShunt/1000.0f;
  float pI=pShunt/100.0f; 
  float bShunt=battery.getShuntVoltage_mV();
  float bV=battery.getBusVoltage_V()+bShunt/1000.0f;
  float bI=-bShunt/100.0f; 
  float lI=(readAverageADC(SHUNT_PIN)*ADC_VREF/1023.0f)/SHUNT_OHMS;
  if (!isfinite(pV)||!isfinite(pI)||!isfinite(bV)||!isfinite(bI)||!isfinite(lI)) return false;
  if (pV<0||pV>26||pI< -0.05f||pI>2.0f ||
      bV<2.0f||bV>4.6f||fabs(bI)>2.0f||lI<0||lI>1.5f) return false;
  pvV=pV; pvI=pI; pvP=pV*pI; batV=bV; chargeI=bI; loadI=lI;
  leftLDR=readAverageADC(LDR_L); rightLDR=readAverageADC(LDR_R);
  return true;
}

void overcurrentEvent(unsigned long now) {
  byte recent=0;
  for (byte i=0;i<tripCount;i++)
    if (now-tripTimes[i]<TRIP_WINDOW) recent++;
  if (recent>=3) { shutdown(F("repeated charge overcurrent")); return; }
  if (tripCount<3) tripTimes[tripCount++]=now;
  else {
    tripTimes[0]=tripTimes[1]; tripTimes[1]=tripTimes[2]; tripTimes[2]=now;
  }
  buckOff(); setLoad(0); overcurrentRetryAt=now;
  mode="OC_RETRY";
  Serial.println(F("# charge overcurrent; outputs OFF, retry in 3 s"));
}

void controlLoad() {
  if (batV<=LOAD_CUTOFF) loadLocked=true;
  if (batV>=LOAD_RESTART) loadLocked=false;
  setLoad(loadEnabled && !fault && !loadLocked ? targetLoadA : 0.0f);
}

void controlCharger(unsigned long now) {
  unsigned long dt=now-lastChargeTick; lastChargeTick=now;
  if (buckDuty>0) chargeTime+=dt;
  if (overcurrentRetryAt) {
    if (now-overcurrentRetryAt<RETRY_DELAY) {
      buckOff(); setLoad(0); mode="OC_RETRY"; return;
    }
    overcurrentRetryAt=0; previousPower=NAN; mpptDirection=1;
  }
  if (!chargeEnabled || fault || full) {
    buckOff(); mode=full?"FULL":(fault?"FAULT":"OFF"); return;
  }
  if (batV>=OVERVOLT_TRIP) { shutdown(F("battery overvoltage")); return; }
  float currentLimit=(batV<PRECHARGE_BELOW)?PRECHARGE_LIMIT:CHARGE_LIMIT;
  float tripLimit=(batV<PRECHARGE_BELOW)?PRECHARGE_LIMIT+0.03f:OVERCURRENT_TRIP;
  if (chargeI>tripLimit) { overcurrentEvent(now); return; }
  if (chargeTime>=CHARGE_TIMEOUT) { shutdown(F("charge timeout")); return; }
  static bool pvAvailable=false;
  if (pvV<6.5f || pvV<batV+PV_HEADROOM) pvAvailable=false;
  else if (pvV>=7.0f && pvV>=batV+PV_HEADROOM) pvAvailable=true;
  if (!pvAvailable) {
    buckOff(); mode="NO_SUN"; cvLowCurrentSince=0; return;
  }

  if (batV>=CV_TARGET-0.02f) {
    mode="CV"; previousPower=NAN; mpptDirection=1; mpptWait=0;
    if (chargeI>currentLimit || batV>=CV_TARGET) stepBuck(-1);
    else if (batV<CV_TARGET-CV_HYSTERESIS &&
             chargeI<currentLimit-CC_HYSTERESIS) stepBuck(+1);
    if (buckDuty>0 && batV>=CV_TARGET-0.015f &&
        chargeI>0 && chargeI<TERMINATE_BELOW) {
      if (!cvLowCurrentSince) cvLowCurrentSince=now;
      if (now-cvLowCurrentSince>=TERMINATION_HOLD) {
        full=true; buckOff(); mode="FULL";
      }
    } else cvLowCurrentSince=0;
    return;
  }
  cvLowCurrentSince=0;
  if (chargeI>=currentLimit-CC_HYSTERESIS) {
    mode="CC_LIMIT"; previousPower=NAN; mpptDirection=1; mpptWait=0;
    if (chargeI>=currentLimit) stepBuck(-1);
    return;
  }
  mode="MPPT";
  if (!buckDuty) {
    mpptDirection=1;
    previousPower=NAN;
    stepBuck(+1);
    return;
  }
  if (++mpptWait<MPPT_SETTLE_TICKS) return;
  mpptWait=0;
  if (!isfinite(previousPower)) {
    previousPower=pvP;
    stepBuck(mpptDirection);
    return;
  }
  if (pvP<previousPower-0.010f) mpptDirection=-mpptDirection;
  previousPower=pvP;
  stepBuck(mpptDirection);
}

bool parseNumber(const char* text, float &value, float minimum, float maximum) {
  if (!*text) return false;
  char* end;
  value=strtod(text,&end);
  while (*end==' ') end++;
  return end!=text && *end=='\0' && isfinite(value) &&
         value>=minimum && value<=maximum;
}
void command(char* line) {
  if (!strcmp(line,"STATUS")) report();
  else if (!strcmp(line,"LOAD OFF")) { loadEnabled=false; setLoad(0); }
  else if (!strncmp(line,"LOAD ",5)) {
    float mA;
    if (!parseNumber(line+5,mA,0.0f,500.0f)) {
      Serial.println(F("# LOAD requires 0..500 mA")); return;
    }
    targetLoadA=mA/1000.0f;
    if (!fault && !loadLocked) loadEnabled=(mA>0);
  }
  else if (!strcmp(line,"CHARGE ON")) {
    if (!fault && !full) chargeEnabled=true;
  }
  else if (!strcmp(line,"CHARGE OFF")) { chargeEnabled=false; buckOff(); }
  else if (!strcmp(line,"TRACK ON")) trackEnabled=true;
  else if (!strcmp(line,"TRACK OFF")) trackEnabled=false;
  else if (!strncmp(line,"ANGLE ",6)) {
    float requested;
    if (!parseNumber(line+6,requested,15.0f,165.0f) ||
        requested!=(int)requested) {
      Serial.println(F("# ANGLE requires integer 15..165")); return;
    }
    angle=(int)requested;
  }
  else if (!strcmp(line,"RESETFAULT")) {
    if (readSensors() && batV>LOAD_CUTOFF && batV<CV_TARGET &&
        chargeI<=CHARGE_LIMIT && pvI>=-0.05f) {
      lastGood=millis(); fault=false; badReadings=0; overcurrentRetryAt=0; tripCount=0; mode="OFF";
      chargeEnabled=false; loadEnabled=false; chargeTime=0;
      buckOff(); setLoad(0);
      Serial.println(F("# fault cleared; outputs remain OFF"));
    } else Serial.println(F("# fault remains; check wiring and limits"));
  }
  else if (!strcmp(line,"REARM")) {
    if (!fault && batV<=4.00f) { full=false; chargeTime=0; }
  }
  else Serial.println(F("# unknown command"));
}
void serialInput() {
  static char buf[32]; static byte n=0; static bool discarding=false;
  byte budget=12; 
  while (Serial.available() && budget--) {
    char c=Serial.read();
    if (c=='\r') continue;
    if (c=='\n') {
      if (!discarding && n) { buf[n]=0; command(buf); }
      n=0; discarding=false;
    } else if (discarding) {
      continue;
    } else if (n<sizeof(buf)-1) {
      if (c>='a' && c<='z') c-=32;
      buf[n++]=c;
    } else { n=0; discarding=true; Serial.println(F("# command too long")); }
  }
}
void report() {
  Serial.print(millis()); Serial.print(',');
  Serial.print(pvV,3); Serial.print(','); Serial.print(pvI,3); Serial.print(',');
  Serial.print(pvP,3); Serial.print(','); Serial.print(batV,3); Serial.print(',');
  Serial.print(chargeI,3); Serial.print(','); Serial.print(loadI,3); Serial.print(',');
  Serial.print(targetLoadA,3); Serial.print(','); Serial.print(buckDuty); Serial.print(',');
  Serial.print(angle); Serial.print(','); Serial.print(mode); Serial.print(',');
  Serial.println(fault?1:0);
}

void setup() {
  pinMode(SERVO_PIN,OUTPUT); digitalWrite(SERVO_PIN,LOW);
  setupPWM(); buckOff(); setLoad(0);
  Serial.begin(115200); Wire.begin(); Wire.setClock(100000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000,true);
#endif
  bool ok1=pv.begin(), ok2=battery.begin();
  if (ok1) pv.setCalibration_32V_2A();
  if (ok2) battery.setCalibration_32V_2A();
  if (!ok1||!ok2) shutdown(F("INA219 missing"));
  else if (!readSensors()) shutdown(F("initial sensor reading invalid"));
  else lastGood=millis();
  lastChargeTick=millis(); servoFrame=micros();
  Serial.println(F("ms,pvV,pvA,pvW,batV,chargeA,loadA,targetA,buckDuty,servo,mode,fault"));
  Serial.println(F("# charge OFF at startup; use CHARGE ON to enable"));
  wdt_enable(WDTO_1S);
}
void loop() {
  wdt_reset(); servoPulse(); serialInput();
  unsigned long now=millis();
  if (now-lastGood>STALE_LIMIT) {
    if (!fault) shutdown(F("stale sensor data"));
  }
  if (!fault && now-lastSense>=SENSOR_PERIOD && quietWindow()) {
    lastSense=now;
    if (!readSensors()) {
      buckOff(); setLoad(0); mode="SENSOR_WAIT";
      if (++badReadings>=3) shutdown(F("three invalid sensor readings"));
    } else {
      badReadings=0; lastGood=millis();
      if (!overcurrentRetryAt) controlLoad();
      controlCharger(millis());
    }
  }
  if (now-lastTrack>=TRACK_PERIOD) { lastTrack=now; trackSun(); }
  if (now-lastReport>=TELEMETRY_PERIOD && quietWindow()) {
    lastReport=now; report();
  }
}