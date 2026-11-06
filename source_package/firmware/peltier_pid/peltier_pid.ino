/*
  peltier_pid.ino - PID temperature control of a Peltier-cooled surface
  IE3034 Control Systems Engineering project

  Board   : Arduino Uno / Nano (ATmega328P, 16 MHz)
  Sensors : 2 x DS18B20 on one 1-Wire bus (pin 2, 4.7 k pull-up to 5 V)
              index SURF_IDX - bonded to the cold surface (controlled variable)
              index HOT_IDX  - on the heat sink next to the module (safety only)
  Actuator: TEC1-12706 switched low-side by a logic-level N-MOSFET on pin 9,
            25 kHz PWM from Timer1 (inaudible, 321 duty levels)
  Load    : power resistor switched by a second MOSFET on pin 6 (test disturbance)
  Fan     : wired permanently to 12 V - the module must NEVER run without it

  Control law - identical to matlab/simulate_loop.m:
      e = T_meas - T_ref                 (reverse acting: plant gain is negative)
      P = Kp*e
      I = I + Ki*Ts*e                    (frozen while saturated in the same direction)
      D = Tf/(Tf+Ts)*D + Kd/(Tf+Ts)*(T_meas - T_meas_prev)   (on measurement, filtered)
      u = constrain(P + I + D, 0, 1)

  Serial, 115200 baud, newline-terminated commands:
      SP 15.0      setpoint [degC] (limited to SP_MIN..SP_MAX)
      KP 0.48      KI 0.0048     KD 1.2     TF 2.0      gains / derivative filter [s]
      MODE AUTO    closed loop (bumpless from OPEN)
      MODE OPEN    open loop, duty set by DUTY  (identification tests)
      DUTY 0.6     open-loop duty 0..1
      LOAD 1       heater disturbance on (LOAD 0 = off)
      RESET        clear a latched fault
      ?            print gains and state
  Log line every Ts (CSV): t_s,T_surf,T_hot,T_ref,duty,load,mode
  Lines starting with '#' are messages, not data.
*/
#include <OneWire.h>
#include <DallasTemperature.h>

// ------------------------------- pins & limits ------------------------------
const uint8_t  PIN_ONEWIRE = 2;
const uint8_t  PIN_TEC     = 9;      // OC1A
const uint8_t  PIN_HEATER  = 6;
const uint8_t  SURF_IDX    = 0;      // swap to 1/0 if the startup print shows them reversed
const uint8_t  HOT_IDX     = 1;
const uint16_t PWM_TOP     = 320;    // 16 MHz / (2 * 320) = 25 kHz, phase-correct
const float    TS          = 1.0f;   // controller period [s] - must equal P.Ts in MATLAB
const float    T_HOT_MAX   = 60.0f;  // heat-sink trip temperature [degC]
const float    SP_MIN      = 5.0f;
const float    SP_MAX      = 30.0f;

// ------------------------------- tuning (MATLAB: PID_recommended) -----------
float Kp = 0.48f, Ki = 0.0048f, Kd = 1.2f, Tf = 2.0f;

// ------------------------------- state --------------------------------------
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);
DeviceAddress addrSurf, addrHot;
bool  haveHot = false;

enum Mode { AUTO, OPEN };
Mode  mode = AUTO;
float Tref = 25.0f;                  // start at ambient: no cooling until SP is sent
float duty = 0.0f, dutyOpen = 0.0f;
float Iterm = 0.0f, Dterm = 0.0f, yPrev = 0.0f;
bool  havePrev = false, loadOn = false, fault = false;
unsigned long tick0, lastTick;
char  line[40]; uint8_t nline = 0;

// ------------------------------- PWM ----------------------------------------
void pwmInit() {
  pinMode(PIN_TEC, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);               // non-inverting, mode 10 (phase-correct, TOP = ICR1)
  TCCR1B = _BV(WGM13) | _BV(CS10);                 // prescaler 1
  ICR1   = PWM_TOP;
  OCR1A  = 0;
}
void setDuty(float u) {
  duty = constrain(u, 0.0f, 1.0f);
  OCR1A = (uint16_t)(duty * PWM_TOP + 0.5f);
}

// ------------------------------- setup --------------------------------------
void setup() {
  pwmInit();
  setDuty(0);
  pinMode(PIN_HEATER, OUTPUT); digitalWrite(PIN_HEATER, LOW);
  Serial.begin(115200);
  sensors.begin();
  sensors.setResolution(12);                        // 0.0625 degC
  sensors.setWaitForConversion(false);              // non-blocking: convert during the 1 s period
  uint8_t n = sensors.getDeviceCount();
  Serial.print(F("# DS18B20 found: ")); Serial.println(n);
  if (n == 0 || !sensors.getAddress(addrSurf, SURF_IDX)) {
    fault = true; Serial.println(F("# FAULT: surface sensor missing"));
  }
  haveHot = (n > 1) && sensors.getAddress(addrHot, HOT_IDX);
  if (!haveHot) Serial.println(F("# WARNING: no heat-sink sensor - over-temperature trip disabled"));
  sensors.requestTemperatures();
  delay(800);
  Serial.println(F("t_s,T_surf,T_hot,T_ref,duty,load,mode"));
  tick0 = lastTick = millis();
}

// ------------------------------- control step -------------------------------
void controlStep(float y) {
  float e = y - Tref;
  if (!havePrev) { yPrev = y; havePrev = true; }
  Dterm = (Tf / (Tf + TS)) * Dterm + (Kd / (Tf + TS)) * (y - yPrev);
  yPrev = y;
  if (mode == OPEN) {                               // keep I consistent for bumpless transfer
    Iterm = dutyOpen - Kp * e - Dterm;
    setDuty(dutyOpen);
    return;
  }
  float P = Kp * e;
  float Inew = Iterm + Ki * TS * e;
  float v = P + Inew + Dterm;
  if ((v > 1.0f && e > 0) || (v < 0.0f && e < 0)) Inew = Iterm;   // clamping anti-windup
  Iterm = Inew;
  setDuty(P + Iterm + Dterm);
}

// ------------------------------- main loop ----------------------------------
void loop() {
  readSerial();
  if (millis() - lastTick < (unsigned long)(TS * 1000)) return;
  lastTick += (unsigned long)(TS * 1000);

  float Ts_ = sensors.getTempC(addrSurf);           // conversion requested one period ago
  float Th  = haveHot ? sensors.getTempC(addrHot) : NAN;
  sensors.requestTemperatures();                    // start the next conversion

  bool badSurf = (Ts_ == DEVICE_DISCONNECTED_C) || (Ts_ == 85.0f);   // 85 = power-on value
  if (!fault && badSurf) { fault = true; Serial.println(F("# FAULT: surface sensor read error")); }
  if (!fault && haveHot && Th > T_HOT_MAX) { fault = true; Serial.println(F("# FAULT: heat sink over temperature")); }

  if (fault) { setDuty(0); Iterm = 0; havePrev = false; }
  else       { controlStep(Ts_); }

  float t = (millis() - tick0) / 1000.0f;
  Serial.print(t, 1);       Serial.print(',');
  Serial.print(Ts_, 4);     Serial.print(',');
  Serial.print(Th, 2);      Serial.print(',');
  Serial.print(Tref, 2);    Serial.print(',');
  Serial.print(duty, 4);    Serial.print(',');
  Serial.print(loadOn ? 1 : 0); Serial.print(',');
  Serial.println(fault ? F("FAULT") : (mode == AUTO ? F("AUTO") : F("OPEN")));
}

// ------------------------------- commands -----------------------------------
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n' || nline >= sizeof(line) - 1) { line[nline] = 0; nline = 0; handle(line); }
    else line[nline++] = c;
  }
}

void handle(char *s) {
  char *cmd = strtok(s, " ");
  char *arg = strtok(NULL, " ");
  if (!cmd) return;
  float v = arg ? atof(arg) : 0;
  if      (!strcasecmp(cmd, "SP"))   Tref = constrain(v, SP_MIN, SP_MAX);
  else if (!strcasecmp(cmd, "KP"))   Kp = v;
  else if (!strcasecmp(cmd, "KI"))   Ki = v;
  else if (!strcasecmp(cmd, "KD"))   Kd = v;
  else if (!strcasecmp(cmd, "TF"))   Tf = max(v, TS);
  else if (!strcasecmp(cmd, "DUTY")) dutyOpen = constrain(v, 0.0f, 1.0f);
  else if (!strcasecmp(cmd, "LOAD")) { loadOn = (v > 0.5f); digitalWrite(PIN_HEATER, loadOn ? HIGH : LOW); }
  else if (!strcasecmp(cmd, "RESET")) { fault = false; Serial.println(F("# fault cleared")); }
  else if (!strcasecmp(cmd, "MODE") && arg) {
    if (!strcasecmp(arg, "OPEN")) { dutyOpen = duty; mode = OPEN; }
    else if (!strcasecmp(arg, "AUTO")) mode = AUTO;     // Iterm already tracks -> bumpless
  }
  else if (!strcmp(cmd, "?")) {
    Serial.print(F("# Kp=")); Serial.print(Kp, 4); Serial.print(F(" Ki=")); Serial.print(Ki, 6);
    Serial.print(F(" Kd=")); Serial.print(Kd, 4);  Serial.print(F(" Tf=")); Serial.print(Tf, 2);
    Serial.print(F(" SP=")); Serial.print(Tref, 2); Serial.print(F(" I=")); Serial.println(Iterm, 4);
    return;
  }
  else { Serial.print(F("# unknown: ")); Serial.println(cmd); return; }
  Serial.print(F("# ok ")); Serial.println(cmd);
}
