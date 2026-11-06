// Open-loop Peltier surface-temperature test (no PID).
// Hardware assumed: DS18B20 taped to the Peltier cold surface (pin 2, 4.7k pull-up),
// logic-level MOSFET gate on PWM pin 9. CHANGE to match your build.
// !! Always run the hot side with a heatsink + fan, otherwise the module can overheat. !!
//
// Serial commands (115200 baud, newline terminated):
//   D40  -> apply 40 % duty (step) and start logging
//   X    -> duty = 0 (stop). Wait until the plate returns to ambient before the next test.
// Output CSV, once per second:  time_s,duty,temp_C
// Save the output as surface_20.csv, surface_40.csv, surface_60.csv ... (number = duty %)

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_PIN 2
#define PWM_PIN 9

OneWire ow(ONE_WIRE_PIN);
DallasTemperature sensors(&ow);

const unsigned long Ts_ms = 1000;
const unsigned long MAX_ON_MS = 1200000UL;   // safety: auto switch-off after 20 min

float dutyPct = 0;
bool logging = false;
unsigned long tStart = 0, tNext = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  sensors.begin();
  sensors.setResolution(10);
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() > 0 && (s[0] == 'D' || s[0] == 'd')) {
      dutyPct = constrain(s.substring(1).toFloat(), 0, 100);
      analogWrite(PWM_PIN, (int)(dutyPct * 2.55));
      tStart = millis(); tNext = tStart;
      logging = true;
      Serial.println("time_s,duty,temp_C");
    } else if (s.length() > 0 && (s[0] == 'X' || s[0] == 'x')) {
      analogWrite(PWM_PIN, 0);
      logging = false;
      Serial.println("STOPPED");
    }
  }

  if (logging && millis() >= tNext) {
    tNext += Ts_ms;
    sensors.requestTemperatures();
    float T = sensors.getTempCByIndex(0);
    Serial.print((millis() - tStart) / 1000.0, 0); Serial.print(',');
    Serial.print(dutyPct, 0);                       Serial.print(',');
    Serial.println(T, 2);
    if (millis() - tStart > MAX_ON_MS) {            // safety cut-off
      analogWrite(PWM_PIN, 0); logging = false; Serial.println("AUTO-STOP");
    }
  }
}
