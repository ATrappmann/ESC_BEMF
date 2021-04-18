// NAME: TestAC.ino
//
// DESC: Test the Analog Comparator (AC) of an Atmega328p
//

#include <Arduino.h>

bool state = HIGH;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println(F("TestAC running..."));
  Serial.println("Uploaded: "  __DATE__  " " __TIME__);

  Serial.println(F("Set AIN0=D6 to 3.3V and connect A0 to D2"));
  Serial.println(F("D2 will toggle every 2s"));

  pinMode(2, OUTPUT);
  Serial.print(F("state=")); Serial.println(state); Serial.flush();
  digitalWrite(2, state);  // ACO=0
  delay(5000);

  ADCSRA = (0 << ADEN);     // Disable the ADC module
  ADCSRB = (1 << ACME);     // MUX select for negative input of comparator
  ADMUX = 0;                // Select A0 as comparator negative input
  ACSR = B00011011;         // Set IRQ on rising edge, enable AC IRQ and clear ACI
}

unsigned long lastMillis = 0L;
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis < 2000L) return;
  lastMillis = currentMillis;

  // toggle state
  state = !state;
  Serial.print(F("\nstate=")); Serial.print(state); Serial.flush();
  digitalWrite(2, state);

  /*
  byte acStatus = ACSR;
  Serial.print(F("ACSR=0b")); Serial.print(acStatus, BIN);
  if (0 != (acStatus & (1<<ACO))) {
    Serial.println(F(", ACO=HIGH"));
  }
  else Serial.println(F(", ACO=LOW"));
  Serial.flush();
  */
}

/*
 * Interrupt vector for the Analog comparator
 * Def: AC Output (ACO) is 1, if voltage on positive input pin of AC (=AIN0) is
 *      higher than voltage on negative input pin (=AIN1).
 * AIN0(=D6) is tied to common ground of all 3 phases.
 * AIN1(multiplexed) to: A2 for phase A, A1 for phase B, A0 for phase C
 * To detect a  rising edge, AIN1 has to go below common ground (AIN0 > AIN1), so ACO is 1. This is for a falling BEMF signal (1->0).
 * To detect a falling edge, AIN1 has to go above common ground (AIN0 < AIN1), so ACO is 0. This is for a rising BEMF signal (0->1).
 */
ISR (ANALOG_COMP_vect) {
  Serial.print("*");
  for (int i=0; i<10; i++) {            // We check the comparator 10 times just to be sure
    if (0 != (ACSR & 0x01)) {           // If step is with IRQ on rising edge, ACO should be 1
      if (0 == (ACSR & (1<<ACO))) --i;  // If ACO is 0, check again
      Serial.print("#"); Serial.println(ACSR, BIN);
    } else {                            // If step is with IRQ on falling edge, ACO should be 0
      if (0 != (ACSR & (1<<ACO))) --i;  // If ACO is 1, check again
      Serial.print("+"); Serial.println(ACSR, BIN);
    }
  }

/*
  // Toggle between rising and falling edge
  if (0 != (ACSR & 0x01)) {
    ACSR &= ~0x01;    // Set interrupt on falling edge
  } else {
    ACSR |= 0x01;     // Set interrupt on rising edge
  }
*/
}
