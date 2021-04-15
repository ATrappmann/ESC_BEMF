// NAME: TestAC.ino
//
// DESC: Test the Analog Comparator (AC) of an Atmega328p
//

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("TestAC running..."));
  Serial.println("Uploaded: "  __DATE__  " " __TIME__);

  Serial.println(F("Set AIN0=D6 to 3.3V and A0 to LOW"));
  ADCSRA = (0 << ADEN);     // Disable the ADC module
  ADCSRB = (1 << ACME);     // MUX select for negative input of comparator
  ADMUX = 0;                // Select A0 as comparator negative input
//  ACSR |= Bx00011011;       // Set IRQ on rising edge, enable AC IRQ and clear ACI

}

void loop() {
  byte acStatus = ACSR;
  Serial.print(F("ACSR=0b")); Serial.print(acStatus, BIN);
  if (0 != (acStatus & (1<<ACO))) {
    Serial.println(F(", ACO=HIGH"));
  }
  else Serial.println(F(", ACO=LOW"));
  Serial.flush();
}

/*
 * Interrumption vector for the Analog comparator
 */
ISR (ANALOG_COMP_vect) {
  Serial.print("*");
/*
  for (int i=0; i<10; i++) {            // We check the comparator 10 times just to be sure
    if (ACSR & 0x01) {                  // If step is with IRQ on rising BEMF, ACO should be 0
      if (0 != (ACSR & (1<<ACO)))) {
        i--;  // ACO = 1, check again
        Serial.print("#");
      }
    } else {                            // If step is with IRQ on falling BEMF, ACO should be 1
      if (0 == (ACSR & (1<<ACO)))) {
        i--;  // ACO = 0, check again
        Serial.print("#");
    }
  }
*/
  // Toggle between rising and falling edge
  if (0 != (ACSR & 0x01)) {
    ACSR &= ~0x01;    // Set interrupt on falling edge
  } else {
    ACSR |= 0x01;     // Set interrupt on rising edge
  }
}
