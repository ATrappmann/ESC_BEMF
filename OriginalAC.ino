// NAME: OriginalAC.ino
//
// DESC: Test the Analog Comparator (AC) of an Atmega328p
//

#include <Arduino.h>

#define STATE_PIN   2
bool state = HIGH;
int sequence_step = 0;
int i;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println(F("OriginalAC running..."));
  Serial.println("Uploaded: "  __DATE__  " " __TIME__);

  Serial.println(F("Set D6 to 3.3V and connect A0 to D2"));
  Serial.println(F("D2 will toggle every 2s"));

  pinMode(STATE_PIN, OUTPUT);
  Serial.print(F("state=")); Serial.println(state); Serial.flush();
  digitalWrite(STATE_PIN, state);  // state=HIGH -> AIN1>AIN0 => ACO=0
  delay(5000);

  // Comparator on pin D6
  DIDR1 |= (1<<AIN0D);          // Disable digital input buffer for AIN0
  ACSR  = 0x10;                 // Clear flag comparator interrupt
  BEMF_C_RISING();
  sequence_step++;
  ACSR |= 0x08;                 // Enable analog comparator interrupt
}

unsigned long lastMillis = 0L;
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis < 2000L) return;
  lastMillis = currentMillis;

  // toggle state
  state = !state;
  Serial.print(F("\nstate=")); Serial.print(state); Serial.flush();
  digitalWrite(STATE_PIN, state);
}

/*On each step we know that the next 0 cross will be rising or falling and if it will be
on coil A, B or C. With these funcstions we select that according to the step of the sequence*/
void BEMF_C_RISING(){
  ADCSRA = (0 << ADEN);     // Disable the ADC module
  ADCSRB = (1 << ACME);     // MUX select for negative input of comparator
  ADMUX = 0;                // Select A0 as comparator negative input
  ACSR |= 0x03;             // Set interrupt on rising edge
}

// Interrumption vector for the Analog comparator
ISR (ANALOG_COMP_vect) {
  for(i = 0; i < 10; i++) {           //We check the comparator 10 times just to be sure
    if(sequence_step & 1)             //If step = odd (0001, 0011, 0101) 1, 3 or 5
    {
      if(!(ACSR & B00100000)) i -= 1; //!B00100000 -> B11011111 ACO = 0 (Analog Comparator Output = 0)
      Serial.print("#"); Serial.println(ACSR, BIN);
    }
    else                              //else if step is 0, 2 or 4
    {
      if((ACSR & B00100000))  i -= 1; //else if B00100000 -> B11011111 ACO = 1 (Analog Comparator Output = 1)
      Serial.print("+"); Serial.println(ACSR, BIN);
    }
  }
  /*
  set_next_step();                    //set the next step of the sequence
  sequence_step++;                    //increment step by 1, next part of the sequence of 6
  sequence_step %= 6;                 //If step > 5 (equal to 6) then step = 0 and start over
  */
}
