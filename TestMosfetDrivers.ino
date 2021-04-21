// NAME: TestMosfetDrivers.ino
//

#include <Arduino.h>
#include "PinFunctions.h"

#define STATE_PIN   5

static int step = 0;

void pressEnterToContinue() {
  Serial.println(F("Press Enter to continue"));
  char c = '\0';
  while ('\n' != c) {
    if (Serial.available()) {
      c = Serial.read();
    }
    else delay(50);
  }
}

void setup() {
  // Our pins for the MOSFET drivers are 2,3,4 and 9,10,11
  DDRD  |= B00011100;       // Configure pins 2, 3 and 4 as outputs CL, BL and AL
  PORTD  = B00000000;       // Pins 0 to 7 set to LOW
  DDRB  |= B00001110;       // Configure pins 9, 10 and 11 as outputs
  PORTB &= B00110001;       // D9, D10 and D11 to LOW

  pinMode(STATE_PIN, OUTPUT);
  
  Serial.begin(9600);
  while (!Serial);
  Serial.println(F("TestMosfetDrivers running..."));
  Serial.println("Uploaded: "  __DATE__  " " __TIME__);

  // Configure Analog Comparator
  ACSR   = (1<<ACI);      // Clear analog comparator interrupt flag
  DIDR1 |= (1<<AIN0D);    // Disable digital input buffer for AIN0(=D6)

  // Timer1
  TCCR1A = 0;     // OC1A(D9) and OC1B(D10) normal ports
  TCCR1B = 0x01;  // no prescaling

  // Timer2
  TCCR2A = 0;     // OC2A(D11) normal port
  TCCR2B = 0x01;  // no prescaling

  // Define PWM signal (50% duty-cycle)
  SET_PWM(128);

  Serial.println(F("Check all pins for inactive signals"));
  Serial.println(F("Phase A: AH= D9, AL=D4"));
  Serial.println(F("Phase B: BH=D10, BL=D3"));
  Serial.println(F("Phase C: CH=D11, CL=D2"));
  pressEnterToContinue();
  
  step = 0;
}

void loop () {
  testStep(step++);
  if (step >= 6) stop("Done");
}

void testStep(const int step) {
  Serial.print(F("Test step #")); Serial.println(step);
  switch (step) {
    case 0:
      Serial.println(F("#0: AH-BL, trigger on C falling"));
      Serial.print(F("Set C to 12V or connect D6 with 3.3V and A0 to D")); Serial.println(STATE_PIN);
      pressEnterToContinue();
      
      AH_BL();
      BEMF_C_FALLING();
      digitalWrite(STATE_PIN, HIGH);
      Serial.println(F("Check Phases: A=12V PWM (D9=PWM), B=0V (D3=HIGH), C=12V (A0=HIGH)"));
      pressEnterToContinue();
      
      Serial.print(F("Set C to 0V or wait for trigger with D")); Serial.print(STATE_PIN);
      Serial.println(F(" and check for interrupt"));
      ACSR |=  (1<<ACIE);   // Enable analog comparator interrupt
      digitalWrite(STATE_PIN, LOW);
      pressEnterToContinue();

      ACSR &= ~(1<<ACIE);   // Disable analog comparator interrupt
      TCCR2A = 0;           // OC2A(D11) normal port
      TCCR1A = 0;           // OC1A(D9) and OC1B(D10) normal ports
      PORTD  = 0;           // pins 0 to 7 set to LOW
      PORTB  = 0;           // pins 8 to 13 set to LOW
      Serial.print(F("Disconnect C or A0 from D")); Serial.println(STATE_PIN);
      pressEnterToContinue();
      break;
   
    case 1:
      Serial.println(F("#1: AH-CL, trigger on B rising"));
      Serial.print(F("Set B to 0V or connect D6 with 3.3V and A1 to D")); Serial.println(STATE_PIN);
      pressEnterToContinue();
      
      AH_CL();
      BEMF_B_RISING();
      digitalWrite(STATE_PIN, LOW);
      Serial.println(F("Check Phases: A=12V PWM (D9=PWM), C=0V (D2=HIGH), B=0V (A1=LOW)"));
      pressEnterToContinue();
      
      Serial.print(F("Set B to 12V or wait for trigger with D")); Serial.print(STATE_PIN);
      Serial.println(F(" and check for interrupt"));
      ACSR |=  (1<<ACIE);   // Enable analog comparator interrupt
      digitalWrite(STATE_PIN, HIGH);
      pressEnterToContinue();

      ACSR &= ~(1<<ACIE);   // Disable analog comparator interrupt
      TCCR2A = 0;           // OC2A(D11) normal port
      TCCR1A = 0;           // OC1A(D9) and OC1B(D10) normal ports
      PORTD  = 0;           // pins 0 to 7 set to LOW
      PORTB  = 0;           // pins 8 to 13 set to LOW
      Serial.print(F("Disconnect B or A1 from D")); Serial.println(STATE_PIN);
      pressEnterToContinue();
      break;

    case 2:
      Serial.println(F("#2: BH-CL, trigger on A falling"));
      Serial.print(F("Set A to 12V or connect D6 with 3.3V and A2 to D")); Serial.println(STATE_PIN);
      pressEnterToContinue();
      
      BH_CL();
      BEMF_A_FALLING();
      digitalWrite(STATE_PIN, HIGH);
      Serial.println(F("Check Phases: B=12V PWM (D10=PWM), C=0V (D2=HIGH), A=12V (A2=HIGH)"));
      pressEnterToContinue();
      
      Serial.print(F("Set A to 0V or wait for trigger with D")); Serial.print(STATE_PIN);
      Serial.println(F(" and check for interrupt"));
      ACSR |=  (1<<ACIE);   // Enable analog comparator interrupt
      digitalWrite(STATE_PIN, LOW);
      pressEnterToContinue();

      ACSR &= ~(1<<ACIE);   // Disable analog comparator interrupt
      TCCR2A = 0;           // OC2A(D11) normal port
      TCCR1A = 0;           // OC1A(D9) and OC1B(D10) normal ports
      PORTD  = 0;           // pins 0 to 7 set to LOW
      PORTB  = 0;           // pins 8 to 13 set to LOW
      Serial.print(F("Disconnect A or A2 from D")); Serial.println(STATE_PIN);
      pressEnterToContinue();
      break;
      
    case 3:
      Serial.println(F("#3: BH-AL, trigger on C rising"));
      Serial.print(F("Set C to 0V or connect D6 with 3.3V and A0 to D")); Serial.println(STATE_PIN);
      pressEnterToContinue();

      BH_AL();
      BEMF_C_RISING();
      digitalWrite(STATE_PIN, LOW);
      Serial.println(F("Check Phases: B=12V PWM (D10=PWM), A=0V (D4=HIGH), C=0V (A0=LOW)"));
      pressEnterToContinue();
      
      Serial.print(F("Set C to 12V or wait for trigger with D")); Serial.print(STATE_PIN);
      Serial.println(F(" and check for interrupt"));
      ACSR |=  (1<<ACIE);   // Enable analog comparator interrupt
      digitalWrite(STATE_PIN, HIGH);
      pressEnterToContinue();

      ACSR &= ~(1<<ACIE);   // Disable analog comparator interrupt
      TCCR2A = 0;           // OC2A(D11) normal port
      TCCR1A = 0;           // OC1A(D9) and OC1B(D10) normal ports
      PORTD  = 0;           // pins 0 to 7 set to LOW
      PORTB  = 0;           // pins 8 to 13 set to LOW
      Serial.print(F("Disconnect C or A0 from D")); Serial.println(STATE_PIN);
      pressEnterToContinue();
      break;
      
    case 4:
      Serial.println(F("#4: CH-AL, trigger on B falling"));
      Serial.print(F("Set B to 12V or connect D6 with 3.3V and A1 to D")); Serial.println(STATE_PIN);
      pressEnterToContinue();

      CH_AL();
      BEMF_B_FALLING();
      digitalWrite(STATE_PIN, HIGH);
      Serial.println(F("Check Phases: C=12V PWM (D11=PWM), A=0V (D4=HIGH), B=12V (A1=HIGH)"));
      pressEnterToContinue();
      
      Serial.print(F("Set B to 0V or wait for trigger with D")); Serial.print(STATE_PIN);
      Serial.println(F(" and check for interrupt"));
      ACSR |=  (1<<ACIE);   // Enable analog comparator interrupt
      digitalWrite(STATE_PIN, LOW);
      pressEnterToContinue();

      ACSR &= ~(1<<ACIE);   // Disable analog comparator interrupt
      TCCR2A = 0;           // OC2A(D11) normal port
      TCCR1A = 0;           // OC1A(D9) and OC1B(D10) normal ports
      PORTD  = 0;           // pins 0 to 7 set to LOW
      PORTB  = 0;           // pins 8 to 13 set to LOW
      Serial.print(F("Disconnect B or A1 from D")); Serial.println(STATE_PIN);
      pressEnterToContinue();
      break;

    case 5:
      Serial.println(F("#5: CH-BL, trigger on A rising"));
      Serial.print(F("Set A to 0V or connect D6 with 3.3V and A2 to D")); Serial.println(STATE_PIN);
      pressEnterToContinue();

      CH_BL();
      BEMF_A_RISING();
      digitalWrite(STATE_PIN, LOW);
      Serial.println(F("Check Phases: C=12V PWM (D11=PWM), B=0V (D3=HIGH), A=0V (A2=LOW)"));
      pressEnterToContinue();
      
      Serial.print(F("Set A to 12V or wait for trigger with D")); Serial.print(STATE_PIN);
      Serial.println(F(" and check for interrupt"));
      ACSR |=  (1<<ACIE);   // Enable analog comparator interrupt
      digitalWrite(STATE_PIN, HIGH);
      pressEnterToContinue();

      ACSR &= ~(1<<ACIE);   // Disable analog comparator interrupt
      TCCR2A = 0;           // OC2A(D11) normal port
      TCCR1A = 0;           // OC1A(D9) and OC1B(D10) normal ports
      PORTD  = 0;           // pins 0 to 7 set to LOW
      PORTB  = 0;           // pins 8 to 13 set to LOW
      Serial.print(F("Disconnect A or A2 from D")); Serial.println(STATE_PIN);
      pressEnterToContinue();
      break;
      
    default:
      stop("FATAL ERROR: Illegal state detected!");
  }
}

/*
 * Interrupt vector for the Analog comparator
 * Def: AC Output (ACO) is 1, if voltage on positive input pin of the AC (=AIN0)
 *      is higher than voltage on negative input pin (=AIN1).
 * AIN0(=D6) is tied to common ground of all 3 phases.
 * AIN1(multiplexed) to: A2 for phase A, A1 for phase B, A0 for phase C
 * To detect a rising edge on the AC, AIN1 has to go below common ground (AIN0>AIN1), so ACO gets 1.
 * -> This is for detecting a falling BEMF signal.
 * To detect a falling edge on the AC, AIN1 has to go above common ground (AIN0<AIN1), so ACO gets 0.
 * -> This is for detecting a rising BEMF signal.
 */
ISR (ANALOG_COMP_vect) {
  byte acStatus = ACSR;
  Serial.print("*");
  for (int i=0; i<10; i++) {                // We check the comparator 10 times just to be sure
    if (0 != (acStatus & 0x01)) {           // If step is with IRQ on rising edge, ACO should be 1
      if (0 == (acStatus & (1<<ACO))) --i;  // If ACO is 0, check again
      Serial.print("\\_"); Serial.println(acStatus, BIN); Serial.flush();
    } else {                                // If step is with IRQ on falling edge, ACO should be 0
      if (0 != (acStatus & (1<<ACO))) --i;  // If ACO is 1, check again
      Serial.print("/^"); Serial.println(acStatus, BIN); Serial.flush();
    }
  }
}

void stop(const char *msg) {
  ACSR   = 0;       // Disable analog comparator interrupt
  TCCR2A = 0;       // OC2A(D11) normal port
  TCCR1A = 0;       // OC1A(D9) and OC1B(D10) normal ports
  PORTD  = 0;       // pins 0 to 7 set to LOW
  PORTB  = 0;       // pins 8 to 13 set to LOW

  Serial.println(msg);
  Serial.println(F("Exit - Press RESET to continue!"));
  Serial.flush();
  Serial.end();
  exit(-1);
}
