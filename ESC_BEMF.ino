/* Title: ELECTRONOOBS open source electronic speed controller.
 * Date: 02/04/2019
 * Version: 3.3
 * Author: http://electronoobs.com
 * Tutorial link: https://www.electronoobs.com/eng_arduino_tut91.php
 * Schematic link: https://www.electronoobs.com/eng_arduino_tut91_sch1.php
 * PCB gerbers: https://www.electronoobs.com/eng_arduino_tut91_gerbers1.php
 * This is a sensorless ESC based on Arduino with the ATmega328 chip. It uses
 * BEMF with the internal comparator of the ATmega328 to detect the rotor position.
 * The speed control is made by a PWM signal. Feel free to change it and improve
 * it however you want
 * Subscribe: http://youtube.com/c/ELECTRONOOBS
 */
//#define beeping_PWM_VALUE  100

#include "PinFunctions.h"
#include "EEPROMAnything.h"         //This is used to store more than just one byte to the EEPROM
#include "Beeps.h"

//#define TEST_FUNCTIONS  1
#ifdef TEST_FUNCTIONS
#include "TestFunctions.h"
#endif

/* Inputs/Outputs
 * PWM in pin - D8
 * High A - D9
 * LOW A - D4
 * HIGH B - D10
 * LOW B - D3
 * HIGH C - D11
 * LOW C - D2
 * Comparator - D6
*/

#define PWM_IN_MIN_ADRESS 2
#define PWM_IN_MAX_ADRESS 7

int PWM_IN_MIN = 1000;
int PWM_IN_MAX = 2000;
int MIN_PWM_TO_STORE = 0;
int MAX_PWM_TO_STORE = 0;
int pwm_set_counter = 0;

byte sequence_step = 0, motor_speed;
bool MOTOR_SPINNING = false;
int motor_off_counter = 0;
bool PWM_RANGE_SET = false;
unsigned long previousMillis = 0;
bool full_stop = false;

extern int PWM_INPUT;

void setup() {
  Serial.begin(9600);
  Serial.println(F("ESC_BEMF"));
  Serial.println("Uploaded: "  __DATE__  " " __TIME__);

  //This will only run once after you upload the code
  if (EEPROM.read(1) != 1) {
    Serial.println(F("Init EEPROM with defaults"));
    EEPROM_writeAnything(PWM_IN_MIN_ADRESS, PWM_IN_MIN);
    EEPROM_writeAnything(PWM_IN_MAX_ADRESS, PWM_IN_MAX);
    EEPROM.write(1, 1);
  } else { // Load PWM range from EEPROM
    EEPROM_readAnything(PWM_IN_MIN_ADRESS, PWM_IN_MIN);
    EEPROM_readAnything(PWM_IN_MAX_ADRESS, PWM_IN_MAX);
    Serial.println(F("Read from EEPROM:"));
    Serial.print(F("PWM_IN_MIN = ")); Serial.println(PWM_IN_MIN);
    Serial.print(F("PWM_IN_MAX = ")); Serial.println(PWM_IN_MAX);
  }

  //Our pins for the MOSFET drivers are 2,3,4 and 9,10,11
  DDRD  |= B00011100;           //Configure pins 2, 3 and 4 as outputs CL, BL and AL
  PORTD  = B00000000;           //Pins 0 to 7 set to LOW
  DDRB  |= B00001110;           //Configure pins 9, 10 and 11 as outputs
  PORTB &= B00110001;           //D9, D10 and D11 to LOW

  // Timer1
  TCCR1A = 0;
  TCCR1B = 0x01;  // no prescaling

  // Timer2
  TCCR2A = 0;
  TCCR2B = 0x01;  // no prescaling

#ifdef TEST_FUNCTIONS
//  testPinFunction();
  ACSR  = 0x10;           // Clear flag comparator interrupt
  ACSR |= 0x08;           // Enable analog comparator interrupt
  testRotationSteps();

  Serial.println(F("Exit -- Press RESET\n"));
  Serial.flush();
  Serial.end();
  exit(0);
#endif

  // Configure the Analog Comparator
  ACSR   = (1<<ACI);      // Clear analog comparator interrupt flag
  DIDR1 |= (1<<AIN0D);    // Disable digital input buffer for AIN0(=D6)

  // Set D8 (PWM in) to trigger interrupt (we use this to read PWM input)
  PCICR  |= (1 << PCIE0);    //enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);   //Set pin D8 trigger an interrupt on state change.

  /*Now we detect the PWM input and if is higher than PWM_IN_MIN we enter
  configuration mode and if not, we jump to the void loop*/
  delay(200);

  // Enter config mode if Throttle is not in idle state, close to minimum
  if (PWM_INPUT > PWM_IN_MIN + 115) {
    Serial.println(F("Configuration mode: move Throttle to max, than to min value"));
    PWM_RANGE_SET = false;
    while (!PWM_RANGE_SET) {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= 500) {  // beep every 500ms with 1KHz
        previousMillis += 500;
//        OCR1A = beeping_PWM_VALUE;
        TCCR2A = 0;   // OC2A(D11) normal port
        TCCR1A = 0;   // OC1A(D9) and OC1B(D10) normal ports
        beep_1KHz(100);
      }

      if (PWM_INPUT > MAX_PWM_TO_STORE) { // save new max value
        MAX_PWM_TO_STORE = PWM_INPUT;
      }

      if (PWM_INPUT < PWM_IN_MIN + 115) {  // Throttle close to minimum
        if (pwm_set_counter > 1000) { // longer than 1000 cycles in minimum?
          MIN_PWM_TO_STORE = PWM_INPUT; // save current value as minimum

          PWM_IN_MIN = MIN_PWM_TO_STORE;
          PWM_IN_MAX = MAX_PWM_TO_STORE;

          Serial.println(F("Saving to EEPROM:"));
          Serial.print(F("PWM_IN_MIN = ")); Serial.println(PWM_IN_MIN);
          Serial.print(F("PWM_IN_MAX = ")); Serial.println(PWM_IN_MAX);
          EEPROM_writeAnything(PWM_IN_MIN_ADRESS, PWM_IN_MIN);
          EEPROM_writeAnything(PWM_IN_MAX_ADRESS, PWM_IN_MAX);

          PWM_RANGE_SET = true;   // end config mode
        }
        pwm_set_counter = pwm_set_counter + 1;
        delay(1);
      } else {
        pwm_set_counter = 0;
      }
    } //end of !PWM_RANGE_SET
  }

  Serial.println(F("Run mode"));
  MOTOR_SPINNING = false;

  TCCR1A = 0;   // OC1A(D9) and OC1B(D10) normal ports
  TCCR2A = 0;   // OC2A(D11) normal port

  beep_1KHz(100);
  delay(100);
  beep_2KHz(100);
  delay(100);
  beep_3KHz(100);
  delay(100);

  OCR1A = 0;
  OCR1B = 0;
  OCR2A = 0;

  Serial.flush();
  Serial.end();
} //End of setup loop

// Switch to next step functions
void set_next_step() {
  Serial.print(F("set_next_step: ")); Serial.println(sequence_step);
  switch (sequence_step) {
    case 0:
      AH_BL();
      BEMF_C_FALLING();
      break;
    case 1:
      AH_CL();
      BEMF_B_RISING();
      break;
    case 2:
      BH_CL();
      BEMF_A_FALLING();
      break;
    case 3:
      BH_AL();
      BEMF_C_RISING();
      break;
    case 4:
      CH_AL();
      BEMF_B_FALLING();
      break;
    case 5:
      CH_BL();
      BEMF_A_RISING();
      break;
  }
  sequence_step++;
  sequence_step %= 6;
} //end of set_next_step

// main loop
void loop() {
  // if PWM input is higher than PWM_IN_MIN + 115 we start the motor
  if (PWM_INPUT > (PWM_IN_MIN + 115)) {
    MOTOR_SPINNING = true;
    full_stop = false;
    motor_off_counter = 0;
  }

  //////////////////////////Motor is rotating////////////////////////
  if (MOTOR_SPINNING) {
    SET_PWM(PWM_min_value);     // Setup starting PWM with duty cycle = PWM_START_DUTY
    int i = 2200;
    // Manual motor start, without BEMF signal
    while(i > 100) {
      delayMicroseconds(i);
      set_next_step();
      i = i - 20;
    }

    ACSR |= 0x08;               // Enable analog comparator interrupt
    while (MOTOR_SPINNING) {
      PWM_INPUT = constrain(PWM_INPUT, PWM_IN_MIN, PWM_IN_MAX);
      motor_speed = map(PWM_INPUT, PWM_IN_MIN, PWM_IN_MAX, PWM_min_value, PWM_max_value);
      SET_PWM(motor_speed);

      // check for motor stop
      if (PWM_INPUT < (PWM_IN_MIN + 30)) {
        if (motor_off_counter > 1000) { // stop motor if Throttle steady at minimum
          MOTOR_SPINNING = false;
          motor_off_counter = 0;
          PORTD = B00000000;      //Set D2, D3, D4 to to LOW
          TCCR1A = 0;             //OC1A and OC1B normal port
          TCCR1B = 0;             //OC2A - D11 normal port.
        }
        motor_off_counter = motor_off_counter + 1;
      }
    }
  } //end of if MOTOR_SPINNING

  //////////////////////////Motor STOP////////////////////////
  if (!MOTOR_SPINNING) {
    unsigned long currentMillis = millis();
    if ((currentMillis - previousMillis >= 4000) && !full_stop) {
      previousMillis += 4000;
      full_stop = true;
    }

    if (full_stop) {
      if (currentMillis - previousMillis >= 2000) {
        previousMillis += 2000;
        ACSR   = 0x10;            // Disable and clear (flag bit) analog comparator interrupt
        TCCR1A =  0;
        TCCR2A =  0;
        PORTD  = 0x00;            // pins 0 to 7 set to LOW //stop everything
        PORTB  &= 0x31;           // pins D9, D10 and D11 to LOW
//        OCR1A = beeping_PWM_VALUE;
        beep_1KHz(100);
      }
    } else {
       ACSR   = 0x10;            // Disable and clear (flag bit) analog comparator interrupt
       TCCR1A =  0;
       TCCR2A =  0;
       PORTD  = B00000000;       // pins 0 to 7 set to LOW //stop everything
       PORTB  = B00000000;       // pins D9, D10 and D11 to LOW
    }
  } //end of if !MOTOR_SPINNING
} //end of loop
