/*
 * NAME: PinFunctions.h
 */

#define PWM_max_value      255
#define PWM_min_value      30

 /*
  * On each step we know that the next 0 cross will be rising or falling and if it will be
  * on coil A, B or C. With these funcstions we select that according to the step of the sequence
  */
 void BEMF_A_RISING(){
   ADCSRA = (0 << ADEN);     // Disable the ADC module
   ADCSRB = (1 << ACME);     // MUX select for negative input of comparator
   ADMUX = 2;                // Select A2 as comparator negative input
   ACSR |= 0x03;             // Set interrupt on rising edge
 }
 void BEMF_A_FALLING(){
   ADCSRA = (0 << ADEN);     // Disable the ADC module
   ADCSRB = (1 << ACME);     // MUX select for negative input of comparator
   ADMUX = 2;                // Select A2 as comparator negative input
   ACSR &= ~0x01;            // Set interrupt on falling edge
 }
 void BEMF_B_RISING(){
   ADCSRA = (0 << ADEN);     // Disable the ADC module
   ADCSRB = (1 << ACME);     // MUX select for negative input of comparator
   ADMUX = 1;                // Select A1 as comparator negative input
   ACSR |= 0x03;             // Set interrupt on rising edge
 }
 void BEMF_B_FALLING(){
   ADCSRA = (0 << ADEN);     // Disable the ADC module
   ADCSRB = (1 << ACME);     // MUX select for negative input of comparator
   ADMUX = 1;                // Select A1 as comparator negative input
   ACSR &= ~0x01;            // Set interrupt on falling edge
 }
 void BEMF_C_RISING(){
   ADCSRA = (0 << ADEN);     // Disable the ADC module
   ADCSRB = (1 << ACME);     // MUX select for negative input of comparator
   ADMUX = 0;                // Select A0 as comparator negative input
   ACSR |= 0x03;             // Set interrupt on rising edge and clear ACI
 }
 void BEMF_C_FALLING(){
   ADCSRA = (0 << ADEN);     // Disable the ADC module
   ADCSRB = (1 << ACME);     // MUX select for negative input of comparator
   ADMUX = 0;                // Select A0 as comparator negative input
   ACSR &= ~0x01;            // Set interrupt on falling edge
 }

 /*
  * On each step we change the digital pins to be HIGH or LOW or to be PWM or no-PWM
  * depending on which step of the sequence we are
  */
 //D9 PWM and D3 HIGH.
 void AH_BL(){
   PORTD = B00001000;      //Set D3 (BL) to HIGH and the rest to LOW
   TCCR2A =  0;            //OC2A - D11 normal port.
   TCCR1A =  0x81;         //OC1A - D9 (AH) compare match noninverting mode, downcounting ,PWM 8-bit
 }
 //D9 PWM and D2 HIGH
 void AH_CL(){
   PORTD = B00000100;      //Set D2 (CL) to HIGH and the rest to LOW
   TCCR2A =  0;            //OC2A - D11 normal port.
   TCCR1A =  0x81;         //OC1A - D9 (AH) compare match noninverting mode, downcounting ,PWM 8-bit
 }
 //D10 PWM and D2 HIGH
 void BH_CL(){
   PORTD = B00000100;      //Set D2 (CL) to HIGH and the rest to LOW
   TCCR2A =  0;            //OC2A - D11 normal port.
   TCCR1A =  0x21;         //OC1B - D10 (BH) compare match noninverting mode, downcounting ,PWM 8-bit
 }
 //D10 PWM and D4 HIGH
 void BH_AL(){
   PORTD = B00010000;      //Set D4 (AL) to HIGH and the rest to LOW
   TCCR2A =  0;            //OC2A - D11 normal port.
   TCCR1A =  0x21;         //OC1B - D10 (BH) compare match noninverting mode, downcounting ,PWM 8-bit
 }
 //D11 PWM and D4 HIGH
 void CH_AL(){
   PORTD = B00010000;      //Set D4 (AL) to HIGH and the rest to LOW
   TCCR1A =  0;            // OC1A and OC1B normal port
   TCCR2A =  0x81;         // OC2A - D11 (CH) compare match noninverting mode, downcounting ,PWM 8-bit
 }
 //D11 PWM and D3 HIGH
 void CH_BL(){
   PORTD = B00001000;      //Set D3 (BL) to HIGH and the rest to LOW
   TCCR1A =  0;            // OC1A and OC1B normal port
   TCCR2A =  0x81;         // OC2A - D11 (CH) compare match noninverting mode, downcounting ,PWM 8-bit
 }

 /*
  * This function will only change the PWM values according to the received width_value
  * that is given by the PWM read on pin D8
  */
void SET_PWM(byte width_value) {
  //We keep the range of PWM between min and max (8 bit value)
  if(width_value < PWM_min_value)    width_value  = PWM_min_value;
  if(width_value > PWM_max_value)    width_value  = PWM_max_value;
  OCR1A  = width_value;                   // Set pin 9  PWM duty cycle
  OCR1B  = width_value;                   // Set pin 10 PWM duty cycle
  OCR2A  = width_value;                   // Set pin 11 PWM duty cycle
}
