/*
 * NAME: TestFunctions.h
 */

extern void SET_PWM(byte width_value);
extern void set_next_step();
extern byte sequence_step;

void ask(const char *msg) {
   Serial.print(msg);
   Serial.flush();
   char c = 0x00;
   while ('\n' != c) {
     if (Serial.available() > 0) {
       c = Serial.read();
       Serial.write(c);
     }
   }
}

void testPinFunction() {
   Serial.println(F("testPinFunction"));
   Serial.println(F("AL=D4, BL=D3, CL=D2, AH=D9, BH=D10, CH=D11 all set to LOW"));

   SET_PWM(128);

   ask("continue with AH_BL?");
   AH_BL();
   Serial.println(F("D3=High, D9=PWM\n"));

   ask("continue with AH_CL?");
   AH_CL();
   Serial.println(F("D2=High, D9=PWM\n"));

   ask("continue with BH_CL?");
   BH_CL();
   Serial.println(F("D2=High, D10=PWM\n"));

   ask("continue with BH_AL?");
   BH_AL();
   Serial.println(F("D4=High, D10=PWM\n"));

   ask("continue with CH_AL?");
   CH_AL();
   Serial.println(F("D4=High, D11=PWM\n"));

   ask("continue with CH_BL?");
   CH_BL();
   Serial.println(F("D3=High, D11=PWM\n"));

   ask("continue with all to LOW?");
   PORTD = B00000000;      //Set all to LOW
   TCCR2A =  0;            //OC2A - D11 (CH) normal port.
   TCCR1A =  0x0;          //OC1B - D10 (BH) and OC1A - D9 (AH) normal port.
}

void testRotationSteps() {
  Serial.println(F("testRotationSteps"));
  Serial.println(F("AL=D4, BL=D3, CL=D2, AH=D9, BH=D10, CH=D11 all set to LOW"));
  Serail.println(F("Set AIN0=D6 to 3,3V"));

  SET_PWM(128);
  sequence_step = 0;

  ask("continue with step 0? - AH/BL, C rising - set A0 to LOW, A1 to HIGH");
  set_next_step();
  Serial.println(F("D3=High, D9=PWM, A0 rising\n"));

  ask("continue with step 1? - AH/CL, B falling - set A1 to HIGH, A2 to LOW");
  set_next_step();
  Serial.println(F("D2=High, D9=PWM, A1 falling\n"));

  ask("continue with step 2? - BH/CL, A rising - set A2 to LOW, A0 to HIGH");
  set_next_step();
  Serial.println(F("D2=High, D10=PWM, A2 rising\n"));

  ask("continue with step 3? - BH/AL, C falling - set A0 to HIGH, A1 to LOW");
  set_next_step();
  Serial.println(F("D4=High, D10=PWM, A0 falling\n"));

  ask("continue with step 4? - CH/AL, B rising - set A1 to LOW, A2 to HIGH");
  set_next_step();
  Serial.println(F("D4=High, D11=PWM, A1 rising\n"));

  ask("continue with step 5? - CH/BL, A falling - set A2 to HIGH, A0 to LOW");
  set_next_step();
  Serial.println(F("D3=High, D11=PWM, A2 falling\n"));

  ask("continue with all to LOW?");
  PORTD = B00000000;      //Set all to LOW
  TCCR2A =  0;            //OC2A - D11 (CH) normal port.
  TCCR1A =  0x0;          //OC1B - D10 (BH) and OC1A - D9 (AH) normal port.
}
