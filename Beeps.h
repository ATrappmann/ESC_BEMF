void beep_1KHz(int milliseconds)
{
  int x = 0;
  PORTD = B00000100;      //Set D2 (CL) to HIGH and the rest to LOW
  while (x < milliseconds)
  { 
    PORTB = B00000010;      //Set D9 (AH) to HIGH, (BH) to LOW
    delayMicroseconds(50);
    PORTB = B00000000;      //Set D9 (AH) to LOW, (BH) to LOW
    delayMicroseconds(450);
        
    PORTB = B00000100;      //Set D10 (BH) to HIGH, (AH) to LOW 
    delayMicroseconds(50);
    PORTB = B00000000;      //Set D10 (BH) to LOW, (AH) to LOW 
    delayMicroseconds(450);

    x = x + 1;
  }
  PORTD = B00000000;      //Set D2 (CL) to LOW and the rest to LOW
  PORTB = B00000000;      //Set D10 (BH) to LOW, (AH) to LOW 
}

void beep_2KHz(int milliseconds)
{
  int x = 0;
  PORTD = B00000100;      //Set D2 (CL) to HIGH and the rest to LOW
  while (x < milliseconds)
  { 
    for (int i=0; i<2; i++) {
      PORTB = B00000010;      //Set D9 (AH) to HIGH, (BH) to LOW
      delayMicroseconds(50);
      PORTB = B00000000;      //Set D9 (AH) to LOW, (BH) to LOW
      delayMicroseconds(200);
      
      PORTB = B00000100;      //Set D10 (BH) to HIGH, (AH) to LOW 
      delayMicroseconds(50);
      PORTB = B00000000;      //Set D10 (BH) to LOW, (AH) to LOW 
      delayMicroseconds(200);
    }
    x = x + 1;
  }
  PORTD = B00000000;      //Set D2 (CL) to LOW and the rest to LOW
  PORTB = B00000000;      //Set D10 (BH) to LOW, (AH) to LOW 
}

void beep_3KHz(int milliseconds)
{
  int x = 0;
  PORTD = B00000100;      //Set D2 (CL) to HIGH and the rest to LOW
  while (x < milliseconds)
  { 
    for (int i=0; i<3; i++) {
      PORTB = B00000010;      //Set D9 (AH) to HIGH, (BH) to LOW
      delayMicroseconds(50);
      PORTB = B00000000;      //Set D9 (AH) to LOW, (BH) to LOW
      delayMicroseconds(116);
  
      PORTB = B00000100;      //Set D10 (BH) to HIGH, (AH) to LOW 
      delayMicroseconds(50);
      PORTB = B00000000;      //Set D10 (BH) to LOW, (AH) to LOW 
      delayMicroseconds(117);
    }
    x = x + 1;
  }
  PORTD = B00000000;      //Set D2 (CL) to LOW and the rest to LOW
  PORTB = B00000000;      //Set D10 (BH) to LOW, (AH) to LOW 
}
