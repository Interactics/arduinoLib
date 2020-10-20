/*********Encoder Tester************

   Motor's 1CH Encoder Tester Code for arduino Mega.
   Interrupt pin
    - Mega               - pin 2, 3

   Created By Interactics
 *************************************/

void CH_A_ISR();
void CH_B_ISR();

byte ddreMask =
  ~(
    (1 << DDE4) |  // pinMode( 2, INPUT ); // Set to input
    (1 << DDE5)   //  pinMode( 3, INPUT ); // Set to input
  );

byte porteMask =
  (
    (1 << PORTE4) |  // digitalWrite( 2, HIGH ); // Enable the pullup
    (1 << PORTE5)    // digitalWrite( 3, HIGH ); // Enable the pullup
  );

byte pincMask =
  (
    (1 << PINE4) |
    (1 << PINE5)
  );

volatile long Encoder = 0;
uint8_t       Pdat    = PINC & pincMask;
unsigned long time_m;

void setup( void )
{
  DDRE  = DDRE  & ddreMask;       // Configure the pins for input
  PORTE = PORTE | porteMask;      // Enable the pullups

  time_m =  millis();
  
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(2), CH_A_ISR, RISING);
}

void loop() {
  if (millis() - time_m  > 1000) {
    Serial.println(Encoder);
    time_m = millis();
  }
}

void CH_A_ISR() {
  Pdat = PINE & pincMask;
  if (((Pdat >> PINE5) & 0b1) ^ ((Pdat >> PINE4) & 0b1))
    Encoder++;
  else
    Encoder--;
}
