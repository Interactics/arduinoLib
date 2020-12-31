/**************************************************
 * Encoder Warpping Bounding 
 * Encoder : AS5048A 
 * Absolute Magnetic Encoder : 0 ~ 16384 CPR 
 *************************************************/

#include "AS5048A.h"

#define CPR       16384
#define GEAR_RATE 1      // Direct Drive Motor
#define END_CPR (CPR * GEAR_RATE)

typedef int16_t encoder_t;

AS5048A angleSensor(10);

int16_t prev_val;
int16_t val;

encoder_t wrapBound(const encoder_t* val_prev, const encoder_t* val, const encoder_t bound);

void setup()
{
  Serial.begin(9600);
  angleSensor.init();

  prev_val = angleSensor.getRawRotation();
  val      = prev_val;
}

void loop()
{
  delay(10);

  val = angleSensor.getRawRotation();
  int16_t delta_val = wrapBound(&prev_val, &val, 8000);
  Serial.println(delta_val);
  prev_val = val;
}

encoder_t wrapBound(const encoder_t* val_prev, const encoder_t* val, const encoder_t bound) {
  encoder_t val_delta = *val - *val_prev;
  if     (val_delta > bound)  val_delta -= END_CPR;
  else if (val_delta < -bound) val_delta += END_CPR;

  return val_delta;
}
