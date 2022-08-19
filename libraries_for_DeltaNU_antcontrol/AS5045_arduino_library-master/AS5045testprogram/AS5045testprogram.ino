#include <AS5045.h>

AS5045 myAS5045(12,14,15); // data, clock, chip select, pins 12, 14, and 15 on the esp32 board.

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  long value;
  value = myAS5045.encoder_value();
  Serial.print("measured value: ");
  Serial.println(value);
  value = myAS5045.encoder_degrees();
  Serial.print("measured degrees: ");
  Serial.println(value);
  if (myAS5045.encoder_error())
  {
    Serial.println("error detected.");
    if (myAS5045.err_value.DECn) Serial.println("DECn error");
    if (myAS5045.err_value.INCn) Serial.println("INCn error");
    if (myAS5045.err_value.COF) Serial.println("COF error");
    if (myAS5045.err_value.OCF) Serial.println("OCF error");
    if (myAS5045.err_value.LIN) Serial.println("LIN error");
  }
  delay(2000);
}
