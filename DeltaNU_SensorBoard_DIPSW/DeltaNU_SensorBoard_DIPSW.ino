/**************************************************************************/
/*
    @file     DeltaNU_SensorBoard_DIPSW.ino
    @author   Deltanu (sv1dnu/sa0dnu), 2021-2022.
    @license  GNU GPL v3.0 (see LICENSE)
    *** DeltaNU_SensorBoard ***

 * Arduino Nano Every used as the serial Transmitter. RS-485 is used (MAX485).
 * The following absolute encoders can be connected:
 * - HH-12 for Az and/or El, HH12-INC for El 12-bit encoders. Pins for SSI protocol: D4 (Data), D5 (CS for EL), D6 (CS for AZ), D7 (CLK).
 * - AS5048A (SPI) 14-bit encoder for AZ and/or El. Pins: SCK(Clock) D13, CIPO D12, COPI D11. AZ CS pin D8, EL CS pin D10
 * - AS5048B (I2C) 14-bit encoder for AZ and/or El. Pins: SDA D18, SCL D19
 * RX0 (pc5) and TX1 (pc4) pins are the Serial1 interface (used as RS-485). D3 is used as RX-TX control of the serial1.
 * Revision history:
 * v1.0 - Use 4 jumpers (2 for AZ, 2 for EL) for selecting the AZ and EL type of encoder.
 * v0.6 - Removed the Reverse angle counting for SPI. 
 * v0.5 - Supporting all three types of encoders, HH-12 (SSI), AS5048A (SPI) and AS5048B (I2C).
 * v0.4 - Adding AS5048A (SPI) support.
 * v0.3 - Checking diagnostic errors of AS5048B. When error then send angle as 0 value.
 * v0.2 
 * Using AS5048A SPI (pending implementation), AS5048B I2C 14 bit encoders or HH-12 12 bit encoders in the sensorboard with Arduino Nano Every.
  Output from this program is a frame of 6 bytes: del azhi azlo elhi ello 13(hex);
  del is:
  -- D0 for azimuth data only
  -- D1 for elevation data only
  -- D2 for azimuth and elevation data
  
  Azimuth and elevation data is split in into 2 bytes for tansport (high byte and low byte)

  - Configure which data (AZ, EL or both) to measure at: what_to_measure (at the start of the program)
  - Configure the default type of encoders, HH-12 (12 bit) or AS5048 (14 bit)at: az_encoder and el_encoder
  - Set the 4 jumpers (AZ1, AZ2, EL1, EL2) for selecting the type of AZ and EL encoders: 
  Input pins A0 (jmp AZ1) and A1 (jmp AZ2) -> Setting the AZ encoder.
  AZ1 on, AZ2 on  = HH-12 (SSI)
  AZ1 off, AZ2 on = SPI
  AZ1 on, AZ2 off = I2C
  Input pins A2 (jmp EL1) and A3 (jmp EL2) -> Setting the EL encoder
  EL1 on, EL2 on  = HH-12 (SSI)
  EL1 off, EL2 on = SPI
  EL1 on, EL2 off = I2C
  
*/

#define CODE_VERSION "1.0"

#include <ams_as5048b.h>
#include <AS5048A.h>

// #define DEBUG
// *** unit consts
#define U_RAW 1
#define U_TRN 2
#define U_DEG 3
#define U_RAD 4
#define U_GRAD 5
#define U_MOA 6
#define U_SOA 7
#define U_MILNATO 8
#define U_MILSE 9
#define U_MILRU 10

// MAKE YOUR CHOICE
int az_encoder = 2;   // 0 for AZ HH-12, 1 for AS5048A (SPI), 2 for AS5048B (I2C)
int el_encoder = 1;   // 0 for EL HH-12, 1 for AS5048A (SPI), 2 for AS5048B (I2C)
int what_to_measure = 2; // 0 for az only; 1 for el only; 2 for az & el
int refresh_rate = 4 ; // data update rate. Figures between 1(fast) and 5 (slow)are useful

int dip_az1;  // DIP switch AZ pin1
int dip_az2;  // DIP switch AZ pin2
int dip_el1;  // DIP switch EL pin1
int dip_el2;  // DIP switch EL pin2

AMS_AS5048B mysensorI2C;

// Pins for HH-12 SSI protocol
int clockPin = 7; //output for clock
int CSPin_az = 6; //output to Chip Select for az data
int CSPin_el = 5; //output to Chip Select for el data
int inputPin = 4; //input encoderdata (AZ & EL)

AS5048A mySensorSPIaz(8, false);    // AZ CS pin 8, debug: false
AS5048A mySensorSPIel(10, false);    // EL CS pin 10, debug: false

float floatangle = 0;
int inputdata;
int ChipSelect; // used for selecting the encoder
int rxtx_control = 3;
int statusdata = 0; // statusdata from encoder
// int angledata = 0;  // angle data from encoder
long angledata = 0;  // angle data from encoder
long angle = 0; //holds processed angle value
int mask_lo_byte = 0xFF; // 000011111111; mask for getting least significant byte
long anglemask = 262080; // 0x111111111111000000: mask to obtain first 12 digits with position info
int shortdelay = 1; // clock delay
//frame
unsigned char del = 0;      // delimiter frame; 
unsigned char AZ_hi_byte = 0; // azimuth high byte value
unsigned char AZ_lo_byte = 0; // azimuth low byte value
unsigned char EL_hi_byte = 0; // elevation high byte value
unsigned char EL_lo_byte = 0; // elevation low byte value
unsigned char endframe = 0x13;  // end of frame byte


void setup() {

  //Start serial
  Serial.begin(9600);
  Serial1.begin(9600);
  while (!Serial) ; //wait until Serial ready
  delay(500);
  Serial.println("DeltaNU_SensorBoard, v" + String(CODE_VERSION));

  pinMode(A0, INPUT_PULLUP);    // Dip switch AZ1 pin
  pinMode(A1, INPUT_PULLUP);    // Dip switch AZ2 pin
  pinMode(A2, INPUT_PULLUP);    // Dip switch EL1 pin
  pinMode(A3, INPUT_PULLUP);    // Dip switch EL2 pin
  dip_az1 = digitalRead(A0);
  dip_az2 = digitalRead(A1);
  dip_el1 = digitalRead(A2);
  dip_el2 = digitalRead(A3);
  if(dip_az1 == 0 && dip_az2 == 0) {
    az_encoder = 0;   // HH-12
  }
  else if(dip_az1 == 1 && dip_az2 == 0) {
    az_encoder = 1;   // SPI
  }
  else if(dip_az1 == 0 && dip_az2 == 1) {
    az_encoder = 2;   // I2C
  }
  else {
    az_encoder = 3;   // Unused for now
    Serial.println("az_encoder 3, unused!");
  }

  if(dip_el1 == 0 && dip_el2 == 0) {
    el_encoder = 0;   // HH-12
  }
  else if(dip_el1 == 1 && dip_el2 == 0) {
    el_encoder = 1;   // SPI
  }
  else if(dip_el1 == 0 && dip_el2 == 1) {
    el_encoder = 2;   // I2C
  }
  else {
    el_encoder = 3;   // Unused for now
    Serial.println("el_encoder 3, unused!");
  }
  
  switch (az_encoder) {
      case 0:
        Serial.println("Az encoder is: HH-12");
        break;
      case 1:
        Serial.println("Az encoder is: AS5048A (SPI)");
        break;
      case 2:
        Serial.println("Az encoder is: AS5048B (I2C)");
        break;
      default:
        Serial.println("Unknown Az encoder, error!");
  }
  switch (el_encoder) {
      case 0:
        Serial.println("El encoder is: HH-12");
        break;
      case 1:
        Serial.println("El encoder is: AS5048A (SPI)");
        break;
      case 2:
        Serial.println("El encoder is: AS5048B (I2C)");
        break;
      default:
        Serial.println("Unknown El encoder, error!");
  }
  delay(2000);

  if(az_encoder == 0 || el_encoder == 0) {  // If at least one of two encoders is HH-12 (SSI)
    pinMode(clockPin, OUTPUT); // clock
    pinMode(inputPin, INPUT);   // input encoderdata (AZ & EL)
    if (az_encoder == 0) {
      pinMode(CSPin_az, OUTPUT); // Chip Select az encoder
    }
    if (el_encoder == 0) {
      pinMode(CSPin_el, OUTPUT); // Chip Select el encoder
    }
  }

  if(az_encoder == 1) {    // If the AZ encoder is AS5048A (SPI)
    mySensorSPIaz.begin();
  }
  if(el_encoder == 1) {   // If the EL encoder is AS5048A (SPI)
    mySensorSPIel.begin();
  }

  if(az_encoder == 2 || el_encoder == 2) {    // If at least one of the encoders is AS5048B (I2C)
    //Start Wire object. Unneeded here as this is done (optionally) by the AMS_AS5048B object (see lib code - #define USE_WIREBEGIN_ENABLED)
    //Wire.begin();

    //init AMS_AS5048B object
    mysensorI2C.begin();

    //consider the current position as zero
    // mysensorI2C.setZeroReg();

    // Set clock wise counting - sensor counts CCW natively
    //mysensorI2C.setClockWise();

    delay(1000);
    uint8_t addrI2C;
    int addrI2C_int;

    addrI2C = mysensorI2C.addressRegR();
    addrI2C_int = addrI2C;
    Serial.print("I2C address : ");
    Serial.println(uint8_t(addrI2C, HEX));
  }

  pinMode(rxtx_control, OUTPUT); // direction control rs485
  pinMode(13, OUTPUT);           // LED indication on transmit
  delay(100);
  Serial.println("Starting sending the angle data via Serial1 ...");

  delay(2000);
}


// encoder_data(). Reads data from HH-12 encoder (12 bits)
void encoder_data()
{
  int inputstream = 0;
  angledata = 0;
  statusdata = 0;
  digitalWrite(ChipSelect, HIGH); // CS high for disabling encoder
  digitalWrite(clockPin, HIGH);   // CLK high
  // delay(shortdelay);              // delaytime between readings
  digitalWrite(ChipSelect, LOW);  // CS low for enabling encoder, start transfer
  delayMicroseconds(100);  // delay for chip initialization
  digitalWrite(clockPin, LOW);    // start clocking
  delayMicroseconds(100);  // hold low

  for (int x=0; x <18; x++) // clock signal, 18 transitions, output to clock pin
  {
    digitalWrite(clockPin, HIGH);        // clock goes high
    delayMicroseconds(100);       // delaytime between readings
    inputstream = digitalRead(inputPin); // read one bit of data from pin
    angledata = ((angledata << 1) + inputstream); // left shift and add pin value
    digitalWrite(clockPin, LOW);
    delayMicroseconds(100);      // end of one clock cycle
  }

  digitalWrite(ChipSelect, HIGH); // CS high
  digitalWrite(clockPin, HIGH);

  angle = angledata & anglemask; // mask rightmost 6 digits of angledata to zero, into angle

  angle = (angle >> 6); // shift 18-digit angle right 6 digits to form 12-digit value
  #ifdef DEBUG
  Serial.print("Raw Angle HH-12 : ");
  Serial.println(angle);
  floatangle = angle * 0.08789; // angle * (360/4096) == actual degrees
  Serial.print("Degrees Angle HH-12 : ");
  Serial.println(floatangle);
  #endif
}


// read_data_from_I2C(). Reads data from AS5048B I2C (14 bits) encoder
void read_data_from_I2C()
{
  #ifdef DEBUG
  Serial.println("");
  Serial.print("Angle I2C sensor raw : ");
  #endif
  angle = (long)mysensorI2C.angleR(U_RAW, true);
  #ifdef DEBUG
  Serial.println(angle);
  Serial.println(mysensorI2C.angleR(U_RAW, true), DEC);

  Serial.print("Angle I2C degree : ");
  Serial.println(mysensorI2C.angleR(U_DEG, false), DEC);
  #endif
  int diag_reg;
  diag_reg = mysensorI2C.getDiagReg();
  // Serial.println("diag_reg value: " + String(diag_reg));
  if(diag_reg != 1) {
    angle = 0;    // Reset the transmitted angle to value zero while the error occurs.
    Serial.print("Magnet error, diag_reg: " + String(diag_reg));
    switch (diag_reg) {
      case 9:
        Serial.println(" / COMP High, weak magnetic field!");
        break;
      case 5:
        Serial.println(" / COMP low, high magnetic field!");
        break;
      case 3:
        Serial.println(" / COF, CORDIC Overflow!");
        break;
      default:
        Serial.println(" / Unexpected error!");
    }
  }
  
}


// read_data_from_SPI(). Reads data from AS5048A SPI (14 bits) encoder
void read_data_from_SPI()
{
  // Serial.println(" ");
  if(az_encoder == 1) {    // If the AZ encoder is AS5048A (SPI)
    
    word valRaw = mySensorSPIaz.getRawRotation();
    // word valRawRev = 16383 - valRaw;    // Reverse reading of angle
    // angle = AS5048A_MAX_VALUE_360 - (long)valRaw;
    angle = (long)valRaw;
    #ifdef DEBUG
    Serial.print("AZ angle SPI sensor raw : ");
    Serial.println(angle);

    // float val = 359.9 - mySensorSPIaz.getRotationInDegrees2();  // Reverse angle rotation
    val = mySensorSPIaz.getRotationInDegrees2(); 
    Serial.print("\nGot AZ rotation of: ");
    Serial.println(val,2);
    #endif

    String ret_diag = "";
    ret_diag = mySensorSPIaz.getDiagnostic();
    if(ret_diag != "") {
      Serial.println("Az SPI Diagnostics: ");
      Serial.println(ret_diag);
      angle = 0;    // Reset the transmitted angle to value zero while the error occurs.
    }
  }

  if(el_encoder == 1) {    // If the EL encoder is AS5048A (SPI)
    
    word valRaw = mySensorSPIel.getRawRotation();
    // word valRawRev = 16383 - valRaw;    // Reverse reading of angle
    // angle = 16383 - (long)valRaw;
    angle = (long)valRaw;
    #ifdef DEBUG
    Serial.print("EL angle SPI sensor raw : ");
    Serial.println(angle);

    // float val = 359.9 - mySensorSPIel.getRotationInDegrees2();  // Reverse angle rotation
    float val = mySensorSPIel.getRotationInDegrees2();
    Serial.print("\nGot EL rotation of: ");
    Serial.println(val,2);
    #endif

    String ret_diag = "";
    ret_diag = mySensorSPIel.getDiagnostic();
    if(ret_diag != "") {
      Serial.println("El SPI Diagnostics: ");
      Serial.println(ret_diag);
      angle = 0;    // Reset the transmitted angle to value zero while the error occurs.
    }
  }
  
}

void decode_az_data()
{
  // AZ_hi_byte = (angledata) >> 8; // shift 8 bits to form az high byte
  // AZ_lo_byte = angledata & mask_lo_byte;
  AZ_hi_byte = (angle) >> 8; // shift 8 bits to form az high byte
  AZ_lo_byte = angle & mask_lo_byte;
}

void decode_el_data()
{
  // EL_hi_byte = (angledata) >> 8; // shift 8 bits to form az high byte
  // EL_lo_byte = angledata & mask_lo_byte;
  EL_hi_byte = (angle) >> 8; // shift 8 bits to form az high byte
  EL_lo_byte = angle & mask_lo_byte;
//  Serial.print("EL_hi_byte: ");
 // Serial.println(EL_hi_byte);
//  Serial.print("EL_lo_byte: ");
//  Serial.println(EL_lo_byte);
//  Serial.print("floatangle: ");
//  Serial.println(floatangle, 1);  // 1 decimal places
}

void sendframe()
{
  int i;
  char buf [6] = {del, EL_hi_byte, EL_lo_byte, AZ_hi_byte, AZ_lo_byte, endframe};
  // Serial1.flush();
  digitalWrite (13, HIGH); // led on
  for (i = 0; i < 6; i++)
  {
    
    digitalWrite (rxtx_control, HIGH);
    Serial1.write(buf[i]);
    //Serial.println (buf[i]);
    delay(2);
    digitalWrite (rxtx_control, LOW);
    
  }
  digitalWrite (13, LOW); // led off
}


void loop() {
  if ((what_to_measure == 0) ||  (what_to_measure == 2)){
    del = 0xD0;
    digitalWrite (CSPin_el,HIGH); // Disable Chip Select for EL encoder
    if(az_encoder == 0) {   // If AZ encoder is HH-12
      ChipSelect = CSPin_az;  // select az SSI encoder
      encoder_data();         // read encoder angle
    }
    else if(az_encoder == 1) {    // If AZ encoder is SPI
      read_data_from_SPI();   // read SPI encoder angle
    }
    else if(az_encoder == 2) {   // If AZ encoder is I2C
      read_data_from_I2C();         // read I2C encoder angle
    }
    else {
      Serial.println("AZ Encoder unknown. Unexpected error!");
    }
    decode_az_data();       // make bytes for transport
  }
  
  if ((what_to_measure == 1) ||  (what_to_measure == 2)){
    del = 0xD1;
    digitalWrite (CSPin_az,HIGH); // Disable Chip Select for AZ encoder
    if(el_encoder == 0) {   // If EL encoder is HH-12
      ChipSelect = CSPin_el;  // select el encoder 
      encoder_data();         // read encoder angle
    }
    else if(el_encoder == 1) {    // If EL encoder is SPI
      read_data_from_SPI();   // read SPI encoder angle
    }
    else if(el_encoder == 2) {   // If EL encoder is I2C
      read_data_from_I2C();
    }
    else {
      Serial.println("EL Encoder unknown. Unexpected error!");
    }
    decode_el_data();
  }
  if (what_to_measure == 2) {
    del = 0xD2;   
  }
  sendframe();            // send frame
  delay(50 * refresh_rate);

  #ifdef DEBUG
    delay(1000);
  #endif
}
