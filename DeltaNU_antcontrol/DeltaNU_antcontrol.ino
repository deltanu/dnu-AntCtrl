// DeltaNU antenna control arduino sw for ESP32
// Filename: DeltaNU_antcontrol.cpp
/*
// Revision History:
// 0.12 - Adding Elevation control. (PARK done, pending tracking function update and OLED display update)
// 0.11 - Update function func_track_object to track in periods of X seconds (TRACK_FREQ parameter). 
// 0.10 - Added function func_track_object for antenna tracking of object (currently only the Azimuth). Added Antenna Azimuth and Elevetion LIMITS in features_options.h
//        Temporary: Start tracking by serial command "a". Stop tracking by "s". 
//        Added option for reverse turning of encoders HH-12 (Az and/or El)
// 0.09 - Remove interrupt for button pressed (occasionally crashes). Add function for buttonState (short or long pressed)
//        Button short press: Change tracking object (Sun - Moon). Button Long press: Antenna goes to PARK position (set by PARK_AZ and PARK_EL)
// 0.08 - Add DC motor control, test movements with numbers 1,2,3,4,5. Enhance interrupt to support long pressed button. 
// 0.06 - Add default Lat/Lon and grid square in features_options.h 
//        Grid square is used for calculating object position. If Grid square is invalid then the default coords are used.
// ESP32 connected with HH-12 and HH-12INC (SPI connected) and OLED (I2C connected)
// SPI connected PINS: MISO(Data)-> GPIO 12 (both HH-12 and HH-12INC), CLK (both HH-12 and HH-12INC) -> GPIO14, CS1 (HH-12) -> GPIO15, CS2 (HH-12INC) -> GPIO05
// Attention: GPIO05 and GPIO15 are in state HIGH during boot -> use pull-up
//    GPIO12 is in state LOW during boot -> use pull-down
// ESP32 connected with OLED display via I2C. SDA -> GPIO21, SCL -> GPIO22
// Get UTC time from NTP server "pool.ntp.org" via WiFi at boot, show it at the first line of OLED
// Show Sun and Moon position (Az and El), toggle between the two by press of button (connected to pin GPIO32) as interrupt.
// Keep latest tracking object showing on display in internal flash memory. Int value 0 is Sun, value 1 is moon.
*/
#define CODE_VERSION "0.12"

#include "features_options.h" // Edit this file, temp usage
// For HH-12 (SPI connected) and OLED (I2C connected)
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <hh12.h>

// For Sun position
#include "sunpos.h"
#include <math.h>
// For Moon position
#include <moon2.h>
// For NTP server connection
#include <WiFi.h>
#include "time.h"
// For using the internal flash memory
#include <Preferences.h>

Preferences preferences;  //  create an instance of the library called preferences

const int maxObj = 1;   // Maximum number of tracking objects (First is 0 zero)
const int buttonPin = TRACKOBJ_PIN; // See definition in features_options.h
int trackingObject = 0; // Used to select the object (Sun or Moon) for tracking. 0 = Sun, 1 = Moon
int objectState = 0; // Default is 0 (the Sun)

// Definitions for input button
int buttonState1 = 0;     // current state of the button
int lastButtonState1 = 0; // previous state of the button
int startPressed = 0;    // the moment the button was pressed
int endPressed = 0;      // the moment the button was released
int holdTime = 0;        // how long the button was hold

// Below parameters is for NTP time 
const char *ssid     = WIFI_SSID;
const char *password = WIFI_PASS;

const char* ntpServer = NTP_SERVER;
const long  gmtOffset_sec = GMTOFFSET_SEC;  // We want UTC time
const int   daylightOffset_sec = DAYLIGHTOFFSET_SEC;

struct tm timeinfo;
int lDay = 0;
int lMonth = 0;
int lYear = 0;
int lHours = 0;
int lMinutes = 0;
int lSeconds = 0;

// cTime udtTime = {2021, 04, 03, 4, 07, 30};   // Year, Month, Day, UTC hours, Mins, Secs
// cLocation udtLocation = {18.0686,59.3293};  // Longitude, Latitude Stockholm
cLocation udtLocation = {DEFAULT_LONGITUDE, DEFAULT_LATITUDE};  // Initialize with the default coords set in features_options.h
cSunCoordinates coord;
double latitude = 0; // = DEFAULT_LATITUDE;
double longitude = 0; // = DEFAULT_LONGITUDE;
char grid[10] = DEFAULT_GRID;

float sun_azim = 0;
float sun_elev = 0;
double moon_azimuth = 0;
double moon_elevation = 0;

float obj_azim = 0;
float obj_elev = 0;
int startTrackTime = 0;
int currentTrackTime = 0;
int statusTrackingObject = 0;

// See OLED params in features_options.h

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_RESET);
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

// Definitions for DC motor control
const int freq = PWM_FREQ;
const int pwmChanAz = PWM_CHAN1;
const int pwmChanEl = PWM_CHAN2;
const int resolution = PWM_RESOLUTION;
int dutyCycleAz = DUTY_CYCLE;
int dutyCycleEl = DUTY_CYCLE;


const int ledPin = 2; // onboard LED connected to digital pin 2, used as a heartbeat

// Below is the Azimuth and Elevation HH-12 connections, HSPI is used
const int hspi_clk = hspi_clk_pin;  // Common for both AZ and EL
const int az_hspi_cs = az_hspi_cs_pin;  // CS for AZ
const int el_hspi_cs = el_hspi_cs_pin;   // CS for EL
const int hspi_miso = hspi_miso_pin; // Common for both AZ and EL

float az_floatAngle = 0;
float el_floatAngle = 0;

hh12 azimuth_hh12;
hh12 elevation_hh12;

const int az_park = PARK_AZ;
const int el_park = PARK_EL;
int az_moving_state = 0;  // now obsolete, to delete
int moving_state[] = {0, 0};  // 1st elem -> azimuth, 2nd elem -> elevation
// int park_state = 0;     // now obsolete, to delete
int park_state[] = {0, 0};    // 1st elem -> azimuth, 2nd elem -> elevation

int tracking_state = 0;


int incomingByte = 0; // TEMP!!! for incoming serial data


void serial_menu_motor_test(int read_input) {     // Temp function to test motor
// send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    if(incomingByte >= 48 && incomingByte <= 57) {
      read_input = incomingByte - 48; //convert ASCII code of numbers to 1,2,3 ... 9
    }
    else if ((incomingByte >= 97 && incomingByte <= 122) || incomingByte == 10) {
      read_input = incomingByte; // char a-z or LF
    }
    else {
      Serial.println("Invalid command!");
    }
    
    switch (read_input) { 
      case 1:         // if input=1 ....... motors turn forward
        Motor_forward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz); // third param 0 for azimuth motor
       break;
      case 2:         // if input=2 ....... motors turn fast forward
        Motor_fast_forward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // third param 0 for azimuth motor
        break;
      case 3:         // if input=3 ....... motors turn backward
        Motor_backward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // third param 0 for azimuth motor
        break;
      case 4:         // if input=4 ....... motors turn stop
        Motor_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz);  // third param 0 for azimuth motor
        park_state[0] = 0;  // To avoid continuing to park position
        break;
      case 5:         // if input=5 ....... motors soft stop
        Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);   // third param 0 for azimuth motor
        park_state[0] = 0;  // To avoid continuing to park position
        break;
      case 97:  // a , start tracking
        Serial.print("case_a: ");
        Serial.println(read_input);
        Serial.println("Start tracking!");
        tracking_state = 1;   // Enable tracking
        statusTrackingObject = 1; // Enable current period tracking
        startTrackTime = millis();  // Store the start time of tracking
        func_track_object(obj_azim, obj_elev);
        break;
      case 101: // e
        Serial.print("case_e: ");
        Serial.println(read_input);
        break;
      case 115: // s , Stop tracking
        Serial.print("case_s: ");
        Serial.println(read_input);
        tracking_state = 0;   // Disable tracking
        Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);   // third param 0 for azimuth motor
        break;
    }
    delay(300);
 //   read_input=0;
  }
}


// *** Antenna goes to PARK posision (Az only for now)
void goto_park() {
  #if defined(ENABLE_AZIMUTH)
    int l_azIntAngle = round(az_floatAngle);    // Convert float to int
    if(az_park < l_azIntAngle) {   // Move CCW (backward) 
      if(moving_state[0] != 2) {    // if not already moving azimuth the same direction
        Motor_backward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);    // third param 0 for azimuth motor
        park_state[0] = 1; // Moving azimuth to park
      }
      Serial.println("Parking azimuth backward... ");
    }
    else if((az_park > l_azIntAngle)) {   // Move CW (forward) 
      if(moving_state[0] != 1) {        // if not already moving azimuth the same direction
        Motor_fast_forward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);    // third param 0 for azimuth motor
        park_state[0] = 1; // Moving azimuth to park
      }
      Serial.println("Parking azimuth forward... ");
    }
    else if((az_park == l_azIntAngle)) {  // Reached PARK azimuth position
      if(moving_state[0] != 0) { // If azimuth Motor is already moving
        Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the azimuth motor
        park_state[0] = 0; // Stop azimuth moving, parked.
        Serial.println("Reached azimuth park position! ");
      }
      else {
        Serial.println("PARK, azimuth motor already stopped! ");
      }
    }
    else {
      Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);
      park_state[0] = 0;
      Serial.println("Error PARK! Should never reach here. Stopping Motor. ");
    }
  #else
    park_state[0] = 0; // Disable azimuth moving, parked.
  #endif
  #if defined(ENABLE_ELEVATION)
    int l_elIntAngle = round(el_floatAngle);    // Convert float to int
    if(el_park < l_elIntAngle) {   // Move Down
      if(moving_state[1] != 2) {    // if not already moving elevation the same direction
        Motor_backward(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);    // third param 1 for elevation motor
        park_state[1] = 1; // Moving elevation to park
      }
      Serial.println("Parking elevation Down... ");
    }
    else if((el_park > l_elIntAngle)) {   // Move CW (forward) 
      if(moving_state[1] != 1) {        // if not already moving elevation the same direction
        Motor_fast_forward(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);    // third param 1 for elevation motor
        park_state[1] = 1; // Moving elevation to park
      }
      Serial.println("Parking elevation Up... ");
    }
    else if((el_park == l_elIntAngle)) {  // Reached PARK elevation position
      if(moving_state[1] != 0) { // If elevation Motor is already moving
        Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop the elevation motor
        park_state[1] = 0; // Stop elevation moving, parked.
        Serial.println("Reached park elevation position! ");
      }
      else {
        Serial.println("PARK, elevation motor already stopped! ");
      }
    }
    else {
      Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);
      park_state[1] = 0;
      Serial.println("Error PARK! Should never reach here. Stopping Motor. ");
    }
  #else
    park_state[1] = 0; // Disable elevation moving, parked.
  #endif 
}


// *** Get the antenna motors track the object  (Az only for now)
void func_track_object(int trackobj_azim, int trackobj_elev) {
// void func_track_object(float trackobj_azim) {
  int l_azIntAngle = round(az_floatAngle);    // Convert float to int
  Serial.print("l_azIntAngle is: ");
  Serial.println(l_azIntAngle);
  int l_trackobjAzim = round(trackobj_azim);    // Convert float to int
  Serial.print("l_trackobjAzim is: ");
  Serial.println(l_trackobjAzim);
  int l_elIntAngle = round(el_floatAngle);    // Convert float to int
  Serial.print("l_elIntAngle is: ");
  Serial.println(l_elIntAngle);
  int l_trackobjElev = round(trackobj_elev);    // Convert float to int
  Serial.print("l_trackobjElev is: ");
  Serial.println(l_trackobjElev);
  int l_diffTrackTime = 0;

  Serial.println("Entered func_track_object... ");

  currentTrackTime = millis();
  l_diffTrackTime = currentTrackTime - startTrackTime;
  Serial.print("l_diffTrackTime is: ");
  Serial.println(l_diffTrackTime);

  if (l_diffTrackTime >= TRACK_FREQ*1000) {   // Period passed since last track, do it again.
    statusTrackingObject = 1;
    Serial.println("Next period reached, align antenna to object... ");
  }

  if (statusTrackingObject != 0) {
    // Track the azimuth
    if(l_trackobjAzim < ANT_AZ_MAX && l_trackobjAzim > ANT_AZ_MIN) {
      if(l_trackobjAzim < l_azIntAngle) {   // Move CCW (backward)
        statusTrackingObject = 1; // Keep the current tracking period as active
        if(moving_state[0] != 2) {    // if not moving already the same direction
          Motor_backward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);    // third param 0 for azimuth motor
        }
      }
      else if((l_trackobjAzim > l_azIntAngle)) {   // Move CW (forward)
        statusTrackingObject = 1; // Keep the current tracking period as active
        if(moving_state[0] != 1) {        // if not moving already the same direction
          Motor_fast_forward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);    // third param 0 for azimuth motor
        }
      }
      else if((l_trackobjAzim == l_azIntAngle)) {  // Reached object position
        statusTrackingObject = 0; // Tracking done for this period
        if(moving_state[0] != 0) {    // If Motor is already moving
          Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the motor
          Serial.println("Reached object position! ");
        }
        else {
          Serial.println("Already at object position, motor already stopped! ");
        }
      }
      else {
        Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);
        Serial.println("Error! Should never reach here. Stopping Motor. ");
      }
    }
    else {    // Tracking object out of antenna limits, stop the motors.
      Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the motor
      Serial.println("Tracking object out of antenna limits, motor stopped! ");
      tracking_state = 0;   // Disable tracking
    }
    
    startTrackTime = currentTrackTime;  // Update the start time of the period
  }
} 


// *** Motor Functions
/*
void az_motor_init() {    // Obsolete, to be removed
  pinMode(MOTOR_AZ1_PIN, OUTPUT);
  pinMode(MOTOR_AZ2_PIN, OUTPUT);

// Stop motor
  digitalWrite(MOTOR_AZ1_PIN, LOW);
  digitalWrite(MOTOR_AZ2_PIN, LOW);

// configure PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the A2 to be controlled
  ledcAttachPin(MOTOR_AZ2_PIN, pwmChannel);
}
*/

void motor_init(int motor_pin1, int motor_pin2, int pwmCh) {
  pinMode(motor_pin1, OUTPUT);
  pinMode(motor_pin2, OUTPUT);

// Stop motor
  digitalWrite(motor_pin1, LOW);
  digitalWrite(motor_pin2, LOW);

// configure PWM functionalitites
  // ledcSetup(pwmChannel, freq, resolution);
  ledcSetup(pwmCh, freq, resolution);

  // attach the channel to the pin2 to be controlled
  ledcAttachPin(motor_pin2, pwmCh);
}


// *** Motor_Stop function. 
// m1 is the pin1 of DC motor, m2 is the pin2 of DC motor
// az_el_ind sets the AZ (param 0) or the EL (param 1) moving_state
void Motor_Stop(int m1, int m2, int az_el_ind, int pwmCh) {         // function of Motor stop
  Serial.println("Motor stopped");
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
  ledcWrite(pwmCh, 0);  
  // az_moving_state = 0;  // Az motor stopped
  moving_state[az_el_ind] = 0;
}

// *** Motor_Soft_Stop function. 
// m1 is the pin1 of DC motor, m2 is the pin2 of DC motor
// az_el_ind sets the AZ (param 0) or the EL (param 1) moving_state
void Motor_Soft_Stop(int m1, int m2, int az_el_ind, int pwmCh, int *dutyCycle) {    // function of Motor Soft stop
  Serial.println("Soft stopping... ");
  Serial.print("dutyCycle is: ");
  Serial.println(*dutyCycle);
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
  while (*dutyCycle <= 255 && *dutyCycle >= 0){
    ledcWrite(pwmCh, *dutyCycle);   
    *dutyCycle = *dutyCycle - 5;
    delay(100);
  }
  Serial.print("Final dutyCycle is: ");
  Serial.println(*dutyCycle);
  // az_moving_state = 0;  // Az motor stopped
  moving_state[az_el_ind] = 0;
}

// *** Motor_fast_forward function. 
// m1 is the pin1 of DC motor, m2 is the pin2 of DC motor
// az_el_ind sets the AZ (param 0) or the EL (param 1) moving_state
void Motor_fast_forward(int m1, int m2, int az_el_ind, int pwmCh, int *dutCycl) {          //function of fast_forward 
  int l_dutyCycle = *dutCycl;
  if (az_el_ind == 0) {
    Serial.println("Moving az Fast Forward");
  }
  else {
    Serial.println("Moving el Fast UP");
  }
  // Stop motor first
//  digitalWrite(A1, LOW);
//  digitalWrite(A2, LOW);
//  ledcWrite(pwmChannel, 0);
  Motor_Soft_Stop(m1, m2, az_el_ind, pwmCh, &l_dutyCycle);
  delay(DIR_DELAY);
  // Move forward
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
//  dutyCycle = 255; 
  *dutCycl = 255;
//  ledcWrite(pwmCh, dutyCycle);
  ledcWrite(pwmCh, *dutCycl);
  // az_moving_state = 1;  // Az motor moving forward CW
  moving_state[az_el_ind] = 1;  // Motor moving forward CW / Up
}

// *** Motor_forward function. 
// m1 is the pin1 of DC motor, m2 is the pin2 of DC motor
// az_el_ind sets the AZ (param 0) or the EL (param 1) moving_state
void Motor_forward(int m1, int m2, int az_el_ind, int pwmCh, int *dutCycl) {          //function of forward in medium speed
  int l_dutyCycle = *dutCycl;
  if (az_el_ind == 0) {
    Serial.println("Moving az Forward");
  }
  else {
    Serial.println("Moving el UP");
  }
  // Stop motor first
//  digitalWrite(A1, LOW);
//  digitalWrite(A2, LOW);
//  ledcWrite(pwmChannel, 0);'
  Motor_Soft_Stop(m1, m2, az_el_ind, pwmCh, &l_dutyCycle);
  delay(DIR_DELAY);
  // Move forward
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
//  dutyCycle = 255-DUTY_CYCLE;
  *dutCycl = 255-DUTY_CYCLE;
//  ledcWrite(pwmCh, dutyCycle);
  ledcWrite(pwmCh, *dutCycl);
  // az_moving_state = 1;  // Az motor moving forward CW
  moving_state[az_el_ind] = 1;  // Motor moving forward CW / Up
}

// *** Motor_backward function. 
// m1 is the pin1 of DC motor, m2 is the pin2 of DC motor
// az_el_ind sets the AZ (param 0) or the EL (param 1) moving_state
void Motor_backward(int m1, int m2, int az_el_ind, int pwmCh, int *dutCycl) {         //function of backward
  int l_dutyCycle = *dutCycl;
  if (az_el_ind == 0) {
    Serial.println("Moving az Backwards with reduced speed");
  }
  else {
    Serial.println("Moving el Down with reduced speed");
  }
    // Stop motor first
//  digitalWrite(A1, LOW);
//  digitalWrite(A2, LOW);
//  ledcWrite(pwmChannel, 0);
  Motor_Soft_Stop(m1, m2, az_el_ind, pwmCh, &l_dutyCycle);
  delay(DIR_DELAY);
  // Move backward
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW); 
//  dutyCycle = DUTY_CYCLE;
  *dutCycl = DUTY_CYCLE;
//  ledcWrite(pwmCh, dutyCycle);
  ledcWrite(pwmCh, *dutCycl);
  // az_moving_state = 2;  // Az motor moving backward CCW
  moving_state[az_el_ind] = 2;  // Motor moving backward CCW / Down
}


/// int Park_Position(int m1, int m2, 

// *** Functions ***
int getTimeNTP() {
//  struct tm timeinfo;
  delay(100);
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return 0;
  }
  Serial.println(&timeinfo, "%d %m %Y %H:%M:%S");
  return 1;
}

void getSunPos() {
  cTime udtTime;
  sun_azim = 0;
  sun_elev = 0;
  
  udtTime.iDay = timeinfo.tm_mday;
  udtTime.iMonth = timeinfo.tm_mon + 1; // tm_mon is from 0 to 11
  udtTime.iYear = timeinfo.tm_year + 1900;  // tm_year is the year since 1900
  udtTime.dHours = timeinfo.tm_hour;
  udtTime.dMinutes = timeinfo.tm_min;
  udtTime.dSeconds = timeinfo.tm_sec;
  Serial.println(" ");
  sunpos(udtTime, udtLocation, &coord);
  Serial.print("Latitude: ");
  Serial.print(udtLocation.dLatitude);
  Serial.print(" Longitude: ");
  Serial.print(udtLocation.dLongitude);
  Serial.println();

  Serial.print("Sun Zenith angle: ");
  Serial.print(coord.dZenithAngle);
   Serial.print(" Sun Elevetion angle: ");
  sun_elev = 90-coord.dZenithAngle; // Elev is 90 degress minus the Zenith angle
  Serial.print(sun_elev);  
  Serial.println();
  Serial.print(" Sun Azimuth: ");
  sun_azim = coord.dAzimuth;
  Serial.print(sun_azim);
  Serial.println();
  Serial.println();
  Serial.println();
}

void getMoonPos() {
  cTime udtTime;
  double RA, Dec, topRA, topDec, LST, HA, dist;
  
  udtTime.iDay = timeinfo.tm_mday;
  udtTime.iMonth = timeinfo.tm_mon + 1; // tm_mon is from 0 to 11
  udtTime.iYear = timeinfo.tm_year + 1900;  // tm_year is the year since 1900
  udtTime.dHours = timeinfo.tm_hour;
  udtTime.dMinutes = timeinfo.tm_min;
  udtTime.dSeconds = timeinfo.tm_sec;
  Serial.println(" ");
  moon2(udtTime.iYear, udtTime.iMonth, udtTime.iDay, (udtTime.dHours + (udtTime.dMinutes / 60.0) + (udtTime.dSeconds / 3600.0)), udtLocation.dLongitude, udtLocation.dLatitude, &RA, &Dec, &topRA, &topDec, &LST, &HA, &moon_azimuth, &moon_elevation, &dist);

  Serial.print("Latitude: ");
  Serial.print(udtLocation.dLatitude);
  Serial.print(" Longitude: ");
  Serial.print(udtLocation.dLongitude);
  Serial.println();

  Serial.print(" Moon Elevetion angle: ");
  Serial.print(moon_elevation);  
  Serial.println();
  Serial.print(" Moon Azimuth: ");
  Serial.print(moon_azimuth);
  Serial.println();
  Serial.print(" Moon RA: "); // Right Ascension
  Serial.print(RA);  
  Serial.print(", Moon Dec: "); // Declination
  Serial.print(Dec);
  Serial.println();
  Serial.print(" Moon topRA: ");  // Topocentric Right Ascension
  Serial.print(topRA);  
  Serial.print(", Moon Dec: ");   // Topocentric Declination
  Serial.print(topDec);
  Serial.println();
  Serial.println();
  Serial.println();
}

void init_display() {
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize the I2C addr 0x3C (for the 64x48)
  // init done
  Serial.print("Setup initial display");
  display.display(); // shows Initial default symbol on display
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();
  Serial.println("Setup after clear display");
  Serial.printf("Version: %s ", CODE_VERSION);

  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("deltanu");
  display.setCursor(0,10);
  display.println("Ant ctrl");
  display.setCursor(0,20);
  display.print("Ver: ");
  display.println(CODE_VERSION);
  display.setCursor(0,30);
  display.print("qth:");
  display.println(grid);
  display.display();
  delay(2000);
  display.clearDisplay();
}


void display_angle(float azAngle, float elAngle) {
  //clear display
//  display.clearDisplay();

  // Display Az degrees
//  display.setTextSize(1);
//  display.setCursor(0,0);
//  display.print("Ant pos");
  display.setTextSize(1);
  display.setCursor(0,10);
  display.print("Az:");
  display.print(azAngle, 1);
  display.print(" ");
  display.setTextSize(1);
  display.write(167); // Ascii code of Degree symbol

  display.setTextSize(1);
  display.setCursor(0,20);
  display.print("El:");
  display.print(elAngle, 1);
  display.print(" ");
  display.setTextSize(1);
  display.write(167); // Ascii code of Degree symbol
  
//  display.display();
}


void display_sunPos(float azAngle, float elAngle) {
  //clear display
//  display.clearDisplay();

  // Display Az degrees
  display.setTextSize(1);
  display.setCursor(0,30);
  display.print("Sun Pos:");
  display.setTextSize(1);
  display.setCursor(0,40);
  display.print(azAngle, 1);
  display.print(" ");
  display.print(elAngle, 1);

//  display.display();
}

void display_moonPos(float azAngle, float elAngle) {
  //clear display
//  display.clearDisplay();

  // Display Az degrees
  display.setTextSize(1);
  display.setCursor(0,30);
  display.print("Moon Pos:");
  display.setTextSize(1);
  display.setCursor(0,40);
  display.print(azAngle, 1);
  display.print(" ");
  display.print(elAngle, 1);

//  display.display();
}


void display_time() {
  char wholeTime[11];
  lDay = timeinfo.tm_mday;
  lMonth = timeinfo.tm_mon + 1; // tm_mon is from 0 to 11
  lYear = timeinfo.tm_year + 1900;  // tm_year is the year since 1900
  lHours = timeinfo.tm_hour;
  lMinutes = timeinfo.tm_min;
  lSeconds = timeinfo.tm_sec;
  snprintf(wholeTime, sizeof(wholeTime), "%02d%02d%02d%02d%02d", lMonth, lDay, lHours, lMinutes, lSeconds);
  display.setTextSize(1);
  display.setCursor(0,0); // 1st line of OLED, only 10 characters MMDDhhmmss
  display.print(wholeTime);
}

void clear_DISPLAY() {
  display.clearDisplay();
}
void display_DISPLAY() {
  display.display();
}

// Button input
// Short press: Change tracking object
// Long press: Go to Park position. 180-Az, 89-El
void updateState_button1() {
  // the button has been just pressed
  if (buttonState1 == HIGH) {
      startPressed = millis();

  // the button has just been released
  } else {    // buttonState1 == LOW
      endPressed = millis();
      holdTime = endPressed - startPressed;

      if (holdTime >= 50 && holdTime < LONGPRESS_TIME) {
        Serial.println("Button was short pressed"); 
        if (objectState >= maxObj) {   // For now we can have only 0 (Sun) and 1 (Moon) values
          objectState = 0;  // Go to first object (Sun)
        }
        else {
          objectState++;  // Increase by 1
        }
      }

      if (holdTime >= LONGPRESS_TIME && holdTime < 10000) { // Long pressed button should be not more than 10 seconds
          Serial.println("Button was LONG pressed");
          goto_park();
      }

  }
}
// End of updateState_button1()

/*
// Interrupt function
// void IRAM_ATTR down1() {
void button_down1() {
  attachInterrupt(digitalPinToInterrupt(buttonPin), button_up1, RISING);
  secondv = millis();
  if((secondv - firstv) >= 2000) {    // If pressed more than 2 seconds 
    firstv = secondv;
    Serial.println("Long press");
  }
  else {
    Serial.println("Short press");
    if (objectState >= maxObj) {   // For now we can have only 0 (Sun) and 1 (Moon) values
      objectState = 0;  // Go to first object (Sun)
    }
    else {
      objectState++;  // Increase by 1
    }
  }
}
*/


void init_test_motor() {
  delay(DIR_DELAY);
  Serial.println("select direction of movement");
  Serial.println("1.forward");
  Serial.println("2.fast_forward");
  Serial.println("3.backward");
  Serial.println("4.stop");
  Serial.println("5.soft_stop");
  Serial.println("a.start tracking");
  Serial.println("s.stop tracking");
}
// End functions


void setup()
{
  Serial.print("Begin setup");
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT); // visual signal of chip initialization, onboard LED
  digitalWrite(ledPin, HIGH); // signal start of transfer with LED
  delay(2000);
  digitalWrite(ledPin, LOW);

  //Create a namespace called "gpio"
  preferences.begin("gpio", false);
  pinMode(buttonPin, INPUT);
  // Set buttonPin as interrupt, assign interrupt function and set RISING mode
//  attachInterrupt(digitalPinToInterrupt(buttonPin), button_up1, RISING);  // to REMOVE

  // read the last tracking object from flash memory
  objectState = preferences.getInt("state", 0); 
  Serial.printf("Object state before reset: %d \n", objectState);
  // set the tracking object to the last stored state
  trackingObject = objectState; 

//  udtLocation = {DEFAULT_LONGITUDE, DEFAULT_LATITUDE};  // It has been initialized in the beginning of the sketch
  //strcpy(grid, DEFAULT_GRID);
  if(strlen(grid) == 6) {  // grid is 6 characters long
    grid2deg(grid, &longitude, &latitude);
    Serial.printf(" grid: %s ,", grid);
    Serial.printf(" length of grid: %d ,", strlen(grid));
    Serial.printf(" longitude: %f ,", longitude);
    Serial.printf(" latitude: %f \n", latitude);
    udtLocation.dLatitude = latitude;
    udtLocation.dLongitude = longitude;
  }
  else {
    Serial.printf(" grid square is invalid, using default coords: [%s] \n", grid);
  }
  
  init_display();

  pinMode(ledPin, OUTPUT); // visual signal of I/O to chip: heartbeat
  ////pinMode(clockPin, OUTPUT); // SCK HSPI
  ////pinMode(CSnPin, OUTPUT); // CSn HSPI -- has to toggle high and low to signal chip to start data transfer
  ////pinMode(inputPin, INPUT); // SDA HSPI
  azimuth_hh12.initialize(hspi_clk, az_hspi_cs, hspi_miso);
  elevation_hh12.initialize(hspi_clk, el_hspi_cs, hspi_miso);

  //connect to WiFi in order to get time from NTP server once
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" CONNECTED");
  Serial.print(" IP address: ");
  Serial.println(WiFi.localIP());
  //init and get the time
  Serial.print(" Init Time: ");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  delay(200);
  if(getTimeNTP() == 0)
  {
    Serial.println(" Failed to get time from NTP server! ");
  }

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println(" ");
  Serial.printf("Disconnecting from %s ", ssid);
  while (WiFi.status() == WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" DISCONNECTED");
  // End connections to WiFi 

  // az_motor_init();
  #ifdef ENABLE_AZIMUTH
    motor_init(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, pwmChanAz); // Init AZ motor
  #endif
  #ifdef ENABLE_ELEVATION
    motor_init(MOTOR_EL1_PIN, MOTOR_EL2_PIN, pwmChanEl); // Init EL motor
  #endif
  init_test_motor();    // TEMP!!!
  
  Serial.print("End setup");
}

int  input = 0;   // TEMP!!!

void loop()
{
  serial_menu_motor_test(input);
  input = 0;   // TEMP!!!

  if((park_state[0] != 0) || (park_state[1] != 0)) {
    goto_park();  // Going to parking position
  }

  // Check if button1 has been pressed (short or long pressed)
  buttonState1 = digitalRead(buttonPin); // read the button1 input
  if (buttonState1 != lastButtonState1) { // button state changed
     updateState_button1();
  }
  lastButtonState1 = buttonState1;        // save state for next loop

  if (trackingObject != objectState) {
    Serial.println("Changing object tracking ");
    // change the tracking object state
    trackingObject = objectState;

    // save the tracking object state in flash memory
    preferences.putInt("state", objectState);

    Serial.printf("Object State saved: %d \n", objectState);
    
  }


// CSn needs to cycle from high to low to initiate transfer. Then clock cycles. As it goes high
// again, data will appear on sda

  digitalWrite(el_hspi_cs, HIGH); // Disable the EL encoder before using the AZ
  #if defined(OPTION_REVERSE_AZ_HH12_AS5045)
    az_floatAngle = 360.0 - azimuth_hh12.heading();  // Read the azimuth angle from the hh-12
  #else
    az_floatAngle = azimuth_hh12.heading();  // Read the azimuth angle from the hh-12
  #endif
  Serial.print("az_floatAngle degrees: "); // and, finally, print it.
  Serial.println(az_floatAngle, 1);  // 1 decimal places

  digitalWrite(az_hspi_cs, HIGH); // Disable the AZ encoder before using the EL
  #if defined(OPTION_REVERSE_EL_HH12_AS5045)
    el_floatAngle = 360.0 - elevation_hh12.heading();  // Read the elevation angle from the hh-12 inc
  #else
    el_floatAngle = elevation_hh12.heading();  // Read the elevation angle from the hh-12 inc
  #endif
  Serial.print("el_floatAngle degrees: "); // and, finally, print it.
  Serial.println(el_floatAngle, 1);  // 1 decimal places

  clear_DISPLAY();
  display_angle(az_floatAngle, el_floatAngle);

////  packeddata = 0; // reset both variables to zero so they don't just accumulate
////  angle = 0;

  if(getTimeNTP() == 1) {
    display_time();
    Serial.printf("trackingObject: %d \n", trackingObject);
    if (trackingObject == 0) {  // Sun tracking
      #ifdef FEATURE_SUN_TRACKING
        getSunPos();
        display_sunPos(sun_azim, sun_elev);
        obj_azim = sun_azim;
        obj_elev = sun_elev;
      #endif
    }
    else if (trackingObject == 1) {  // Moon tracking
      #ifdef FEATURE_MOON_TRACKING
        getMoonPos();
        display_moonPos(moon_azimuth, moon_elevation);
        obj_azim = (float)moon_azimuth;
        obj_elev = (float)moon_elevation;
      #endif
    }
    else {
      Serial.println(" Invalid tracking object. ");
      tracking_state = 0; // Disable tracking
    }
  
    if(tracking_state != 0) {
      if (az_floatAngle > float(ANT_AZ_MIN) && az_floatAngle < float(ANT_AZ_MAX) && el_floatAngle > float(ANT_EL_MIN) && el_floatAngle < float(ANT_EL_MAX)) {
        func_track_object(obj_azim, obj_elev);
      }
      else {
        Serial.println(" Error: ANT position out of limits! Disabling tracking... ");
        tracking_state = 0;   // Disable tracking
        #if defined(ENABLE_AZIMUTH)
          Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);
        #endif
        #if defined(ENABLE_ELEVATION)
          Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);
        #endif
      }
    }
  }
  else {
    Serial.println(" Failed to get time fron NTP server. ");
  }
  display_DISPLAY();
  
  delay(2000); // wait for 2 second for no particular reason
}
