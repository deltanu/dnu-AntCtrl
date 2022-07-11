// DeltaNU antenna control arduino sw for ESP32 (by SV1DNU)
// Filename: DeltaNU_antcontrol.ino
// The inspiration of this project was initiated by the excellent oe5jfl antenna controller. But wanted to add more features, so it was decided to use the ESP32 microcontroller.
// Example codes by K3NG rotator controller, ON4CDU Remote Sensor boards, alex_chang satellite tracker, SlashDevin GPS library have been considered.
// ================================
// ESP32 connected with HH-12 and HH-12INC (SPI connected) and LCD 4x20 (I2C connected). Also connected to AZ H-bridge and EL H-bridge for antenna motor control.
// SPI connected PINS: MISO(Data)-> GPIO 12 (both HH-12 and HH-12INC), CLK (both HH-12 and HH-12INC) -> GPIO14, CS1 (HH-12) -> GPIO15, CS2 (HH-12INC) -> GPIO05
// Attention: GPIO05 and GPIO15 are in state HIGH during boot -> use pull-up
//    GPIO12 is in state LOW during boot -> use pull-down
// ESP32 connected with LCD display via I2C. SDA -> GPIO21, SCL -> GPIO22
// GPIO pins configuration under file features_options.h
// Get UTC time from NTP server "pool.ntp.org" via WiFi at boot, show it at the first line of LCD. 
// Connect GPS (e.g. NEO6M) in UART2. If GPS connected and WiFi not available, then the time and location is fetch by GPS NMEA messages.
// If neither WiFi nor GPS available, the time can be set manually by command line in UART0 or via the config menu. 
// Show Sun, Moon position (Az and El), 4 satellites and Park position of the Antenna. Toggle between these by press of "shift" button (connected to pin GPIO34).
// Upon booting the TLE keplerian data of the 4 pre-defined satellites are fetched using the WiFi. 
// Keep latest tracking object showing on display in internal EEPROM memory.
// Use of "Auto" button to start automatic tracking of the current object. Stop automatic tracking by pressing "Auto" again. If the current object is a satellite and
// is not above horizon, then show the remaining time before getting "visible". Show also the rising Az and Max El of the next orbit.
// If absolute encoders do not change value after a predefined time-out (TIMEOUT_STOP parameter in seconds under features_options.h)
// which means that the antenna is not moving, then stop any current movement and cancel the automatic tracking. 
// Pressing the "+" button shows the Grid locator (Maidenhead) after fetcing it from GPS (if connected). 
// By long-pressing (more than 4 seconds) the "Shift" button the config menu appears. There the following can be set:
// Menu> AZ offset, El offset, Automatic Tracking Freq (in seconds), Antenna speed (3 speeds using PWM), Tracking Accuracy (0.1 or 1 degrees),
// Menu> Grid locator, SAT1 Norad No, SAT2 Norad No, SAT3 Norad No, SAT4 Norad No, Setting Time Manually.
// The parameter values can be changed by pressing the "+" and "-" buttons. Move to the next digit by pressing "Auto". 
// To store any menu parameter long-press the "Auto" button. All Menu parameters are stored in internal EEPROM except the Time. If a Satellite Norad No is changed then
// the u-controller connects to WiFi and fetches the latest keps of the new satellite. 
// The UART0 is used to receive serial commands by e.g. P.C. Easycomm I protocol commands are accepted for moving the AZ and EL DC motors.
// =============================
/*
// Revision History:
//*** Note: Due to the GPIO12 (MISO pin of SPI) connected to 'Data' of the HH-12 absolute encoders, it was preventing the ESP32 to boot. 
//***      The burning of the efuses cured the issue. See sketch "Burn_efuse1.ino". (found in https://github.com/espressif/arduino-esp32/issues/3621)  
//***      Alternatively the burning of efuse could be performed by command line: "components/esptool_py/esptool/espefuse.py set_flash_voltage 3.3V -p COMXXX"
// 0.51 - Added the constant STARTPRESS_TIME in features_options.h that is used for the buttons trigger. Also, cleanup unused libraries.
// 0.50 - Added readme intructions in the header above. 
// 0.49 - Motor_Soft_Stop function: The delay within the loop where the dutyCycle is decreasing was reduced from 100 to 20 milliseconds. Faster stopping
// 0.48 - If time is fetched from NTP server (via WiFi), pressing of the '+' button to show the Grid Loc will fetch the grid from GPS if available. 
//        If GPS not available the symbol "!" will be shown next to the grid so that we know not to trust the locator. 
// 0.47 - Updated the URL of CELESTRAK from where the TLEs are fetched. Also updated definition of satURL[4][40] (it was satURL[4][30]), since the updated URL is a longer string.
// 0.46 - Store the date-time manually as the system time through the menu. After setting the time by '+' and 'Auto' buttons, store it by pressing the '-' button. 
//          Useful when there is neither WiFi access nor GPS connected.
// 0.45 - Press the '+' button to show the grid locator (instead of date) on LCD display. Press '+' again to show date.
// 0.44 - Set time manually through Serial via command line. Format: ts:20220131-102354 
// 0.43b - (Skipping 0.43) External GPS support. Testing with GPS NEO-6M via UART-1. Change GPS library, <NMEAGPS.h> part of NeoGPS. Fetch time from GPS if NTP not available. 
//          Fetch locator from GPS. Use internal RTC for keeping the clock running (even when GPS signal/data lost). Next to time in LCD set symbol "N" for NTP time, "Y" for GPS time 
//          else "!" for no time source
// 0.43 - Add external GPS support. Testing with GPS NEO-6M via UART-2. Use grid square fetched from GPS (if connected) but do not store it to EEPROM. Use "+" button in main
//          for displaying the grid locator from GPS (if connected). lcd_time(timeSrc) function updated. Parameter timeSrc gets 'N' for NTP time, 'Y' for GPS time and '!' for internal RTC
// 0.42 - Add external GPS support. Testing with GPS NEO-6M via UART-2. Just print the latitude, longitude, time and grid square to Serial. 
// 0.41 - Add support for SPID commands. New function process_rot2prog_command(). SKIPPED for now.
// 0.40 - check_serial(), ignore the <space> as end of byte string in case the string starts with 'w' (setting WiFi credentials)
// 0.39 - Changed the H-bridge from L9110 to MD13S. Changed from [Serial.print("az_floatAngle degrees: ");] to [Serial.print("#AZ");] to be used as easycomm format.
// 0.38 - Correcting bug when soft_stoping motor. If already stopped should not attempt to stop again with initial dutyCycle. 
// 0.37 - While moving motors check if absolute encoder values are changing within a duration period. If they don't and if TIMEOUT_STOP is reached, then stop motors and display ERROR.
// 0.36 - After calling process_easycom_command(): Printing to serial the response of commands AZ and EL.
// 0.35 - Accepting command 'restart' via serial for restarting the Microcontroller. 
// 0.34 - If WiFi connection fails, you can change the credentials via serial command, "ws:<ssid>" and "wp:<passwd>". New functions set_WiFi_cred() and update_wifiCred()
//        The easycom commands via serial can only be given one at a time (<space> separator is not detected any more). 
// 0.33 - Adding more text in menu of "Tracking Freq", "Ant speed", "Track accuracy" and LCD status text while updating TLEs and time.
// 0.32 - Move the WiFi Connection, fetching of satellites TLE and time to the function fetchTLEandTime(). 
//        After satellite ID change and AutoButton is long pressed, the satID is stored in memory and the fetchTLEandTime() function is called.
// 0.31 - Changing the sat IDs in the config menu and storing them in flash memory.
// 0.30 - Retrieving the Sat ID from flash memory. 
// 0.29 - Adding 4 satellites in config menu.
// 0.28 - Two tracking accuracy options, 1 or 0.1 degree. Updating func_track_object and request_command functions.
// 0.27 - Adding error messages if WiFi and / or NTP time connection fails. Adding menu object of tracking accuracy (0 means 0.1 degree, 1 means 1 degree accuracy)
// 0.26 - Improving Motor moving functions. Updating func_track_object. Function lcd_AntStatus updated to show * when auto tracking is enabled
// 0.25 - Adding antenna speed (via PWM) in menu. H: high, M: medium, L:low
// 0.24 - Adding tracking frequency in menu (in seconds)
//        Fixing bug when auto tracking could not be disabled when ANT is out of limits.
// 0.23 - Adding Antenna azimuth and elevation offset.
// 0.22 - In the config menu, move the cursor by pressing Enter (Auto) button. Use the '+' and '-' buttons to modify the char where the cursor it. 
//        Updated grid locator is stored and saved to EEPROM when the shift button is pressed (going to next menu element).
//        Fixed a bug in the upcoming time of satellites. Remove clear_lcd(); from each loop() run. 
// 0.21 - Replacing OLED to LCD 4x20 display (also via I2C).
// 0.20 - Adding buttons '+' and '-'. "Shift" button long-pressed enters the config menu. 1) Set grid locator, 2) Set WiFi credentials.
// 0.19 - Adding Buttons: Shift, Auto. The "Shift" (short pres) button changes the tracking object. The "Auto" button starts or stops the auto tracking. 
// 0.18 - Adding Easycomm protocol commands. GOTO command, STOP command via serial
// 0.17 - Disable tracking when changing tracking object. 
// 0.16 - show Sun, Moon, sat1, sat2, sat3, sat4 by toggle the button. If satellite visible, then show Az, El values. If not visible, show 
//        when it will be the next pass.
// 0.15 - Adding satellite tracking, 4 satellites. Need to import Sgp4.h . Add getTLE.ino and Predict.ino in the same folder as this code   
//        Modified Predict.ino in order to handle overflow of nextpassEpoch in case of Geostationary satellites   
// 0.14 - When auto tracking, show the -> or -< for azimuth, ^ or v for elevation
// 0.13 - Tracking function updated to include elevation, too. 
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
*/

#define CODE_VERSION "0.51"

#include <string.h>
#include "features_options.h" // Edit this file, temp usage
// For HH-12 (SPI connected) and OLED (I2C connected)
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>   // Adafruit_BusIO is used, too
#include <hh12.h>

#include <HardwareSerial.h>

// For NMEAGPS use
#include <GPSport.h>  // part of NeoGPS library

//------------------------------------------------------------
// For the NeoGPS example programs, "Streamers" is common set
//   of printing and formatting routines for GPS data, in a
//   Comma-Separated Values text format (aka CSV).  The CSV
//   data will be printed to the "debug output device".
// If you don't need these formatters, simply delete this section.
#include <Streamers.h>

// #include <TinyGPSPlus.h>
// #include <TinyGPS.h>
#include <NMEAGPS.h>  // part of NeoGPS library
#include "time.h"
#include <maidenhead.h>

//------------------------------------------------------------
// Check that the NeoGPS config files are set up properly

#if !defined( NMEAGPS_PARSE_RMC )
  #error You must uncomment NMEAGPS_PARSE_RMC in NMEAGPS_cfg.h!
#endif

#if !defined( GPS_FIX_TIME )
  #error You must uncomment GPS_FIX_TIME in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_LOCATION )
  #error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_SPEED )
  #error You must uncomment GPS_FIX_SPEED in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_SATELLITES )
  #error You must uncomment GPS_FIX_SATELLITES in GPSfix_cfg.h!
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
  #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

//------------------------------------------------------------
// This object parses received characters
//   into the gps.fix() data structure

static NMEAGPS  gps;

//------------------------------------------------------------
//  Define a set of GPS fix information.  It will
//  hold on to the various pieces as they are received from
//  an RMC sentence.  It can be used anywhere in your sketch.

static gps_fix  fix;

// For Sun position
#include "sunpos.h"
#include <math.h>
// For Moon position
#include <moon2.h>
// For Satellite tracking
#include <Sgp4.h>
// For NTP server connection
#include <WiFi.h>

// #include <TimeLib.h>
// #include <ESP32Time.h>  // For internal RTC
// For using the internal flash memory
#include <Preferences.h>

Preferences preferences;  //  create an instance of the library called preferences

// *** For satellite tracking
// const int timeZone = +1;  // Your Time zone
const int timeZone = TIMEZONE;  // Your Time zone set in features_opions.h
char TLE[500];                //Variable to store satellite TLEs.
char satnames[4][30] = {"AO-7","ZARYA","FO-29","QO-100"};// Names of satellites. OBSOLETE, to DELETE!!!
char satNameTLE[4][30];
char satelName[4][7] = {"", "", "", ""};
char satURL[4][40] = {"/satcat/tle.php?CATNR=07530","/satcat/tle.php?CATNR=25544",
     "/satcat/tle.php?CATNR=24278","/satcat/tle.php?CATNR=43700"}; // URL of Celestrak TLEs for satellites (In same order as names).
char TLE1[4][70]; char TLE2[4][70];
// float myLat = 59.3293; float myLong = 18.0686; 
float myAlt = 10;   // My location altitude.
const int numSats = 4;    // Number of satellites to track.
Sgp4 sat;
char satID[4][6] = {"07530", "25544", "24278", "43700"};  // Default satellites
char tmp_satID[4][6] = {"", "", "", ""};
String satID1_mem = "";
String satID2_mem = "";
String satID3_mem = "";
String satID4_mem = "";
// const String sub_URL = "/satcat/tle.php?CATNR=";   // w w w.celestrak.c o m
const String sub_URL = "/NORAD/elements/gp.php?CATNR=";   // w w w.celestrak.c o m
String tmp_URL = "";
String temp_satID = "";

int i; int k; int SAT; int nextSat; int AZstart; long passEnd; int satVIS;
char satname[] = " ";
int passStatus = 0;
char server[] = CELESTRAK_IP;    //Web address to get TLE (CELESTRAK)
int  year1; int mon; int day1; int hr; int min1; double sec; int today;
long nextpassEpoch; long upcomingPasses[4];
int status;
unsigned long unixtime;
unsigned long testTime = 1620232078;
unsigned long timeNow = 0;
int nextRise[4];  // holds the seconds remaining until the next rise
double azimStart;
double elevMax;
// *** end of satellite tracking definitions

// *** For Internal RTC ***
// ESP32Time rtcESP32;

// *** For GPS connection
// HardwareSerial MySerial_GPS(2); // Use UART2 for external GPS.
// The TinyGPS++ object
// TinyGPSPlus gps;
// TinyGPS gps;
unsigned long timeNowGPS = 0;
unsigned long epochTimeGPS = 0;
struct tm timeDetails;
// struct tm timeinfoGPS;
// tmElements_t timeinfoGPS;
// *** end of GPS definitions
boolean nTP_flag = false;
boolean gPS_flag = false;
boolean selfTime_flag = false;

int gpsStatus = 0;  // 1==GPS, 2==internal RTC, 0==nothing
boolean gpsLOC_flag = false;
boolean gpsTime_flag = false;
// unsigned long startGPSdata = 0;
unsigned long checkGPSdata = 0;
boolean showLoc_flag = false;

struct timeval tv;
unsigned long epochTimeMan = 0;
struct tm timeInfoMan = {0};


const int maxObj = 6;   // Maximum number of tracking objects (First is 0 zero), Moon + Sun + 4 satellites + Park position
const int buttonShiftPin = SHIFT_PIN; // See definition in features_options.h
int trackingObject = 0; // Used to select the object (Sun or Moon) for tracking. 0 = Sun, 1 = Moon
int objectState = 0; // Default is 0 (the Sun)

const int maxMenuObj = 10;   // Max number of menu configs (First is zero).
int menuObject = 0;   // Used to select the object within the menu config. 

// Definitions for Shift button
int buttonStateShift = 0;     // current state of the button
int lastbuttonStateShift = 0; // previous state of the button
int startPressed = 0;    // the moment the button was pressed
int endPressed = 0;      // the moment the button was released
int holdTime = 0;        // how long the button was hold
int shiftButtonMode = 0; // Keeps track between Main function and Config Menu

// Definitions for Auto button
const int buttonAutoPin = AUTO_PIN; // See definition in features_options.h
int buttonAutoState1 = 0;     // current state of the Auto button
int lastButtonAutoState1 = 0; // previous state of the Auto button
int startAutoPressed = 0;    // the moment the Auto button was pressed
int endAutoPressed = 0;      // the moment the Auto button was released
int autoButtonMode = 0;     // 0 for starting Auto tracking, 1 for pressing enter within config menu.
int blinkInd = 0;           // Used in the menu to show the blinking cursor

// Definitions for Plus button
const int buttonPlusPin = PLUS_PIN; // See definition in features_options.h
int buttonPlusState1 = 0;     // current state of the Plus button
int lastButtonPlusState1 = 0; // previous state of the Plus button
int startPlusPressed = 0;    // the moment the Plus button was pressed
int endPlusPressed = 0;      // the moment the Plus button was released
int plusButtonMode = 0;

// Definitions for Minus button
const int buttonMinusPin = MINUS_PIN; // See definition in features_options.h
int buttonMinusState1 = 0;     // current state of the Minus button
int lastButtonMinusState1 = 0; // previous state of the Minus button
int startMinusPressed = 0;    // the moment the Minus button was pressed
int endMinusPressed = 0;      // the moment the Minus button was released
int minusButtonMode = 0;


// Below parameters is for NTP time 
//const char *ssid     = WIFI_SSID;
//const char *password = WIFI_PASS;
//const String ssid = WIFI_SSID;
//const String password = WIFI_PASS;
String ssid_mem = "";
String passwd_mem = "";
char tmp_ssid[25] = "";
char tmp_passwd[25] = "";
char wifi_ssid[25] = WIFI_SSID;
char wifi_passwd[25] = WIFI_PASS;
// Initialize the WiFi client library
WiFiClient client;

const char* ntpServer = NTP_SERVER;
const long  gmtOffset_sec = GMTOFFSET_SEC;  // We want UTC time
const int   daylightOffset_sec = DAYLIGHTOFFSET_SEC;

unsigned long epochTime = 0;
struct tm timeinfo;
int lDay = 0;
int lMonth = 0;
int lYear = 0;
int lHours = 0;
int lMinutes = 0;
int lSeconds = 0;

char tmp_time[18] = "";
char manual_time[16] = {'2', '0', '2', '2', '0', '2', '1', '9', '-', '1', '4', '2', '0', '4', '5'};  // Initializing manual_time , 20220219-142045

// cTime udtTime = {2021, 04, 03, 4, 07, 30};   // Year, Month, Day, UTC hours, Mins, Secs
// cLocation udtLocation = {18.0686,59.3293};  // Longitude, Latitude Stockholm
cLocation udtLocation = {DEFAULT_LONGITUDE, DEFAULT_LATITUDE};  // Initialize with the default coords set in features_options.h
cSunCoordinates coord;
double latitude = 0; // = DEFAULT_LATITUDE;
double longitude = 0; // = DEFAULT_LONGITUDE;
char grid[7] = DEFAULT_GRID;
char tmp_grid[7] = "";
String grid_mem = "";
char gridGPS[7];  // Initialized in Setup()

float sun_azim = 0;
float sun_elev = 0;
double moon_azimuth = 0;
double moon_elevation = 0;

float obj_azim = 0;
float obj_elev = 0;
int startTrackTime = 0;
int currentTrackTime = 0;
int statusTrackingObject[] = {0, 0};  // 1st element for azimuth status, 2nd element for elevation status

// LCD display 4x20 via I2C (address 0x27)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// See OLED params in features_options.h

/*
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_RESET);
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
*/

int int_trackFreq = TRACK_FREQ;
char trackFreq[4] = "";
char tmp_trackFreq[4] = "";
String trackFreq_mem = "";

const int menuDutyCycle[3] = {LOW_DUTY_CYCLE, MID_DUTY_CYCLE, HIGH_DUTY_CYCLE}; // 3 different duty cycles, 0:Low, 1:Mid, 2:High
// menuDutyCycle[0] = LOW_DUTY_CYCLE;  // Low
// menuDutyCycle[1] = MID_DUTY_CYCLE; // Mid
// menuDutyCycle[2] = HIGH_DUTY_CYCLE; // High
int indexDC = 0;  // Default is Low speed
char tmp_indexDC;
char indexDC_mem;

const int trackAccuFactor[2] = {10, 1}; // For 0.1 or 1 degree accuracy
int trackAccuInd = 0;  // Default is 0.1 degree accuracy
char tmp_trackAccuInd;
char trackAccuInd_mem;

// Definitions for DC motor control
const int freq = PWM_FREQ;
const int pwmChanAz = PWM_CHAN1;
const int pwmChanEl = PWM_CHAN2;
const int resolution = PWM_RESOLUTION;
// int dutyCycleAz = DUTY_CYCLE;
// int dutyCycleEl = DUTY_CYCLE;
int dutyCycleAz = menuDutyCycle[indexDC];
int dutyCycleEl = menuDutyCycle[indexDC];

const int ledPin = 2; // onboard LED connected to digital pin 2, used as a heartbeat

// Below is the Azimuth and Elevation HH-12 connections, HSPI is used
const int hspi_clk = hspi_clk_pin;  // Common for both AZ and EL
const int az_hspi_cs = az_hspi_cs_pin;  // CS for AZ
const int el_hspi_cs = el_hspi_cs_pin;   // CS for EL
const int hspi_miso = hspi_miso_pin; // Common for both AZ and EL

float az_floatAngle = 0;
float el_floatAngle = 0;
// float prev_azAngle = 0; 
// float prev_elAngle = 0;
int az_intAngle = 0;
int el_intAngle = 0;
int prev_intAzAngle = 0; // Used to check if absolute encoder value is changing while motor is moving
int prev_intElAngle = 0;
float azimuth_offset = -0.1;
char azOffset[7] = "";
char tmp_azOffset[7] = "";
String azOffset_mem = "";
float elevation_offset = -0.1;
char elOffset[7] = "";
char tmp_elOffset[7] = "";
String elOffset_mem = "";

hh12 azimuth_hh12;
hh12 elevation_hh12;

int startMovingTime[] = {0, 0}; // 1st elem -> azimuth, 2nd elem -> elevation
int currentMovingTimeAz = 0;
int diffMovingTimeAz = 0;
int currentMovingTimeEl = 0;
int diffMovingTimeEl = 0;
int timeout_stop = TIMEOUT_STOP;

const int az_park = PARK_AZ;
const int el_park = PARK_EL;
int az_moving_state = 0;  // now obsolete, to delete
int moving_state[] = {0, 0};  // 1st elem -> azimuth, 2nd elem -> elevation
// int park_state = 0;     // now obsolete, to delete
int park_state[] = {0, 0};    // 1st elem -> azimuth, 2nd elem -> elevation

int goto_state[] = {0, 0};    // 1st elem -> azimuth, 2nd elem -> elevation
int tracking_state = 0;   // 1 if autoTracking enabled, 0 if disabled
short autoTrack_state = 0;

float goto_az_value = 0;
float goto_el_value = 0;

int incomingByte = 0; // TEMP!!! for incoming serial data
byte incoming_serial_byte = 0;
byte control_port_buffer[COMMAND_BUFFER_SIZE];
int control_port_buffer_index = 0;
String command1;
int CONTROL_PORT0 = 0;
// ### End of definitions

void serial_menu_motor_test(int read_input) {     // Temp function to test motor
// send data only when you receive data:
  if (read_input == 0) { // Check serial for single alphanumeric input
    if (Serial.available() > 0) {
      // read the incoming byte:
      incomingByte = Serial.read();
      read_input = incomingByte;
    }
  }
  
  if(read_input >= 48 && read_input <= 57) {  
    read_input = read_input - 48; //convert ASCII code of numbers to 1,2,3 ... 9
  }
  else if ((read_input >= 65 && read_input <= 90) || read_input == 10) {
    read_input = read_input; // char A-Z or LF
  }
  else {
    Serial.print("Invalid command: ");
    Serial.println(read_input);
    return;
  }
    
    switch (read_input) { 
      case 0:
        ;   // do nothing
        break;
      case 1:         // if input=1 ....... motors turn forward
        Motor_forward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz); // third param 0 for azimuth motor
        break;
      case 2:         // if input=2 ....... motors turn fast forward
        Motor_fast_forward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // third param 0 for azimuth motor
        break;
      case 3:         // if input=3 ....... motors turn backward
        Motor_backward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // third param 0 for azimuth motor
        break;
      case 4:         // if input=4 ....... motors turn fast backward
        Motor_fast_backward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // third param 0 for azimuth motor
        break;
      case 5:         // if input=5 ....... motors soft stop
        Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);   // third param 0 for azimuth motor
        park_state[0] = 0;  // To avoid continuing to park position
        break;
      case 6:         // if input=5 ....... motors turn stop
        Motor_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz);  // third param 0 for azimuth motor
        park_state[0] = 0;  // To avoid continuing to park position
        break;
      case 65:  // A , start tracking
        Serial.print("case_A: ");
        Serial.println(read_input);
        Serial.println("Start tracking!");
        tracking_state = 1;   // Enable tracking
        
        statusTrackingObject[0] = 1; // Enable current period azimuth tracking
        statusTrackingObject[1] = 1; // Enable current period elevation tracking
        startTrackTime = millis();  // Store the start time of tracking
        func_track_object(obj_azim, obj_elev);
        break;
      case 69: // E
        Serial.print("case_E: ");
        Serial.println(read_input);
        break;
      case 83: // S , Stop tracking
        Serial.print("case_S: ");
        Serial.println(read_input);
        tracking_state = 0;   // Disable tracking
        if(moving_state[0] != 0) { // If azimuth Motor is already moving
          Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);   // third param 0 for azimuth motor
        }
        park_state[0] = 0;  // To avoid continuing to park position
        if(moving_state[1] != 0) { // If elevation Motor is already moving
          Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);   // third param 1 for elevation motor
        }
        park_state[1] = 0;  // To avoid continuing to park position
        break;
      default:
        Serial.print("Invalid single char command: ");
        Serial.println(read_input);
    }
    delay(100);
 //   read_input=0;
 
}


// Function restart_esp32(), restarts the microcontroller.
void restart_esp32() {
  Serial.println("Restarting in 10 seconds...");
  Motor_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz);  // Stop Azimuth motor  
  Motor_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl);  // Stop Elevation motor
  delay(10000);
  ESP.restart();
}


// *** Function check_serial() checks for commands by serial interface for moving the motors
void check_serial() {

  char return_string[100] = "";
  char tmp_port_string[8] = "";
  boolean flag_endSpace = false;
  while (Serial.available() > 0) {
    incoming_serial_byte = Serial.read(); 
//    if ((incoming_serial_byte > 96) && (incoming_serial_byte < 123)) {  // uppercase it
//      incoming_serial_byte = incoming_serial_byte - 32;
//    }
    // if it's an easycom command add it to the buffer if it's not a line feed, carriage return, or space
    if(incoming_serial_byte == 'w') { // if setting the WiFi credentials then ignore the "space" 0x20 as end of byte string
      flag_endSpace = true;
    }
 //   if ((incoming_serial_byte != 10) && (incoming_serial_byte != 13) && (incoming_serial_byte != 32)) {
    // if ((incoming_serial_byte != 10) && (incoming_serial_byte != 13)) {   // if neither line feed nor carriage return
    if ((incoming_serial_byte != 10) && (incoming_serial_byte != 13) && ((incoming_serial_byte != 32) || flag_endSpace) ) { 
      control_port_buffer[control_port_buffer_index] = incoming_serial_byte;
      control_port_buffer_index++;
      memcpy(tmp_port_string, control_port_buffer, 7);
    }

    // if it is an Easycom command and we have a space, line feed, or carriage return, process it
 //   if ((incoming_serial_byte == 10) || (incoming_serial_byte == 13) || (incoming_serial_byte == 32)) {
    // if ((incoming_serial_byte == 10) || (incoming_serial_byte == 13)) {   // If line feed or carriage return
    if ((incoming_serial_byte == 10) || (incoming_serial_byte == 13) || ((incoming_serial_byte == 32) && !flag_endSpace) ) {
      if (control_port_buffer_index > 1) {
        if(control_port_buffer[0] == 'w') {   // command to set WiFi credentials, 'ws' (ssid) or 'wp' (pass)
          flag_endSpace = false;
          set_WiFi_cred(control_port_buffer, control_port_buffer_index);
          if((strlen(tmp_ssid) != 0) && (strlen(tmp_passwd) != 0)) {
            update_wifiCred(tmp_ssid, tmp_passwd);
            wifi_ssid[0] = 0;
            wifi_passwd[0] = 0;
            strncpy(wifi_ssid, tmp_ssid, sizeof(tmp_ssid));
            strncpy(wifi_passwd, tmp_passwd, sizeof(tmp_passwd));
            fetchTLEandTime();
          }
          
        }
        else if (control_port_buffer[0] == 't') {   // Command to set Time manually, ts:YYYYMMDD-hhmmss
          set_Time_command(control_port_buffer, control_port_buffer_index);
          if(strlen(tmp_time) != 0) {
            set_time_manual(tmp_time);
          }
        }
        else if ((control_port_buffer_index == 7) && (strcmp(tmp_port_string, "restart") == 0)) {  // 'restart' command
          Serial.print("Serial command request: ");
          Serial.println(tmp_port_string);
          restart_esp32();
        }
        else {
          process_easycom_command(control_port_buffer, control_port_buffer_index, CONTROL_PORT0, return_string);
        }
        
        Serial.println("Command value request: ");
        // Serial.println(return_string);
        Serial.printf("%s \n", return_string);
      }
      // The following is temporary for test the OLD single char serial interface
      else {  // Single character command
        Serial.print("check_serial(), Single Character command. ");
        serial_menu_motor_test(control_port_buffer[0]);
      }
      // end of temporary test
      clear_command_buffer();
      break;
    }
  }
}


void clear_command_buffer(){

  control_port_buffer_index = 0;
  control_port_buffer[0] = 0;
}



// Function set_WiFi_cred. Interprets commands "ws:<WiFi ssid>" and "wp:<WiFi passwd>"
void set_WiFi_cred(byte * wifi_command_buffer, int wifi_command_buffer_index) {
  char tempString[25] = "";

  switch (wifi_command_buffer[1]) { // look at the second character of the command
    case 's':  // ssid
        if (wifi_command_buffer_index > 5) {
          // empty tmp_ssid
          tmp_ssid[0] = '\0';
          memcpy(tempString, wifi_command_buffer, wifi_command_buffer_index);
          tempString[wifi_command_buffer_index] = '\0';
          Serial.print("wifi_command_buffer ssid tempString: ");
          Serial.println(tempString);
          strncpy(tmp_ssid, tempString+3, wifi_command_buffer_index-3);
          //strcat(return_string,tempString);
          Serial.println(tmp_ssid);
          return;
          break;
        }
        //else {
        //  strcpy(return_string,"?");
        //}
        break;
    case 'p': // passwd
        if (wifi_command_buffer_index > 5) {
          // empty tmp_passwd
          tmp_passwd[0] = '\0';
          memcpy(tempString, wifi_command_buffer, wifi_command_buffer_index);
          tempString[wifi_command_buffer_index] = '\0';
          Serial.print("wifi_command_buffer passwd tempString: ");
          Serial.println(tempString);
          strncpy(tmp_passwd, tempString+3, wifi_command_buffer_index-3);
          //strcat(return_string,tempString);
          Serial.println(tmp_passwd);
          return;
          break;
        }
        //else {
        //  strcpy(return_string,"?");
        //}
        break;

    //default: strcpy(return_string,"?"); break;
  } // *switch*

} // end of set_WiFi_cred()


// Function set_Time_command. Interprets commands "ts:YYYYMMDD-hhmmss"
void set_Time_command(byte * time_command_buffer, int time_command_buffer_index) {
  char tempString[20] = "";

  if (time_command_buffer[1] == 's' && time_command_buffer[2] == ':' && time_command_buffer[11] == '-') { // look at the second, third and 12th character of the command
        if (time_command_buffer_index == 18) {
          // empty tmp_ssid
          tmp_time[0] = '\0';
          memcpy(tempString, time_command_buffer, time_command_buffer_index);
          tempString[time_command_buffer_index] = '\0';
          Serial.print("time_command_buffer tempString: ");
          Serial.println(tempString);
          strncpy(tmp_time, tempString+3, time_command_buffer_index-3);
          //strcat(return_string,tempString);
          Serial.println(tmp_time);
          // return;
        }
        else {
          Serial.println("ts: wrong format, length invalid.");
          // strcpy(return_string,"?");
        }

  }
  else {
    Serial.println("ts: wrong format.");
  }

} // end of set_Time_command()


//void process_rot2prog_command(byte * rot2prog_command_buffer, int rot2prog_command_buffer_index, byte source_port, char * return_string) {
// void process_rot2prog_command(byte * rot2prog_command_buffer, int rot2prog_command_buffer_index, byte source_port, byte * return_bytes) {
  /* ROT2PROG protocol for SPID RAS
   * Command packets are 13 byte long.
   * Byte: 0 1 2 3 4 5 6 7 8 9 10 11 12
   * Field: S H1 H2 H3 H4 PH V1 V2 V3 V4 PV K END
   * Value: 0x57 0x3? 0x3? 0x3? 0x3? 0x0? 0x3? 0x3? 0x3? 0x3? 0x0? 0x?F 0x20 
   * S
   *    Start byte. This is always 0x57 ('W')
   * H1-H4
   *    Azimuth as ASCII characters 0-9
   * PH
   *    Azimuth resolution in pulses per degree (ignored!)
   * V1-V4
   *    Elevation as ASCII characters 0-9
   * PV
   *    Elevation resolution in pulses per degree (ignored!)
   * K
   *    Command (0x0F=stop, 0x1F=status, 0x2F=set)
   * END
   *    End byte. This is always 0x20 (space)
   * 
   */
/*
  char tempstring[11] = "";
  float heading = -1;
  strcpy(return_string,"");

//  char tempstring[13] = "";
//  float heading = -1;
//  float fAngle = -1;
//  int intPart;
//  int firstDigit = 0;
//  int secondDigit = 0;
//  int thirdDigit = 0;
//  float decpart = 0.0;
//  int decimalPart;
//  // strcpy(return_string,"");
//  return_bytes = {0x00, 0x00, 0x00, 0x00};

  if((rot2prog_command_buffer[0] == 0x57) && (rot2prog_command_buffer_index == 12)) {
    if(rot2prog_command_buffer[11] == 0x0F) {   // stopSPID
      request_command("AZ", "STOP", 0);
      #ifdef ENABLE_ELEVATION
        request_command("EL", "STOP", 0);
      #endif

*/   
/*
    // Send the response 
    // az = H1 * 100 + H2 * 10 + H3 - 360
    // Rot2Prog response packets are 12 bytes long.
    // Byte: 0 1 2 3 4 5 6 7 8 9 10 11
    // Field: S H1 H2 H3 H4 PH V1 V2 V3 V4 PV END
    // Value: 0x57 0x0? 0x0? 0x0? 0x0? 0x0? 0x0? 0x0? 0x0? 0x0? 0x0? 0x20
      fAngle = az_floatAngle + 360;   // az = 12.5 + 360 = 372.5
      intPart = (int)fAngle;  // Should be 372 (3-digits)
      firstDigit = intPart / 100;
      secondDigit = (intPart / 10) % 10;
      thirdDigit = intPart % 10;
      decpart = fAngle - intPart; 
      decimalPart = (int)(decpart * 10);  // Get the first digit of the decimal part, this is the only we are interested in. Should be 5
      return_bytes[0] = 0x57;   // S
      return_bytes[1] = (byte)firstDigit;  // H1 
      return_bytes[2] = (byte)secondDigit;  // H2
      return_bytes[3] = (byte)thirdDigit;   // H3
      return_bytes[4] = (byte)decimalPart;  // H4, the decimal digit
      switch (trackAccuInd) {
        case 0: // 0.1 degree accuracy
          return_bytes[5] = 0x04;   // PH
        case 1: // 1 degree accuracy
          return_bytes[5] = 0x01;   // PH
        default:
          return_bytes[5] = 0x01;
      }
      #ifdef ENABLE_ELEVATION
        fAngle = el_floatAngle + 360;   // az = 12.5 + 360 = 372.5
        intPart = (int)fAngle;  // Should be 372 (3-digits)
        firstDigit = intPart / 100;
        secondDigit = (intPart / 10) % 10;
        thirdDigit = intPart % 10;
        float decpart = fAngle - intPart; 
        decimalPart = (int)(decpart * 10);  // Get the first digit of the decimal part, this is the only we are interested in. Should be 5
        return_bytes[6] = (byte)firstDigit;  // V1 
        return_bytes[7] = (byte)secondDigit;  // V2
        return_bytes[8] = (byte)thirdDigit;   // V3
        return_bytes[9] = (byte)decimalPart;  // V4, the decimal digit
      #else
        return_bytes[6] = 0x00;  // V1 
        return_bytes[7] = 0x00;  // V2
        return_bytes[8] = 0x00;   // V3
        return_bytes[9] = 0x00;  // V4, the decimal digit
      #endif
      return_bytes[10] = return_bytes[5];   // PV same as PH
      return_bytes[11] = 0x20;  // END

      
      
      //dtostrf(az_floatAngle,4,1,tempstring);
      //strcat(return_string,tempstring);
      stopSPID();
*/
/*
      break;
    }
    else if(rot2prog_command_buffer[11] == 0x1F) {   // readSPID, status command
      strcpy(return_string,"AZ");
      strcat(return_string,"+");
      //strcpy(return_string,"+");
      dtostrf(az_floatAngle,0,1,tempstring);
      strcat(return_string,tempstring);
      return;
      break;
      // readSPID();
    }
    else if(rot2prog_command_buffer[11] == 0x2F) {   // setSPID
      
      setSPID();
    }
    else {
      Serial.println("process_rot2prog_command() ERROR, unexpected 12th byte!");
    }
  }

  
}
*/

void process_easycom_command(byte * easycom_command_buffer, int easycom_command_buffer_index, byte source_port, char * return_string){
  /* Easycom protocol implementation
   *
   * Implemented commands:
   *
   * Command      Meaning     Parameters
   * -------      -------     ----------
   *
   * ML           Move Left
   * MR           Move Right
   * MU           Move Up
   * MD           Move Down
   * SA           Stop azimuth moving
   * SE           Stop elevation moving
   *
   * VE           Request Version
   * AZ           Query azimuth
   * AZx.x        Rotate to Azimuth
   * AZxx.x       Rotate to Azimuth
   * AZxxx.x      Rotate to Azimuth
   * EL           Request Elevation
   * ELx.x        Rotate to Elevation
   * ELxx.x       Rotate to Elevation
   * ELxxx.x      Rotate to Elevation
   *
   *
   * Commands are executed upon space, carriage return, or line feed
   * 
   * Reference: https://www.qsl.net/dh1ngp/onlinehelpft100/Rotator_control_with_Easycomm.htm
   * 
   */

  char tempstring[11] = "";
  float heading = -1;
  strcpy(return_string,"");

  switch (easycom_command_buffer[0]) { // look at the first character of the command
/*    #if defined(OPTION_HAMLIB_EASYCOM_AZ_EL_COMMAND_HACK) && defined(FEATURE_ELEVATION_CONTROL)  
    case 'Z':
      //strcpy(return_string,"+");
      strcpy(return_string,"AZ");
      dtostrf(az_floatAngle,0,1,tempstring);
      strcat(return_string,tempstring);
      //if (elevation >= 0){
        //strcat(return_string,"+");
        strcat(return_string," EL");
      //}
      dtostrf(el_floatAngle,0,1,tempstring);      
      strcat(return_string,tempstring);
      break;
    #endif //OPTION_HAMLIB_EASYCOM_AZ_EL_COMMAND_HACK
*/
    case 'A':  // AZ
      if (easycom_command_buffer[1] == 'Z') {  // format is AZx.x or AZxx.x or AZxxx.x (why didn't they make it fixed length?)
        switch (easycom_command_buffer_index) {
          case 2:
            strcpy(return_string,"AZ");
            strcat(return_string,"+");
            //strcpy(return_string,"+");
            dtostrf(az_floatAngle,0,1,tempstring);
            strcat(return_string,tempstring);
            return;
            break;
          case 5: // format AZx.x
            heading = float(easycom_command_buffer[2] - 48) + (float(easycom_command_buffer[4] - 48) / 10.);
            break;
          case 6: // format AZxx.x
            heading = (float(easycom_command_buffer[2] - 48) * 10.) + float(easycom_command_buffer[3] - 48) + (float(easycom_command_buffer[5] - 48) / 10.);
            break;
          case 7: // format AZxxx.x
            heading = (float(easycom_command_buffer[2] - 48) * 100.) + (float(easycom_command_buffer[3] - 48) * 10.) + float(easycom_command_buffer[4] - 48.) + (float(easycom_command_buffer[6] - 48) / 10.);
            break;
            // default: control_port->println("?"); break;
        }
        // if (((heading >= 0) && (heading < 451))  && (easycom_command_buffer[easycom_command_buffer_index - 2] == '.')) {
        if (((heading >= 0) && (heading < 361))  && (easycom_command_buffer[easycom_command_buffer_index - 2] == '.')) {
          #ifdef DEBUG
            Serial.print("GOTO AZ: ");
            Serial.println(heading);
          #endif
          goto_az_value = heading;    
          request_command("AZ", "GOTO", goto_az_value);
        } else {
          strcpy(return_string,"?");
        }
      } else {
        strcpy(return_string,"?");
      }
      break;
      #ifdef ENABLE_ELEVATION
    case 'E':  // EL
      if (easycom_command_buffer[1] == 'L') {
        switch (easycom_command_buffer_index) {
          case 2:
            strcpy(return_string,"EL");
            if (el_floatAngle >= 0){
              strcat(return_string,"+");
              //strcpy(return_string,"+");
            }
            dtostrf(el_floatAngle,0,1,tempstring);
            strcat(return_string,tempstring);            
            return;
            break;
          case 5: // format ELx.x
            heading = float(easycom_command_buffer[2] - 48) + (float(easycom_command_buffer[4] - 48) / 10.);
            break;
          case 6: // format ELxx.x
            heading = (float(easycom_command_buffer[2] - 48) * 10.) + float(easycom_command_buffer[3] - 48) + (float(easycom_command_buffer[5] - 48) / 10.);
            break;
          case 7: // format ELxxx.x
            heading = (float(easycom_command_buffer[2] - 48) * 100.) + (float(easycom_command_buffer[3] - 48) * 10.) + float(easycom_command_buffer[4] - 48) + (float(easycom_command_buffer[6] - 48) / 10.);
            break;
            // default: control_port->println("?"); break;
        }
        if (((heading >= 0) && (heading < 181)) && (easycom_command_buffer[easycom_command_buffer_index - 2] == '.')) {
          #ifdef DEBUG
            Serial.print("GOTO EL: ");
            Serial.println(heading);
          #endif
          goto_el_value = heading;
          request_command("EL", "GOTO", goto_el_value);
        } else {
          strcpy(return_string,"?");
        }
      } else {
        strcpy(return_string,"?");
      }
      break;
      #endif // #ENABLE_ELEVATION
    case 'S':  // SA or SE - stop azimuth, stop elevation
      switch (easycom_command_buffer[1]) {
        case 'A':
          request_command("AZ", "STOP", 0);
          break;
        #ifdef ENABLE_ELEVATION
        case 'E':
          request_command("EL", "STOP", 0);
          break;
        #endif // ENABLE_ELEVATION
        default: strcpy(return_string,"?"); break;
      }
      break;
/*
    case 'M':  // ML, MR, MU, MD - move left, right, up, down
      switch (easycom_command_buffer[1]) {
        case 'L': // ML - move left
          submit_request(AZ, REQUEST_CCW, 0, 40);
          break;
        case 'R': // MR - move right
          submit_request(AZ, REQUEST_CW, 0, 41);
          break;
        #ifdef FEATURE_ELEVATION_CONTROL
        case 'U': // MU - move up
          submit_request(EL, REQUEST_UP, 0, 42);
          break;
        case 'D': // MD - move down
          submit_request(EL, REQUEST_DOWN, 0, 43);
          break;
        #endif // FEATURE_ELEVATION_CONTROL
        default: strcpy(return_string,"?"); break;
      }
      break;
    case 'V': // VE - version query
      if (easycom_command_buffer[1] == 'E') {
        strcpy(return_string,"VE002");
      }                                                                       // not sure what to send back, sending 002 because this is easycom version 2?
      break;
*/
    default: strcpy(return_string,"?"); break;

  } /* switch */

} /* easycom_serial_commmand */



// request_command from serial port, STOP motors or GOTO command
void request_command(String axis, String request, float parm_degrees) {
  tracking_state = 0; // Disable tracking
  if (request == "STOP") {
    if (axis == "AZ") {
      if(moving_state[0] != 0) { // If azimuth Motor is already moving
        Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the azimuth motor
        goto_state[0] = 0; // Stop azimuth moving, STOP.
        #ifdef DEBUG
          Serial.println("STOP az commanded! ");
        #endif
      }
      else {
        #ifdef DEBUG
          Serial.println("STOP, azimuth motor already stopped! ");
        #endif
        ;
      }
    }
    else if (axis == "EL") {
      if(moving_state[1] != 0) { // If elevation Motor is already moving
        Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop the elevation motor
        goto_state[1] = 0; // Stop elevation moving, STOP.
        #ifdef DEBUG
          Serial.println("STOP el commanded! ");
        #endif
      }
      else {
        #ifdef DEBUG
          Serial.println("STOP, elevation motor already stopped! ");
        #endif
      }
    }
    else {
      Serial.println("Error STOP! Invalid axis parameter. ");
    }
  }

  if (request == "GOTO") { 

    if (axis == "AZ") {
      int l_azIntAngle = round(az_floatAngle * trackAccuFactor[trackAccuInd]);    // Convert float to int
      int az_parm = round(parm_degrees * trackAccuFactor[trackAccuInd]);          // Convert float to int
      // Serial.print("GOTO az l_azIntAngle, az_parm :");
      // Serial.print(l_azIntAngle);
      // Serial.println(az_parm);
      if(az_parm < (ANT_AZ_MAX * trackAccuFactor[trackAccuInd]) && az_parm > (ANT_AZ_MIN * trackAccuFactor[trackAccuInd])) {
        if(az_parm < l_azIntAngle) {   // Move CCW (backward) 
          if(moving_state[0] != 2) {    // if not already moving azimuth the same direction
            if(moving_state[0] == 1) {    // if already moving azimuth the opposite (CW) direction
              Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the azimuth motor first
            }
            Motor_backward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);    // third param 0 for azimuth motor
            goto_state[0] = 1; // Moving azimuth to goto position
          }
          #ifdef DEBUG
            Serial.println("Goto azimuth backward... ");
          #endif
        }
        else if((az_parm > l_azIntAngle)) {   // Move CW (forward) 
          if(moving_state[0] != 1) {        // if not already moving azimuth the same direction
            if(moving_state[0] == 2) {    // if already moving azimuth the opposite (CCW) direction
              Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the azimuth motor first
            }
            Motor_forward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);    // third param 0 for azimuth motor
            goto_state[0] = 1; // Moving azimuth to goto position
          }
          #ifdef DEBUG
            Serial.println("Goto azimuth forward... ");
          #endif
        }
        else if((az_parm == l_azIntAngle)) {  // Reached Goto azimuth position
          if(moving_state[0] != 0) { // If azimuth Motor is already moving
            Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the azimuth motor
            goto_state[0] = 0; // Stop azimuth moving, reached goto position.
            #ifdef DEBUG
              Serial.println("Reached azimuth goto position! ");
            #endif
          }
          else {
            #ifdef DEBUG
              Serial.println("GOTO, azimuth motor already stopped! ");
            #endif
            ;
          }
        }
        else {
          Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);
          goto_state[0] = 0;
          Serial.println("Error GOTO! Should never reach here. Stopping Motor. ");
        } 
      } // az_parm out of antenna limits
      else {  //   
        if(moving_state[0] != 0) { // If azimuth Motor is already moving
          Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the motor
        }
        goto_state[0] = 0;
        #ifdef DEBUG
          Serial.println("GOTO Az request out of azimuth antenna limits, motor stopped! ");
        #endif
      }
    }

    else if (axis == "EL") {
      int l_elIntAngle = round(el_floatAngle * trackAccuFactor[trackAccuInd]);    // Convert float to int
      int el_parm = round(parm_degrees * trackAccuFactor[trackAccuInd]);          // Convert float to int
      if(el_parm < (ANT_EL_MAX * trackAccuFactor[trackAccuInd]) && el_parm > (ANT_EL_MIN * trackAccuFactor[trackAccuInd])) {
        if(el_parm < l_elIntAngle) {   // Move Down
          if(moving_state[1] != 2) {    // if not already moving elevation the same direction
            if(moving_state[1] == 1) {    // if already moving elevation the opposite (Up) direction
              Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop the elevation motor first
            }
            Motor_backward(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);    // third param 1 for elevation motor
            goto_state[1] = 1; // Moving elevation to goto position
          }
          #ifdef DEBUG
            Serial.println("GOTO elevation Down... ");
          #endif
        }
        else if((el_parm > l_elIntAngle)) {   // Move Up 
          if(moving_state[1] != 1) {        // if not already moving elevation the same direction
            if(moving_state[1] == 2) {    // if already moving elevation the opposite (Down) direction
              Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop the elevation motor first
            }
            Motor_forward(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);    // third param 1 for elevation motor
            goto_state[1] = 1; // Moving elevation to GOTO position
          }
          #ifdef DEBUG
            Serial.println("GOTO elevation Up... ");
          #endif
        }
        else if((el_parm == l_elIntAngle)) {  // Reached GOTO elevation position
          if(moving_state[1] != 0) { // If elevation Motor is already moving
            Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop the elevation motor
            goto_state[1] = 0; // Stop elevation moving, GOTO position reached.
            #ifdef DEBUG
              Serial.println("Reached GOTO elevation position! ");
            #endif
          }
          else {
            #ifdef DEBUG
              Serial.println("GOTO, elevation motor already stopped! ");
            #endif
          }
        }
        else {
          Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);
          goto_state[1] = 0;
          Serial.println("Error GOTO! Should never reach here. Stopping Motor. ");
        }  
      }
      else {
        if(moving_state[1] != 0) { // If elevation Motor is already moving
          Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);
        }
        goto_state[1] = 0;
        #ifdef DEBUG
          Serial.println("GOTO El request out of azimuth antenna limits, motor stopped! ");
        #endif
      } 
    }
    else {
      Serial.println("Error GOTO! Invalid axis parameter. ");
    }
  }
}   // End of request_command()



// *** Get the antenna motors track the object
void func_track_object(float trackobj_azim, float trackobj_elev) {
  int l_diffTrackTime = 0;
  #ifdef DEBUG
    Serial.println("Entered func_track_object... ");
  #endif

  currentTrackTime = millis();
  l_diffTrackTime = currentTrackTime - startTrackTime;
  #ifdef DEBUG
    Serial.print("l_diffTrackTime is: ");
    Serial.println(l_diffTrackTime);
  #endif

  if (l_diffTrackTime >= int_trackFreq*1000) {   // Period passed since last track, do it again.
    statusTrackingObject[0] = 1;  // azimuth period tracking enable
    statusTrackingObject[1] = 1;  // elevation period tracking enable
    #ifdef DEBUG
      Serial.println("Next period reached, align antenna to object... ");
    #endif
  }

  int l_trackobjElev = round(trackobj_elev * trackAccuFactor[trackAccuInd]);    // Convert float to int
  #ifdef DEBUG
    Serial.print("l_trackobjElev is: ");
    Serial.println(l_trackobjElev);
  #endif
  if (l_trackobjElev <= 0) {    // Do not start tracking if object is not visible (elevation <= 0)
     statusTrackingObject[0] = 0;  // Disable azimuth tracking
     statusTrackingObject[1] = 0;  // Disable elevation tracking
     #ifdef DEBUG
        Serial.println("func_track_object return! ");
     #endif
     return;
  }

  #if defined(ENABLE_AZIMUTH) 
    if (statusTrackingObject[0] != 0) {
    // Track the azimuth
      int l_azIntAngle = round(az_floatAngle * trackAccuFactor[trackAccuInd]);    // Convert float to int
      #ifdef DEBUG
        Serial.print("l_azIntAngle is: ");
        Serial.println(l_azIntAngle);
      #endif
      
      int l_trackobjAzim = round(trackobj_azim * trackAccuFactor[trackAccuInd]);    // Convert float to int
      #ifdef DEBUG
        Serial.print("l_trackobjAzim is: ");
        Serial.println(l_trackobjAzim);
      #endif
      // if(l_trackobjAzim < ANT_AZ_MAX && l_trackobjAzim > ANT_AZ_MIN) {
      if(l_trackobjAzim < (ANT_AZ_MAX * trackAccuFactor[trackAccuInd]) && l_trackobjAzim > (ANT_AZ_MIN * trackAccuFactor[trackAccuInd])) {
        if(l_trackobjAzim < l_azIntAngle) {   // Move CCW (backward)
          statusTrackingObject[0] = 1; // Keep the current tracking period as active
          if(moving_state[0] != 2) {    // if not moving already the same direction
            if(moving_state[0] == 1) {  // if already moving opposite (CW) direction
              Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the motor first
            }
            Motor_backward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);    // third param 0 for azimuth motor
          }
        }
        else if((l_trackobjAzim > l_azIntAngle)) {   // Move CW (forward)
          statusTrackingObject[0] = 1; // Keep the current tracking period as active
          if(moving_state[0] != 1) {        // if not moving already the same direction
            if(moving_state[0] == 2) {  // if already moving opposite (CCW) direction
              Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the motor first
            }
            Motor_forward(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);    // third param 0 for azimuth motor
          }
        }
        else if((l_trackobjAzim == l_azIntAngle)) {  // Reached object position
          statusTrackingObject[0] = 0; // Tracking done for this period
          if(moving_state[0] != 0) {    // If Motor is already moving
            Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the motor
            park_state[0] = 0;  // Disable azimuth PARK
            #ifdef DEBUG
              Serial.println("Reached azimuth object position! ");
            #endif
          }
          else {
            park_state[0] = 0;  // Disable azimuth PARK
            #ifdef DEBUG
              Serial.println("Already at azimuth object position, motor already stopped! ");
            #endif
          }
        }
        else {
          Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);
          park_state[0] = 0;  // Disable azimuth PARK
          #ifdef DEBUG
            Serial.println("Error azimuth! Should never reach here. Stopping Motor. ");
          #endif
        }
      }
      else {    // Tracking object out of antenna limits, stop the motors.
        if(moving_state[0] != 0) {    // If Motor is already moving
          Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop the motor
        }
        #ifdef DEBUG
          Serial.println("Tracking object out of azimuth antenna limits, motor stopped! ");
        #endif
        tracking_state = 0;   // Disable tracking
      }
      
      startTrackTime = currentTrackTime;  // Update the start time of the period
    }
  #else
    statusTrackingObject[0] = 0;  // Disable azimuth tracking
  #endif

  #if defined(ENABLE_ELEVATION)
    if (statusTrackingObject[1] != 0) {
      // Track the azimuth
      int l_elIntAngle = round(el_floatAngle * trackAccuFactor[trackAccuInd]);    // Convert float to int
      #ifdef DEBUG
        Serial.print("l_elIntAngle is: ");
        Serial.println(l_elIntAngle);
      #endif
      // int l_trackobjElev = round(trackobj_elev);    // Convert float to int
      #ifdef DEBUG
        Serial.print("l_trackobjElev is: ");
        Serial.println(l_trackobjElev);
      #endif
      if(l_trackobjElev < (ANT_EL_MAX * trackAccuFactor[trackAccuInd]) && l_trackobjElev > (ANT_EL_MIN * trackAccuFactor[trackAccuInd])) {
        if(l_trackobjElev < l_elIntAngle) {   // Move Downwards
          statusTrackingObject[1] = 1; // Keep the current tracking period as active
          if(moving_state[1] != 2) {    // if not moving already the same direction
            if(moving_state[1] == 1) {  // if already moving opposite (Upwards) direction
              Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop the motor first
            }
            Motor_backward(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);    // third param 1 for elevation motor
          }
        }
        else if((l_trackobjElev > l_elIntAngle)) {   // Move Upwards
          statusTrackingObject[1] = 1; // Keep the current tracking period as active
          if(moving_state[1] != 1) {        // if not moving already the same direction
            if(moving_state[1] == 2) {  // if already moving opposite (Downwards) direction
              Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop the motor first
            }
            Motor_forward(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);    // third param 1 for elevation motor
          }
        }
        else if((l_trackobjElev == l_elIntAngle)) {  // Reached object position
          statusTrackingObject[1] = 0; // Tracking done for this period
          if(moving_state[1] != 0) {    // If Motor is already moving
            Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop the motor
            park_state[1] = 0;  // Disable elevation PARK
            #ifdef DEBUG
              Serial.println("Reached elevation object position! ");
            #endif
          }
          else {
            park_state[1] = 0;  // Disable elevation PARK
            #ifdef DEBUG
              Serial.println("Already at elevation object position, motor already stopped! ");
            #endif
          }
        }
        else {
          Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);
          park_state[1] = 0;  // Disable elevation PARK
          #ifdef DEBUG
            Serial.println("Error elevation! Should never reach here. Stopping Motor. ");
          #endif
        }
      }
      else {    // Tracking object out of antenna limits, stop the motors.
        if(moving_state[1] != 0) {    // If Motor is already moving
          Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop the motor
        }
        #ifdef DEBUG
          Serial.println("Tracking object out of elevation antenna limits, motor stopped! ");
        #endif
        tracking_state = 0;   // Disable tracking  
      }
      
      startTrackTime = currentTrackTime;  // Update the start time of the period
    }
  #else
    statusTrackingObject[1] = 0;  // Disable elevation tracking
  #endif
} // End of func_track_object()


// *** Motor Functions
// For H-bridge MD13S the motor_pin1 is the DIR and motor_pin2 is the PWM.
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
  ledcAttachPin(motor_pin2, pwmCh);     // ATTENTION: This needs to be modified for an H-BRIDGE with PWM port
}


// *** Motor_Stop function. 
// m1 is the pin1 of DC motor, m2 is the pin2 of DC motor
// az_el_ind sets the AZ (param 0) or the EL (param 1) moving_state
void Motor_Stop(int m1, int m2, int az_el_ind, int pwmCh) {         // function of Motor stop
  #ifdef DEBUG
    Serial.println("Motor stopped");
  #endif
  digitalWrite(m2, LOW);  // PWM pin LOW to stop. 
//  digitalWrite(m1, LOW);
  ledcWrite(pwmCh, 0);  
  // az_moving_state = 0;  // Az motor stopped
  moving_state[az_el_ind] = 0;
}

// *** Motor_Soft_Stop function. 
// m1 is the pin1 of DC motor, m2 is the pin2 of DC motor
// az_el_ind sets the AZ (param 0) or the EL (param 1) moving_state
void Motor_Soft_Stop(int m1, int m2, int az_el_ind, int pwmCh, int *dutyCycle) {    // function of Motor Soft stop
  #ifdef DEBUG
    Serial.println("Soft stopping... ");
    Serial.print("dutyCycle is: ");
    Serial.println(*dutyCycle);
  #endif
  if(moving_state[az_el_ind] == 1) { // if Motor is moving forward CW / Up
    digitalWrite(m1, LOW);
    digitalWrite(m2, HIGH);
    while (*dutyCycle <= 255 && *dutyCycle > 0)  {
      ledcWrite(pwmCh, *dutyCycle);   
      *dutyCycle = *dutyCycle - 5;
      // delay(100);
      delay(20);
    }
  }
  else if(moving_state[az_el_ind] == 2) { // if Motor is moving backward CCW / Down
    digitalWrite(m1, HIGH);
    digitalWrite(m2, HIGH);
    while (*dutyCycle <= 255 && *dutyCycle > 0)  {
      ledcWrite(pwmCh, *dutyCycle);   
      *dutyCycle = *dutyCycle - 5;
      // delay(100);
      delay(20);
    }
  }
  
//  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);  // Set PWM pin to LOW to assure motor is stopped. 
 
  #ifdef DEBUG
    Serial.print("Final dutyCycle is: ");
    Serial.println(*dutyCycle);
  #endif
  // az_moving_state = 0;  // Az motor stopped
  moving_state[az_el_ind] = 0;
}

// *** Motor_fast_forward function. 
// m1 is the pin1 of DC motor, m2 is the pin2 of DC motor
// az_el_ind sets the AZ (param 0) or the EL (param 1) moving_state
void Motor_fast_forward(int m1, int m2, int az_el_ind, int pwmCh, int *dutCycl) {          //function of fast_forward 
  int l_dutyCycle = *dutCycl;
  #ifdef DEBUG
    if (az_el_ind == 0) {
      Serial.println("Moving az Fast Forward");
    }
    else {
      Serial.println("Moving el Fast UP");
    }
  #endif
  // Stop motor first
//  Motor_Soft_Stop(m1, m2, az_el_ind, pwmCh, &l_dutyCycle);
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
  startMovingTime[az_el_ind] = millis();
}

// *** Motor_forward function. 
// m1 is the pin1 of DC motor, m2 is the pin2 of DC motor
// az_el_ind sets the AZ (param 0) or the EL (param 1) moving_state
void Motor_forward(int m1, int m2, int az_el_ind, int pwmCh, int *dutCycl) {          //function of forward in medium speed
  int l_dutyCycle = *dutCycl;
  #ifdef DEBUG
    if (az_el_ind == 0) {
      Serial.println("Moving az Forward");
    }
    else {
      Serial.println("Moving el UP");
    }
  #endif
  // Stop motor first
//  Motor_Soft_Stop(m1, m2, az_el_ind, pwmCh, &l_dutyCycle);
  delay(DIR_DELAY);
  // Move forward
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);

  Serial.printf("Motor_forward, menuDutyCycle [%d] of index [%d]\n", menuDutyCycle[indexDC], indexDC);
  *dutCycl = menuDutyCycle[indexDC];
  ledcWrite(pwmCh, *dutCycl);
  // az_moving_state = 1;  // Az motor moving forward CW
  moving_state[az_el_ind] = 1;  // Motor moving forward CW / Up
  startMovingTime[az_el_ind] = millis();
}

// *** Motor_backward function. 
// m1 is the pin1 of DC motor, m2 is the pin2 of DC motor
// az_el_ind sets the AZ (param 0) or the EL (param 1) moving_state
void Motor_backward(int m1, int m2, int az_el_ind, int pwmCh, int *dutCycl) {         //function of backward
  int l_dutyCycle = *dutCycl;
  #ifdef DEBUG
    Serial.printf("Motor_backward, l_dutyCycle is [%d] and az_el_ind is [%d]\n", l_dutyCycle, az_el_ind);
    if (az_el_ind == 0) {
      Serial.println("Moving az Backwards");
    }
    else {
      Serial.println("Moving el Down with reduced speed");
    }
  #endif
    // Stop motor first
//  Motor_Soft_Stop(m1, m2, az_el_ind, pwmCh, &l_dutyCycle);
  delay(DIR_DELAY);
  // Move backward
  digitalWrite(m1, HIGH);
  digitalWrite(m2, HIGH);
  Serial.printf("Motor_backward, menuDutyCycle[indexDC] is [%d]\n", menuDutyCycle[indexDC]);

  *dutCycl = menuDutyCycle[indexDC];  // for MD13S
  ledcWrite(pwmCh, *dutCycl);
  // az_moving_state = 2;  // Az motor moving backward CCW
  moving_state[az_el_ind] = 2;  // Motor moving backward CCW / Down
  startMovingTime[az_el_ind] = millis();
}


// *** Motor_fast_backward function. 
// m1 is the pin1 of DC motor, m2 is the pin2 of DC motor
// az_el_ind sets the AZ (param 0) or the EL (param 1) moving_state
void Motor_fast_backward(int m1, int m2, int az_el_ind, int pwmCh, int *dutCycl) {         //function of backward
  int l_dutyCycle = *dutCycl;
  #ifdef DEBUG
    if (az_el_ind == 0) {
      Serial.println("Moving az Backwards with max speed");
    }
    else {
      Serial.println("Moving el Down with max speed");
    }
  #endif
    // Stop motor first
//  Motor_Soft_Stop(m1, m2, az_el_ind, pwmCh, &l_dutyCycle);
  delay(DIR_DELAY);
  // Move backward
  digitalWrite(m1, HIGH);
  digitalWrite(m2, HIGH);
  *dutCycl = 255;
  ledcWrite(pwmCh, *dutCycl);
  // az_moving_state = 2;  // Az motor moving backward CCW
  moving_state[az_el_ind] = 2;  // Motor moving backward CCW / Down
  startMovingTime[az_el_ind] = millis();
}

/// int Park_Position(int m1, int m2, 

// *** Functions ***
/* // This function is not needed.
int nextSatPass(long _nextpassEpoch[numSats]){ // Replace with number of satellites
  for(i = 0;i < numSats; ++i){
    if( _nextpassEpoch[0]-timeNow >= _nextpassEpoch[i]-timeNow){
      _nextpassEpoch[0] = _nextpassEpoch[i];
      nextSat = i;
    }
  }
  return nextSat;
}
*/ 

// *** Show in how many seconds the satellite will rise 
int satelRise(long _nextpassEpoch[numSats]) { // Replace with number of satellites
  for(i = 0;i < numSats; ++i){
    if( _nextpassEpoch[i]-timeNow >= 0) {
      nextRise[i] = int(_nextpassEpoch[i] - timeNow);  // The satellite [i] will be visible in x seconds
      // return 1;
    }
    else {
      #ifdef DEBUG
        Serial.print("nextRise of satellite: ");
        Serial.print(i);
        Serial.print(" is negative: ");
        Serial.println(nextRise[i]);
      #endif
      return 0;
    }
  }
  return 1;
}


int getTimeNTP() {
  // delay(100);
  time_t epochTimeNow;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return 0;
  }
  #ifdef DEBUG
    Serial.println(&timeinfo, "%d %m %Y %H:%M:%S");
  #endif
  
  time(&epochTimeNow);
  epochTime = epochTimeNow;
  #ifdef DEBUG
    Serial.print("epochTime: ");
    Serial.println(epochTime);
  #endif
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
  sunpos(udtTime, udtLocation, &coord);
  #ifdef DEBUG
    Serial.println(" ");
    Serial.print("Latitude: ");
    Serial.print(udtLocation.dLatitude);
    Serial.print(" Longitude: ");
    Serial.print(udtLocation.dLongitude);
    Serial.println();
  
    Serial.print("Sun Zenith angle: ");
    Serial.print(coord.dZenithAngle);
    Serial.print(" Sun Elevetion angle: ");
  #endif
  sun_elev = 90-coord.dZenithAngle; // Elev is 90 degress minus the Zenith angle
  #ifdef DEBUG
    Serial.print(sun_elev);  
    Serial.println();
    Serial.print(" Sun Azimuth: ");
  #endif
  sun_azim = coord.dAzimuth;
  #ifdef DEBUG
    Serial.print(sun_azim);
    Serial.println();
    Serial.println();
    Serial.println();
  #endif
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
  
  moon2(udtTime.iYear, udtTime.iMonth, udtTime.iDay, (udtTime.dHours + (udtTime.dMinutes / 60.0) + (udtTime.dSeconds / 3600.0)), udtLocation.dLongitude, udtLocation.dLatitude, &RA, &Dec, &topRA, &topDec, &LST, &HA, &moon_azimuth, &moon_elevation, &dist);
  #ifdef DEBUG
  Serial.println(" ");
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
  #endif
}



// ### update_azOffset(*fazOffset), stores the Az offset in EEPROM. 
void update_azOffset(char *fazOffset) {
  strncpy(azOffset, fazOffset, 7); // Copy the fazOffset back to azOffset
  azimuth_offset = (float)atof(fazOffset);
  #ifdef DEBUG
    Serial.printf("update_azOffset(), the azimuth_offset floating is [%f]\n", azimuth_offset);
  #endif
  azOffset_mem = fazOffset;    // assign char array to string ???
  #ifdef DEBUG
    Serial.printf(" azOffset_mem [%s], saving... \n", azOffset_mem);
  #endif
  preferences.putString("az_offset", azOffset_mem);   // Store the azOffset to EEPROM
  if(strlen(fazOffset) == 6) {  // az offset is 6 characters long, e.g. -003.1
  #ifdef DEBUG
    Serial.printf(" azOffset: %s ,", fazOffset);
    Serial.printf(" length of fazOffset: %d\n", strlen(fazOffset));
  #endif
  }
  else {
    Serial.printf("Need to restart to microcontroller, doing now ... \n");
    Serial.printf(" Az offset is invalid: [%s] \n", fazOffset);
    delay(3000);
    ESP.restart();
  }
} // End of update_azOffset()


// ### update_elOffset(*felOffset), stores the El offset in EEPROM. 
void update_elOffset(char *felOffset) {
  strncpy(elOffset, felOffset, 7); // Copy the felOffset back to elOffset
  elevation_offset = (float)atof(felOffset);
  #ifdef DEBUG
    Serial.printf("update_elOffset(), the elevation_offset floating is [%f]\n", elevation_offset);
  #endif
  elOffset_mem = felOffset;    // assign char array to string ???
  #ifdef DEBUG
    Serial.printf(" elOffset_mem [%s], saving... \n", elOffset_mem);
  #endif
  preferences.putString("el_offset", elOffset_mem);   // Store the elOffset to EEPROM
  if(strlen(felOffset) == 6) {  // el offset is 6 characters long, e.g. -003.1
  #ifdef DEBUG
    Serial.printf(" elOffset: %s ,", felOffset);
    Serial.printf(" length of felOffset: %d\n", strlen(felOffset));
  #endif
  }
  else {
    Serial.printf("Need to restart to microcontroller, doing now ... \n");
    Serial.printf(" El offset is invalid: [%s] \n", felOffset);
    delay(3000);
    ESP.restart();
  }
} // End of update_elOffset()


// ### update_trackFreq(*ftrackFreq), stores the tracking frequency in EEPROM. 
void update_trackFreq(char *ftrackFreq) {
  strncpy(trackFreq, ftrackFreq, 4); // Copy the ftrackFreq back to trackFreq
  // int_trackFreq = atoi(ftrackFreq);
  sscanf(ftrackFreq, "%d", &int_trackFreq);
  #ifdef DEBUG
    Serial.printf("update_trackFreq(), the tracking freq is [%d]\n", int_trackFreq);
  #endif
  trackFreq_mem = ftrackFreq;    // assign char array to string ???
  #ifdef DEBUG
    Serial.printf(" trackFreq_mem [%s], saving... \n", trackFreq_mem);
  #endif
  preferences.putString("track_freq", trackFreq_mem);   // Store the trackFreq to EEPROM
  if(strlen(ftrackFreq) == 3) {  // trackFreq is 3 characters long, e.g. 030
  #ifdef DEBUG
    Serial.printf(" trackFreq: %s ,", ftrackFreq);
    Serial.printf(" length of ftrackFreq: %d ,", strlen(ftrackFreq));
  #endif
  }
  else {
    Serial.printf("Need to restart to microcontroller, doing now ... \n");
    Serial.printf(" track freq is invalid: [%s] \n", ftrackFreq);
    delay(3000);
    ESP.restart();
  }
} // End of update_trackFreq()


// ### update_speed(findexDC), stores the speed in EEPROM. 
void update_speed(char findexDC) {
  indexDC = findexDC - '0';
  dutyCycleAz = menuDutyCycle[indexDC];
  dutyCycleEl = menuDutyCycle[indexDC];
  indexDC_mem = findexDC;
  #ifdef DEBUG
    Serial.printf(" indexDC_mem [%c], saving... \n", indexDC_mem);
  #endif
  preferences.putChar("index_DutyCycle", indexDC_mem);   // Store the indexDC to EEPROM
} // End of update_speed()


// ### update_trackAccuracy(ftrackAccu), stores the tracking accuracy in EEPROM. 
void update_trackAccuracy(char ftrackAccu) {
  trackAccuInd = ftrackAccu - '0';
  
  trackAccuInd_mem = ftrackAccu;
  #ifdef DEBUG
    Serial.printf(" trackAccuInd_mem [%c], saving... \n", trackAccuInd_mem);
  #endif
  preferences.putChar("track_accuracy", trackAccuInd_mem);   // Store the trackAccuInd_mem to EEPROM
} // End of update_trackAccuracy()


// ### update_qth(*fgrid), stores the grid in EEPROM and calculates the long and lat. 
void update_qth(char *fgrid) {
  strncpy(grid, fgrid, 7); // Copy the fgrid back to grid
  grid_mem = fgrid;    // assign char array to string ???
  #ifdef DEBUG
    Serial.printf(" grid_mem [%s], saving... \n", grid_mem);
  #endif
  preferences.putString("grid", grid_mem);   // Store the grid to EEPROM
  if(strlen(fgrid) == 6) {  // grid is 6 characters long
    grid2deg(fgrid, &longitude, &latitude);
    #ifdef DEBUG
      Serial.printf(" grid: %s ,", fgrid);
      Serial.printf(" length of grid: %d ,", strlen(fgrid));
      Serial.printf(" longitude: %f ,", longitude);
      Serial.printf(" latitude: %f \n", latitude);
    #endif
    udtLocation.dLatitude = latitude;
    udtLocation.dLongitude = longitude;
    sat.site(latitude,longitude,myAlt); //set location latitude[°], longitude[°] and altitude[m]
  }
  else {
    Serial.printf("Need to restart to microcontroller, doing now ... \n");
    Serial.printf(" grid square is invalid: [%s] \n", fgrid);
    delay(3000);
    ESP.restart();
  }
} // End of update_qth()



// ### update_satID(*fsatID), stores the satID in EEPROM. 
void update_satID(char *fsatID, int satInd) {
  strncpy(satID[satInd], fsatID, 6); // Copy the fsatID back to satID[satInd]
  switch (satInd) {
    case 0:
      satID1_mem = fsatID;
      #ifdef DEBUG
        Serial.printf(" satID1_mem [%s], saving... \n", satID1_mem);
      #endif
      preferences.putString("sat_id1", satID1_mem);   // Store the satID1_mem to EEPROM
      break;
    case 1:
      satID2_mem = fsatID;
      #ifdef DEBUG
        Serial.printf(" satID2_mem [%s], saving... \n", satID2_mem);
      #endif
      preferences.putString("sat_id2", satID2_mem);   // Store the satID2_mem to EEPROM
      break;
    case 2:
      satID3_mem = fsatID;
      #ifdef DEBUG
        Serial.printf(" satID3_mem [%s], saving... \n", satID3_mem);
      #endif
      preferences.putString("sat_id3", satID3_mem);   // Store the satID3_mem to EEPROM
      break;
    case 3:
      satID4_mem = fsatID;
      #ifdef DEBUG
        Serial.printf(" satID4_mem [%s], saving... \n", satID4_mem);
      #endif
      preferences.putString("sat_id4", satID4_mem);   // Store the satID4_mem to EEPROM
      break;
  }

} // End of update_satID()


// Function retrieve_satName, retrieves the short satellite name extracted from the 1st line of the TLE file
void retrieve_satName(char *fsatName, int satIndex) {
  // char satelName
  int parB = 0;
  int parE = 0;
  String str1 = "";
  String str2 = "";
  for(int i=0; fsatName[i] != '\0'; ++i) {
    if(fsatName[i] == '(') {
      parB = i;
      Serial.printf("parB is [%d]\n", parB);
    }
    if(fsatName[i] == ')') {
      parE = i;
      Serial.printf("parE is [%d]\n", parE);
      break;
    }
  }

  if (parE == 0) {
    Serial.println("No parenthesis!");
    str1 = fsatName;  // convert char array to string
    Serial.printf("str1 [%s]\n", fsatName);
    str2 = str1.substring(0, 6);    // Get the first 6 characters of the name
    Serial.printf("str2 wo parenth [%s]\n", str2);
  }
  else {
    str1 = fsatName;  // convert char array to string
    Serial.printf("whole str1 [%s]\n", fsatName);
    str2 = str1.substring(parB +1, parE);   // Get the name from inside the parenthesis
    Serial.printf("str2 [%s]\n", str2);
  }
  str2.toCharArray(satelName[satIndex], sizeof(satelName[satIndex]));
} // End of retrieve_satName


// ### update_wifiCred(*fssid, *passwd), stores the WiFi credentials ssid and passwd in EEPROM. 
void update_wifiCred(char *fssid, char *fpasswd) {
//  strncpy(wifi_ssid, fssid, sizeof(fssid)); // Copy the fssid back to wifi_ssid
//  strncpy(wifi_passwd, fpasswd, sizeof(fpasswd)); // Copy the fpasswd back to wifi_passwd

  #ifdef DEBUG
    Serial.printf("update_wifiCred(), the ssid is [%s] and passwd is [%s]\n", fssid, fpasswd);
  #endif
  ssid_mem = fssid;    // assign char array to string ???
  passwd_mem = fpasswd;
  #ifdef DEBUG
    Serial.printf(" ssid_mem [%s] and passwd_mem [%s], saving... \n", ssid_mem.c_str(), passwd_mem.c_str());
  #endif
  preferences.putString("ssid", ssid_mem);   // Store the ssid to EEPROM
  preferences.putString("password", passwd_mem);   // Store the password to EEPROM

} // End of update_wifiCred()


// ### set_time_manual(*ftime), sets the date-time entered manually by command ts:YYYYMMDD-hhmmss
void set_time_manual(char *ftime) {
  #ifdef DEBUG
    Serial.printf("set_time_manual(), the time is [%s]\n", ftime);
  #endif
  char mYear[5] = "";
  char mMonth[3] = "";
  char mDay[3] = "";
  char mHours[3] = "";
  char mMinutes[3] = "";
  char mSeconds[3] = "";
  mYear[0] = '\0';
  mMonth[0] = '\0';
  mDay[0] = '\0';
  mHours[0] = '\0';
  mMinutes[0] = '\0';
  mSeconds[0] = '\0';
  int i;
  strncpy(mYear, ftime, 4);
  sscanf(mYear, "%d", &i);
  timeInfoMan.tm_year = i - 1900; // tm_year is the year since 190
  strncpy(mMonth, ftime+4, 2);
  sscanf(mMonth, "%d", &i);
  timeInfoMan.tm_mon = i -1;   // tm_mon is from 0 to 11
  strncpy(mDay, ftime+6, 2);
  sscanf(mDay, "%d", &i);
  timeInfoMan.tm_mday = i;
  strncpy(mHours, ftime+9, 2);
  sscanf(mHours, "%d", &i);
  timeInfoMan.tm_hour = i;
  strncpy(mMinutes, ftime+11, 2);
  sscanf(mMinutes, "%d", &i);
  timeInfoMan.tm_min = i;
  strncpy(mSeconds, ftime+13, 2);
  sscanf(mSeconds, "%d", &i);
  timeInfoMan.tm_sec = i;

  time_t epochTimeManNow;
  epochTimeManNow = mktime(&timeInfoMan);
  epochTimeMan = epochTimeManNow;
  Serial.print("epochTimeMan: ");
  Serial.println(epochTimeMan);

  tv.tv_sec = epochTimeMan; // UnixTime
  settimeofday(&tv, NULL);    // Set the system time
  epochTimeGPS = epochTimeMan;
  gPS_flag = true;

} // End of set_time_manual()


void init_lcd() {
  Serial.println("Setup initial display");
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0); // 1st line
  lcd.print(" deltanu - AntCtrl ");
  lcd.setCursor(0,1); // 2nd line
  lcd.print(" Ver: ");
  lcd.print(CODE_VERSION);
  lcd.setCursor(0,2); // 3rd line
  lcd.print(" QTH loc: ");
  lcd.print(grid);

  delay(3000);
  
}


void init_lcd2() {
  Serial.println("Setup initial display 2");
  clear_lcd();
  lcd.setCursor(0,0); // 1st line
  lcd.print(" Az offset: ");
  lcd.print(azOffset);
  lcd.setCursor(0,1); // 2nd line
  lcd.print(" El offset: ");
  lcd.print(elOffset);
  lcd.setCursor(0,2); // 3rd line
  lcd.print(" Tracking Freq: ");
  lcd.print(trackFreq);
  lcd.setCursor(0,3); // 4th line
  lcd.print(" Ant Speed: ");
  lcd.print(indexDC_mem);
  delay(3000);
  
}


// Showing the status diplay while updating the TLEs and time
void tle_time_lcd() {
  Serial.println("Updating TLE and Time");
  clear_lcd();
  lcd.backlight();
  lcd.setCursor(0,1); // 2nd line
  lcd.print(" Updating the TLEs ");
  lcd.setCursor(0,2); // 3rd line
  lcd.print(" and time... ");
  delay(500);
}


// Showing the status diplay after successfully updated the TLEs and Time
void tle_time_lcd_success() {
  Serial.println("Successfully Updated TLE and Time");
  clear_lcd();
  lcd.backlight();
  lcd.setCursor(0,1); // 2nd line
  lcd.print(" TLEs and Time ");
  lcd.setCursor(0,2); // 3rd line
  lcd.print(" updated ! ");
  delay(1000);
}


void error_lcd(short errNo) {
  clear_lcd();
  switch(errNo)
  {
    case 1:
      lcd.setCursor(0,0); // 1st line
      lcd.print(" WiFi conn error!");
      lcd.setCursor(0,2); // 2nd line
      lcd.print("Restart or set ssid");
      delay(2000);
      break;
    case 2:
      lcd.setCursor(0,1); // 2nd line
      lcd.print(" NTP Time error!");
      lcd.setCursor(0,2); // 2nd line
      lcd.print("Restart controller");
      break;
    default:
      lcd.setCursor(0,2); // 3rd line
      lcd.print("Restart controller");
      lcd.setCursor(0,3); // 4th line
      lcd.print(" Unexpected error!");
      break;
  }
  delay(2000);
}

/*
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
*/

/*
// Display Az and El angle of Antenna
void display_angle(float azAngle, float elAngle) {
  //clear display
//  display.clearDisplay();

  // Display Az degrees
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,10);  // Second line of OLED
  display.print("Az:");
  display.print(azAngle, 1);
//  display.print(" ");
//  display.setTextSize(1);
//  display.write(167); // Ascii code of Degree symbol

  // Display El degrees
  display.setTextSize(1);
  display.setCursor(0,20);  // Third line of OLED
  display.print("El:");
  display.print(elAngle, 1);
//  display.print(" ");
//  display.setTextSize(1);
//  display.write(167); // Ascii code of Degree symbol
  
//  display.display();
}
*/

// Display Az and El angle of Antenna
void lcd_angle(float azAngle, float elAngle) {
  int l_azim;
  int l_elev;
  // Display Az degrees
  lcd.setCursor(1,1); // 2nd line and 2nd char position of LCD
  lcd.print("ANT ");
  lcd.print(azAngle, 1);  // One decimal point
  l_azim = round(azAngle);
  if(l_azim >= 100) {
    ;   // Do nothing
  }
  else if(l_azim >= 10) {
    lcd.setCursor(9,1);
    lcd.print(" "); // clear the last digit just in case it has value from previous screen
  }
  else {
    lcd.setCursor(8,1);
    lcd.print("  ");  // clear the last two digits just in case they have value from previous screen
  }
  
//  display.print(" ");
//  display.setTextSize(1);
//  display.write(167); // Ascii code of Degree symbol

  // Display El degrees
  lcd.setCursor(12,1);  // Go to middle of 2nd line
  lcd.print(" ");
  lcd.print(elAngle, 1);  // One decimal point
  l_elev = round(elAngle);
  if(l_elev >= 10) {
    lcd.setCursor(17,1);
    lcd.print(" ");
  }
  else {
    lcd.setCursor(16,1);
    lcd.print("  "); // clear the last digit just in case it has value from previous screen
  }
//  display.print(" ");
//  display.setTextSize(1);
//  display.write(167); // Ascii code of Degree symbol
 
}


/*
void display_AntStatus(int azStatus, int elStatus) {
  int asciiSym = 32;  // Ascii code for space
  if(azStatus == 1) { // azimuth is moving CW - right
    asciiSym = 175;   // ascii code of >>
  }
  else if(azStatus == 2) {  // azimuth is moving CCW - left
    asciiSym = 174;   // ascii code of <<
  }
  else {  // Azimuth not moving
    asciiSym = 32;  // Ascii code for space
  }
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(55,10);
  display.write(asciiSym);

  asciiSym = 32;  // Ascii code for space
  if(elStatus == 1) { // elevation is moving Upwards
    asciiSym = 94;   // ascii code of ^
  }
  else if(elStatus == 2) {  // elevation is moving Downwards
    asciiSym = 118;   // ascii code of v
  }
  else {  // elevation not moving
    asciiSym = 32;  // Ascii code for space
  }
  display.setTextSize(1);
  display.setCursor(55,20);
  display.write(asciiSym);
}
*/


void lcd_AntStatus(int azStatus, int elStatus, int ftracking_state) {
  int asciiSym = 32;  // Ascii code for space

  if(ftracking_state == 1) {
    asciiSym = 42;   // ascii code of *
  }
  else {  // Auto tracking not enabled
    asciiSym = 32;  // Ascii code for space
  } 
  lcd.setCursor(0,1);
  lcd.print(char(asciiSym));
  
  if(azStatus == 1) { // azimuth is moving CW - right
    asciiSym = 62;   // ascii code of >
  }
  else if(azStatus == 2) {  // azimuth is moving CCW - left
    asciiSym = 60;   // ascii code of <
  }
  else {  // Azimuth not moving
    asciiSym = 32;  // Ascii code for space
  }
  
  lcd.setCursor(10,1);
  lcd.print(char(asciiSym));
  lcd.print(char(asciiSym));

  asciiSym = 32;  // Ascii code for space
  if(elStatus == 1) { // elevation is moving Upwards
    asciiSym = 94;   // ascii code of ^
  }
  else if(elStatus == 2) {  // elevation is moving Downwards
    asciiSym = 118;   // ascii code of v
  }
  else {  // elevation not moving
    asciiSym = 32;  // Ascii code for space
  }
  lcd.setCursor(18,1);
  lcd.print(char(asciiSym));
}


/*
// Display the az and el angle of the tracking object
void display_objPos(float azAngle, float elAngle) {
  //clear display
//  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,40);
  display.print(azAngle, 1);
  display.print(" ");
  display.print(elAngle, 1);

//  display.display();
}
*/


// Display the az and el angle of the tracking object
void lcd_objPos(float azAngle, float elAngle) {

  lcd.setCursor(8,2);
  lcd.print(azAngle, 1);
  lcd.print(" ");
  lcd.print(elAngle, 1);
  lcd.setCursor(0,3);   // Goto to 4th line
  int i;
  for(i=0; i<20; i++) {
    // lcd.setCursor(i,3);   // Goto to 4th line
    lcd.print(" ");       // and clear any previous text
  }
  lcd.setCursor(0,3);

//  display.display();
}


// Shows error in LCD when Motors / Absolute Encoders fail.
void lcd_showError(int azEl_ind) {
  lcd.setCursor(0,3);   // Goto to 4th line
  int i;
  for(i=0; i<20; i++) {
    // lcd.setCursor(i,3);   // Goto to 4th line
    lcd.print(" ");       // and clear any previous text
  }
  
  lcd.setCursor(0,3);
  lcd.print("ERROR ");
  if(azEl_ind == 0) {
    lcd.print("Az");
  }
  if(azEl_ind == 1) {
    lcd.setCursor(9,3);  // Go to middle of 2nd line
    lcd.print("El");
  }
  
  lcd.setCursor(13,3);  // Go to middle of 2nd line
  lcd.print("Motors");
  delay(1000);    // Show LCD_DISPLAY for one second.
}

/*
// Display the label of the tracking object
void display_objText(int objIndex) {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,30);

  switch(objIndex)
  {
    case 0:
      display.print("Sun Pos:");
      break;
    case 1:
      display.print("Moon Pos:");
      break;
    case 6:
      display.print("PARK Pos:");
      break;
    default:
      display.print(satnames[SAT]);
      // display.print("No object");
      
  }
} // End of display_objText()
*/


// Display the label of the tracking object
void lcd_objText(int objIndex) {
  lcd.setCursor(0,2); // beginning of 3rd line

  switch(objIndex)
  {
    case 0:
      lcd.print("Sun: ");
      break;
    case 1:
      lcd.print("Moon: ");
      break;
    case 6:
      lcd.print("PARK: ");
      break;
    default:
      // lcd.print(satnames[SAT]);
      lcd.print(satelName[SAT]);
      // display.print("No object");
      
  }
} // End of lcd_objText()

/*
// Display the label of the Menu
void display_MenuText(int menuIndex) {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,30);

  switch(menuIndex)
  {
    case 0:
      display.print("Grid Loc:");
      break;
    case 1:
      display.print("WiFi ssid:");
      break;
    case 2:
      display.print("WiFi pass:");
      break;
    default:
      display.print("Err menu");
      
  }
} // End of display_MenuText()
*/


// Display the label of the Menu
void lcd_MenuText(int menuIndex) {
  lcd.setCursor(0,2);
  int i;
  int asciiSym = 223;  // Ascii code for Degree symbol

  switch(menuIndex)
  {
    case 0:
      lcd.print(">Az Offset:");
      break;
    case 1:
      lcd.print(">El Offset:");
      break;
    case 2:
      lcd.print(">Tracking Freq:");
      lcd.setCursor(4,3);
      lcd.print("seconds");
      break;
    case 3:
      lcd.print(">Antenna speed:");
      lcd.setCursor(2,3);
      lcd.print("(0:Lo 1:Mid 2:Hi)");
      break;
    case 4:
      lcd.print(">Track accuracy:");
      lcd.setCursor(2,3);
      // lcd.print("(0:0.1' 1:1')");
      lcd.print("(0:0.1");
      lcd.print(char(asciiSym));
      lcd.print(" 1:1");
      lcd.print(char(asciiSym));
      lcd.print(")");
      break;
    case 5:
      lcd.print(">Grid loc:");
      break;
    case 6:
      lcd.print(">SAT1: ");
      // for(i=0; i<=sizeof(satelName[0]); i++) {
      for(i=0; satelName[0][i] != '\0'; i++) {
        lcd.print(satelName[0][i]); // The name of SAT1
      }
      lcd.setCursor(6,3);
      lcd.print("NORAD No");
      break;
    case 7:
      lcd.print(">SAT2: ");
      // for(i=0; i<=sizeof(satelName[1]); i++) {
      for(i=0; satelName[1][i] != '\0'; i++) {
        lcd.print(satelName[1][i]); // The name of SAT2
      }
      lcd.setCursor(6,3);
      lcd.print("NORAD No");
      break;
    case 8:
      lcd.print(">SAT3: ");
      // for(i=0; i<=sizeof(satelName[2]); i++) {
      for(i=0; satelName[2][i] != '\0'; i++) {
        lcd.print(satelName[2][i]); // The name of SAT3
      }
      lcd.setCursor(6,3);
      lcd.print("NORAD No");
      break;
    case 9:
      lcd.print(">SAT4: ");
      // for(i=0; i<=sizeof(satelName[3]); i++) {
      for(i=0; satelName[3][i] != '\0'; i++) {
        lcd.print(satelName[3][i]); // The name of SAT4
      }
      lcd.setCursor(6,3);
      lcd.print("NORAD No");
      break;
    case 10:
      lcd.print(">Time. '-' to store");
      break;
    default:
      lcd.print(">Err menu");
      
  }
} // End of lcd_MenuText()


void lcd_time(char timeSrc, boolean showLoc) {
  char wholeTime[20];
  if (showLoc == true) {  // Show grid Locator

    lHours = timeinfo.tm_hour;
    lMinutes = timeinfo.tm_min;
    lSeconds = timeinfo.tm_sec;
    if(gridGPS[0] == 0) {
      strcpy(gridGPS, "      ");    // Print empty
    }
    if(gpsStatus != 1) {  // GPS signal lost, show "!" after the locator.
      snprintf(wholeTime, sizeof(wholeTime), "Loc:%s!%02d:%02d:%02d", gridGPS, lHours, lMinutes, lSeconds);
    }
    else {
      snprintf(wholeTime, sizeof(wholeTime), "Loc:%s %02d:%02d:%02d", gridGPS, lHours, lMinutes, lSeconds);
    }
  }
  else {    // Show date
    lDay = timeinfo.tm_mday;
    lMonth = timeinfo.tm_mon + 1; // tm_mon is from 0 to 11
    lYear = timeinfo.tm_year + 1900;  // tm_year is the year since 1900
    lHours = timeinfo.tm_hour;
    lMinutes = timeinfo.tm_min;
    lSeconds = timeinfo.tm_sec;
    // snprintf(wholeTime, sizeof(wholeTime), "%04d-%02d-%02d %02d:%02d:%02d", lYear, lMonth, lDay, lHours, lMinutes, lSeconds);
    snprintf(wholeTime, sizeof(wholeTime), "%04d-%02d-%02d %02d:%02d:%02d", lYear, lMonth, lDay, lHours, lMinutes, lSeconds);
// timeSrc. "Y" for GPS time, "N" for NTP time, "!" for temporary internal RTC time.
  }
  lcd.setCursor(0,0); // 1st line of LCD
  lcd.print(wholeTime);
  lcd.print(timeSrc);
}

/*
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
  display.setTextColor(WHITE);
  display.setCursor(0,0); // 1st line of OLED, only 10 characters MMDDhhmmss
  display.print(wholeTime);
}
*/

/*
void display_clock(int _timeRise) {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,40);
  display.print("In: ");
  display.print(_timeRise, 0);
}
*/

void lcd_clock(int _timeRise, double fAzstart, double fMaxEl) {
  lcd.setCursor(0,3);   // Goto to 4th line
  int i;
  for(i=0; i<20; i++) {
    lcd.print(" ");       // and clear any previous text
  }
  
  float l_azStart;
  float l_maxEl;
  l_azStart = (float)fAzstart;
  l_maxEl = (float)fMaxEl;
  
  lcd.setCursor(8,2);
  lcd.print("In:");
  lcd.print(_timeRise, 0);
  lcd.print(" sec ");
  
  lcd.setCursor(0,3);
  lcd.print("Az:");
  lcd.print(l_azStart, 1);
  lcd.setCursor(10,3);  // Go to middle of 2nd line
  lcd.print("MaxEl:");
  lcd.print(l_maxEl, 1);
}

/*
void display_menuValue(int menuObj, short invert_text) {
  char charDisp[30];
  int charDisp_len;
  
  if (menuObj == 0) { // grid
    Serial.printf("grid is %s\n", grid);
    display.setTextSize(1);
    display.setCursor(0,40);
   
    if(invert_text == 0)  { // White
    display.setTextColor(WHITE);
    }
    else {
      display.setTextColor(BLACK, WHITE); // 'inverted' text
    }
    // display.print(grid);
    display.print(grid[0]);
    display.setTextColor(WHITE);
    int cursorx; int cursory;
    cursorx = display.getCursorX();
    cursory = display.getCursorY();
    Serial.printf("after grid[0] cursor X [%d], Y [%d]\n", cursorx, cursory);
    int i=1;
    for(i=1; i<=5; i++) {
      display.print(grid[i]);
    }
    
  }
  else if (menuObj == 1) { // ssid
    char charDisp1[30];
    int charDisp1_len;
    int ten_len;
    char ten_ssid[11];  // Don't forget the termination null character
    ssid_mem.toCharArray(charDisp1, sizeof(charDisp1));
    // charDisp_len = sizeof(charDisp);
    // Serial.printf("ccsid_len is %d\n", cssid_len);
    ////sprintf(cssid, "%s", ssid_mem);
    strncpy(ten_ssid, charDisp1, 10);
    ten_len = sizeof(ten_ssid);
    Serial.printf("ten_ssid is %d\n", ten_len);
    display.setTextSize(1);
    display.setCursor(0,40);
    display.print(ten_ssid);
    // display.print(ssid_mem);
  }
  else if (menuObj == 2) { // WiFi passwd
    char charDisp2[30];
    int charDisp2_len;
    int ten_len;
    char ten_pass[11];  // Don't forget the termination null character
    passwd_mem.toCharArray(charDisp2, sizeof(charDisp2));

    strncpy(ten_pass, charDisp2, 10);
    ten_len = sizeof(ten_pass);
    Serial.printf("ten_len is %d\n", ten_len);
    display.setTextSize(1);
    display.setCursor(0,40);
    display.print(ten_pass);
    // display.print(ssid_mem);
  }
}
*/


void lcd_menuValue(int menuObj) {
  char charDisp[30];
  int charDisp_len;

  if (menuObj == 0) { // Az offset
    #ifdef DEBUG
      Serial.printf("tmp_azOffset is %s\n", tmp_azOffset);
    #endif
    lcd.setCursor(0,3);
    int i;
    for(i=0; i<=5; i++) {
      lcd.print(tmp_azOffset[i]);
    }
  }
  else if (menuObj == 1) { // El offset
    #ifdef DEBUG
      Serial.printf("tmp_elOffset is %s\n", tmp_elOffset);
    #endif
    lcd.setCursor(0,3);
    int i;
    for(i=0; i<=5; i++) {
      lcd.print(tmp_elOffset[i]);
    }
  }

  else if (menuObj == 2) { // Track freq
    #ifdef DEBUG
      Serial.printf("tmp_trackFreq is %s\n", tmp_trackFreq);
    #endif
    lcd.setCursor(0,3);
    int i;
    for(i=0; i<=2; i++) {
      lcd.print(tmp_trackFreq[i]);
    }
  }
  else if (menuObj == 3) { // Speed (PWM)
    #ifdef DEBUG
      Serial.printf("tmp_indexDC is %c\n", tmp_indexDC);
    #endif
    lcd.setCursor(0,3);
    lcd.print(tmp_indexDC);
  }
  else if (menuObj == 4) { // Tracking accuracy
    #ifdef DEBUG
      Serial.printf("tmp_trackAccuInd is %c\n", tmp_trackAccuInd);
    #endif
    lcd.setCursor(0,3);
    lcd.print(tmp_trackAccuInd);
  }
  else if (menuObj == 5) { // grid
    #ifdef DEBUG
      Serial.printf("tmp_grid is %s\n", tmp_grid);
    #endif
    lcd.setCursor(0,3);
    int i;
    for(i=0; i<=5; i++) {
      lcd.print(tmp_grid[i]);
    }
    // lcd.setCursor(0,3);
    // lcd.blink();
  }
  else if (menuObj == 6) { // satID of SAT1
    #ifdef DEBUG
      Serial.printf("tmp_satID[0] is %s\n", tmp_satID[0]);
    #endif
    lcd.setCursor(0,3);
    int i;
    for(i=0; i<5; i++) {
      lcd.print(tmp_satID[0][i]);
    }
  }
  else if (menuObj == 7) { // satID of SAT2
    #ifdef DEBUG
      Serial.printf("tmp_satID[1] is %s\n", tmp_satID[1]);
    #endif
    lcd.setCursor(0,3);
    int i;
    for(i=0; i<5; i++) {
      lcd.print(tmp_satID[1][i]);
    }
  }
  else if (menuObj == 8) { // satID of SAT3
    #ifdef DEBUG
      Serial.printf("tmp_satID[2] is %s\n", tmp_satID[2]);
    #endif
    lcd.setCursor(0,3);
    int i;
    for(i=0; i<5; i++) {
      lcd.print(tmp_satID[2][i]);
    }
  }
  else if (menuObj == 9) { // satID of SAT4
    #ifdef DEBUG
      Serial.printf("tmp_satID[3] is %s\n", tmp_satID[3]);
    #endif
    lcd.setCursor(0,3);
    int i;
    for(i=0; i<5; i++) {
      lcd.print(tmp_satID[3][i]);
    }
  }
  else if (menuObj == 10) { // Set Time manually
    #ifdef DEBUG
      Serial.printf("manual_time is %s\n", manual_time);
    #endif
    lcd.setCursor(0,3);  // 4th line of LCD

    char wholeTime[18];

    //if(gridGPS[0] == 0) {
    //  strcpy(gridGPS, "      ");    // Print empty
    //}
    snprintf(wholeTime, sizeof(wholeTime), "%s", manual_time);
  
    lcd.print(wholeTime);

  }
/*  else if (menuObj == 10) { // ssid
    char charDisp1[30];
    int charDisp1_len;
    int str_len;
    char str_ssid[21];  // Don't forget the termination null character
    ssid_mem.toCharArray(charDisp1, sizeof(charDisp1));
    charDisp1_len = sizeof(charDisp1);
    Serial.printf("charDisp1_len is %d\n", charDisp1_len);
    ////sprintf(cssid, "%s", ssid_mem);
    strncpy(str_ssid, charDisp1, 21);
    str_len = sizeof(str_ssid);
    Serial.printf("str_len is %d\n", str_len);
    lcd.setCursor(0,3);
    int i;
    for(i=0; i<str_len; i++) {
      lcd.print(str_ssid[i]);
    }
    // lcd.print(str_ssid);
    lcd.setCursor(0,3);
    lcd.blink();
  }
  else if (menuObj == 11) { // WiFi passwd
    char charDisp2[30];
    int charDisp2_len;
    int str_len;
    char str_pass[19];  // Don't forget the termination null character
    passwd_mem.toCharArray(charDisp2, sizeof(charDisp2));

    strncpy(str_pass, charDisp2, 19);
    str_len = sizeof(str_pass);
    Serial.printf("str_len is %d\n", str_len);
    lcd.setCursor(0,3);
    lcd.print(str_pass);
    lcd.setCursor(0,3);
    lcd.blink();
  }
*/
} // End lcd_menuValue()


/*
void clear_DISPLAY() {
  switch(inv_text)
  {
    case 0:
      inv_text = 1;
      break;
    case 1:
      inv_text = 0;
      break;
    default:
      inv_text = 0;
  }
  display.clearDisplay();
}
*/

void clear_lcd() {
  lcd.clear();
}

/*
void display_DISPLAY() {
  display.display();
}
*/


// Button Shift input
// Short press: Change tracking object
// Long press: Go to Park position. 180-Az, 89-El
void updateState_buttonShift() {
  // the button has been just pressed
  if (buttonStateShift == HIGH) {
      startPressed = millis();

  } 
  else if (buttonStateShift == LOW && shiftButtonMode == 1) {    // the button has just been released and within the menu
      endPressed = millis();
      holdTime = endPressed - startPressed;

      if (holdTime >= STARTPRESS_TIME && holdTime < LONGPRESS_TIME) {
        #ifdef DEBUG
          Serial.println("Shift Button within menu was short pressed, goto next menu config");
        #endif
        blinkInd = 0; // Set blinking cursor at the beginning
        switch(menuObject)  {
          case 0: // Az offset
              update_azOffset(tmp_azOffset);
              break;
          case 1: // El offset
              update_elOffset(tmp_elOffset);
              break;
          case 2: // Track Freq
              update_trackFreq(tmp_trackFreq);
              break;
          case 3: // Speed (PWM)
              update_speed(tmp_indexDC);
              break;
          case 4: // Tracking Accuracy
              update_trackAccuracy(tmp_trackAccuInd);
              break;
          case 5: // grid
              update_qth(tmp_grid);
              break;
          case 6: // satID of SAT1
              update_satID(tmp_satID[0], 0);
              break;
          case 7: // satID of SAT2
              update_satID(tmp_satID[1], 1);
              break;
          case 8: // satID of SAT3
              update_satID(tmp_satID[2], 2);
              break;
          case 9: // satID of SAT4
              update_satID(tmp_satID[3], 3);
              break;
          case 10:  // Set Time manually
              // do nothing
              break;
        }
        
        if (menuObject >= maxMenuObj) {   // If already in the last menu object go back to first
          menuObject = 0;  // Go to first object (grid loc)
        }
        else {
          menuObject++;  // Increase by 1
        }
        
        startPressed = 0;
      }

      else if (holdTime >= LONGPRESS_TIME && holdTime < 10000) { // Long pressed button should be not more than 10 seconds
        #ifdef DEBUG
          Serial.println("Shift Button within menu was LONG pressed, exiting menu.");
        #endif
        shiftButtonMode = 0; // Exiting menu
        autoButtonMode = 0;
        menuObject = 0;
        startPressed = 0;
        lcd.noBlink();
 
      }
      clear_lcd();
  }
  else if (buttonStateShift == LOW) {  // the button has just been released. Main function
      endPressed = millis();
      holdTime = endPressed - startPressed;

      if (holdTime >= STARTPRESS_TIME && holdTime < LONGPRESS_TIME) {
        #ifdef DEBUG
          Serial.println("Shift Button was short pressed, changing object");
        #endif
        if (objectState >= maxObj) {   // If already in the last object go back to first
          objectState = 0;  // Go to first object (Sun)
        }
        else {
          objectState++;  // Increase by 1
        }
        startPressed = 0;
      }

      else if (holdTime >= LONGPRESS_TIME && holdTime < 10000) { // Long pressed button should not be more than 10 seconds
          #ifdef DEBUG
            Serial.println("Shift Button was LONG pressed, entering menu.");
          #endif
          shiftButtonMode = 1; // Entering config menu
          autoButtonMode = 1;
          startPressed = 0;
          strncpy(tmp_azOffset, azOffset, 7); // Copy the azOffset to tmp_azOffset, to be used for updating its value.
          strncpy(tmp_elOffset, elOffset, 7); // Copy the elOffset to tmp_elOffset, to be used for updating its value.
          strncpy(tmp_trackFreq, trackFreq, 4); // Copy the trackFreq to tmp_trackFreq, to be used for updating its value.
          tmp_indexDC = indexDC + '0';
          tmp_trackAccuInd = trackAccuInd + '0';
          strncpy(tmp_grid, grid, 7); // Copy the grid to tmp_grid, to be used for updating its value.
          int i = 0;
          for (i=0; i<numSats; i++) {
            strncpy(tmp_satID[i], satID[i], 6); // Copy the satID[i] to tmp_satID[i], to be used for updating its value.
          }
      }
      clear_lcd();
  }
  
}
// End of updateState_buttonShift()


// Button Auto input
// Short press: Start Auto tracking
// Argument l_autoButtonMode: 0 for starting Auto tracking, 1 for pressing enter within config menu.
void updateState_AutoButton(int l_autoButtonMode) {
  // the Auto button has been just pressed
  if (buttonAutoState1 == HIGH) {
      startAutoPressed = millis();

  }
  if (buttonAutoState1 == LOW && l_autoButtonMode == 1) { // the Auto <Enter> button has just been released, Menu function

      endAutoPressed = millis();
      holdTime = endAutoPressed - startAutoPressed;
  
      if (holdTime >= STARTPRESS_TIME && holdTime < LONGPRESS_TIME) {
        #ifdef DEBUG
          Serial.println("Auto Button in menu was short pressed");
        #endif
        startAutoPressed = 0;
          
        switch(menuObject) {
          case 0:   // Az offset
            if(blinkInd >= 5) {
              blinkInd = 0;
            }
            else {
              blinkInd++;
            }
            break;
          case 1:   // El offset
            if(blinkInd >= 5) {
              blinkInd = 0;
            }
            else {
              blinkInd++;
            }
            break;
          case 2:   // Track Freq
            if(blinkInd >= 2) {
              blinkInd = 0;
            }
            else {
              blinkInd++;
            }
            break;
          case 5:   // grid
            if(blinkInd >= 5) {
              blinkInd = 0;
            }
            else {
              blinkInd++;
            }
            break;
          case 6:   // satID1
          case 7:   // satID2
          case 8:   // satID3
          case 9:   // satID4
            if(blinkInd >= 4) {
              blinkInd = 0;
            }
            else {
              blinkInd++;
            }
            break;
          case 10:   // Set Time manually
            if(blinkInd >= 14) {
              blinkInd = 2;
            }
            else {
              blinkInd++;
            }
            break;
        //  case 10:   // WiFi ssid
        //    if(blinkInd >= 5) {
        //      blinkInd = 0;
        //    }
        //    else {
        //      blinkInd++;
        //    }
        //    break;
        //  case 11:   // WiFi pass
        //    if(blinkInd >= 5) {
        //    blinkInd = 0;
        //    }
        //    else {
        //      blinkInd++;
        //    }
        //    break;
          default:
            Serial.println("Pending: Auto Press in menu!");
        }    
      }
      else if (holdTime >= LONGPRESS_TIME && holdTime < 10000) { // Long pressed button should not be more than 10 seconds
          #ifdef DEBUG
            Serial.println("Auto Button in menu was long pressed. Saving value in EEPROM...");
          #endif
          switch(menuObject)  {
            case 0: // Az offset
              update_azOffset(tmp_azOffset);
              break;
            case 1: // El offset
              update_elOffset(tmp_elOffset);
              break;
            case 2: // Track Freq
              update_trackFreq(tmp_trackFreq);
              break;
            case 3: // Speed (PWM)
              update_speed(tmp_indexDC);
              break;
            case 4: // Tracking Accuracy
              update_trackAccuracy(tmp_trackAccuInd);
              break;
            case 5: // grid
              update_qth(tmp_grid);
              break;
            case 6: // satID of SAT1
              update_satID(tmp_satID[0], 0);
              fetchTLEandTime();  // Connects to WiFi and gets the TLE of the satellites and the Time from NTP server.
              break;
            case 7: // satID of SAT2
              update_satID(tmp_satID[1], 1);
              fetchTLEandTime();  // Connects to WiFi and gets the TLE of the satellites and the Time from NTP server.
              break;
            case 8: // satID of SAT3
              update_satID(tmp_satID[2], 2);
              fetchTLEandTime();  // Connects to WiFi and gets the TLE of the satellites and the Time from NTP server.
              break;
            case 9: // satID of SAT4
              update_satID(tmp_satID[3], 3);
              fetchTLEandTime();  // Connects to WiFi and gets the TLE of the satellites and the Time from NTP server.
              break;
            case 10: // Set Time Manually
              // Do nothing
              break;
          }
          startAutoPressed = 0;
      }
      else {
          #ifdef DEBUG
            Serial.println("Auto Button was pressed too long in Menu, ignoring...");
          #endif
          startAutoPressed = 0;
      }  
      
  } // Auto <Enter> button released
  else if (buttonAutoState1 == LOW) { // the Auto button has just been released, Main function
      endAutoPressed = millis();
      holdTime = endAutoPressed - startAutoPressed;
  
      if (holdTime >= STARTPRESS_TIME && holdTime < LONGPRESS_TIME) {
        #ifdef DEBUG
          Serial.println("Auto Button was short pressed");
        #endif
        // if (tracking_state == 0) {
        if (autoTrack_state == 0 && tracking_state == 0) {
            autoTrack_state = 1;
            tracking_state = 1;   // Enable tracking
         
            statusTrackingObject[0] = 1; // Enable current period azimuth tracking
            statusTrackingObject[1] = 1; // Enable current period elevation tracking
            startTrackTime = millis();  // Store the start time of tracking
            func_track_object(obj_azim, obj_elev);
        }
        else {
            autoTrack_state = 0;
            tracking_state = 0;   // Disable tracking and stop motors
            #if defined(ENABLE_AZIMUTH)
              if(moving_state[0] != 0) { // If azimuth Motor is already moving
                Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);   // third param 0 for azimuth motor
              }
              park_state[0] = 0;  // To avoid continuing to park position
            #endif
            #if defined(ENABLE_ELEVATION)
              if(moving_state[1] != 0) {  // If elevation motor is already moving
                Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);   // third param 1 for elevation motor
              }
              park_state[1] = 0;  // To avoid continuing to park position
            #endif
        }
        
        startAutoPressed = 0;
      }

      else {
        #ifdef DEBUG
          Serial.println("Auto Button was pressed too long in Main function, ignoring...");
        #endif
        startAutoPressed = 0;
      }

  } // End of autoButtonMode 0

}
// End of updateState_AutoButton()


// Button Plus input
// Short press: 
// Argument1: l_PlusButtonMode, argument2: l_menuObj
void updateState_PlusButton(int l_plusButtonMode, int l_menuObj) {
  // the Plus button has been just pressed
  
    if (buttonPlusState1 == HIGH) {
      startPlusPressed = millis();
    } 
    
    else if ((buttonPlusState1 == LOW) && (l_plusButtonMode == 1)) {    // buttonAutoState1 == LOW , the Plus button has just been released, Menu function
        endPlusPressed = millis();
        holdTime = endPlusPressed - startPlusPressed;
  
        if (holdTime >= STARTPRESS_TIME && holdTime < LONGPRESS_TIME) {
          #ifdef DEBUG
            Serial.println("Plus Button was short pressed");
          #endif
          int asciiNum;
          switch(l_menuObj) {
            case 0:   // Az offset setting
              asciiNum = (int)tmp_azOffset[blinkInd];
              switch(blinkInd) {
                case 0:   // 1st Char + or -
                  if (asciiNum == 43) {    // If letter +, set it to -
                     asciiNum = 45;
                  }
                  else if (asciiNum == 45) {    // If letter -, set it to +
                     asciiNum = 43;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 1st char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                case 1:   // 2nd alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 2nd  asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                case 2:   // 3rd alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 3rd asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                case 3:   // 4th alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 4th asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                case 4:   // 5th Char "."
                  if (asciiNum != 46) {    // If letter is not . then error
                    Serial.printf("Error in Plus, 5th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  }
                  else {
                    asciiNum = 46;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 5th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                case 5:   // 6th alphanum range 0-9
                  asciiNum++; // go to next letter
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 6th asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                default:
                  Serial.printf("Plus, error. blinkInd is [%d]\n", blinkInd);
              } // End Az offset setting
              break;
            case 1:   // El offset setting
              asciiNum = (int)tmp_elOffset[blinkInd];
              switch(blinkInd) {
                case 0:   // 1st Char + or -
                  if (asciiNum == 43) {    // If letter +, set it to -
                     asciiNum = 45;
                  }
                  else if (asciiNum == 45) {    // If letter -, set it to +
                     asciiNum = 43;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 1st char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                case 1:   // 2nd alphanum range 0-9
                  
                  if (asciiNum != 48) {    // If alphanum not zero, then go to 0
                     asciiNum = 48;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 2nd  asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                case 2:   // 3rd alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 3rd asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                case 3:   // 4th alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 4th asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                case 4:   // 5th Char "."
                  if (asciiNum != 46) {    // If letter is not . then error
                    Serial.printf("Error in Plus, 5th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  }
                  else {
                    asciiNum = 46;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 5th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                case 5:   // 6th alphanum range 0-9
                  asciiNum++; // go to next letter
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 6th asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                default:
                  Serial.printf("Plus, error. blinkInd is [%d]\n", blinkInd);
              } // End El offset setting
              break;
            case 2:   // Freq Track setting
              asciiNum = (int)tmp_trackFreq[blinkInd];
              switch(blinkInd) {
                case 0:   // 1st alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_trackFreq[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 1st digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_trackFreq[blinkInd]);
                  #endif
                  break;
                case 1:   // 2nd alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_trackFreq[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 2nd digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_trackFreq[blinkInd]);
                  #endif
                  break;
                case 2:   // 3rd alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_trackFreq[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 3rd digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_trackFreq[blinkInd]);
                  #endif
                  break;
                default:
                  Serial.printf("Plus, error. blinkInd is [%d]\n", blinkInd);
              } // End Track Freq setting
              break;
            case 3:   // Speed setting
              asciiNum = (int)tmp_indexDC;
              // 1st alphanum range 0-2
              asciiNum++; // go to next alphanumeric
              if (asciiNum > 50) {    // If alphanum > 2, then go to 0
                asciiNum = 48;
              }
              tmp_indexDC = asciiNum;
              #ifdef DEBUG
                Serial.printf("Plus, 1st digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_indexDC);
              #endif
              break;
            case 4:   // Tracking accuracy setting
              asciiNum = (int)tmp_trackAccuInd;
              // 1st alphanum range 0-1
              asciiNum++; // go to next alphanumeric
              if (asciiNum > 49) {    // If alphanum > 1, then go to 0
                asciiNum = 48;
              }
              tmp_trackAccuInd = asciiNum;
              #ifdef DEBUG
                Serial.printf("Plus, 1st digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_trackAccuInd);
              #endif
              break;
            case 5:   // Grid setting
              asciiNum = (int)tmp_grid[blinkInd];
              switch(blinkInd) {
                case 0:   // 1st Char range A-R
                  asciiNum++; // go to next letter
                  if (asciiNum > 82) {    // If letter > R, then go to A
                     asciiNum = 65;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 1st char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                case 1:   // 2nd Char range A-R
                  asciiNum++; // go to next letter
                  if (asciiNum > 82) {    // If letter > R, then go to A
                     asciiNum = 65;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 2nd char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                case 2:   // 3rd alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 3rd asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                case 3:   // 4th alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 4th asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                case 4:   // 5th Char range A-Z
                  asciiNum++; // go to next letter
                  if (asciiNum > 90) {    // If letter > Z, then go to A
                     asciiNum = 65;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 5th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                case 5:   // 6th Char range A-Z
                  asciiNum++; // go to next letter
                  if (asciiNum > 90) {    // If letter > Z, then go to A
                     asciiNum = 65;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 6th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                default:
                  Serial.printf("Plus, error. blinkInd is [%d]\n", blinkInd);
              }
              break;
            case 6:   // satID1 setting
            case 7:   // satID2 setting
            case 8:   // satID3 setting
            case 9:   // satID4 setting
              asciiNum = (int)tmp_satID[l_menuObj-6][blinkInd];
              switch(blinkInd) {
                case 0:   // 1st alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_satID[l_menuObj-6][blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 1st digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_satID[l_menuObj-6][blinkInd]);
                  #endif
                  break;
                case 1:   // 2nd alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_satID[l_menuObj-6][blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 2nd digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_satID[l_menuObj-6][blinkInd]);
                  #endif
                  break;
                case 2:   // 3rd alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_satID[l_menuObj-6][blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 3rd digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_satID[l_menuObj-6][blinkInd]);
                  #endif
                  break;
                case 3:   // 4th alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_satID[l_menuObj-6][blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 4th digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_satID[l_menuObj-6][blinkInd]);
                  #endif
                  break;
                case 4:   // 5th alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  tmp_satID[l_menuObj-6][blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 5th digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_satID[l_menuObj-6][blinkInd]);
                  #endif
                  break;
                default:
                  Serial.printf("Plus, error. blinkInd is [%d]\n", blinkInd);
              } // End satID setting
              break; 
            case 10:   // Set Time manually
              asciiNum = (int)manual_time[blinkInd];
              switch(blinkInd) {
                case 0:   // 1st Char constant '2'
                  break;
                case 1:   // 2nd Char constant '0'
                  break;
                case 2:   // 3rd alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 3rd asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                case 3:   // 4th alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 4th asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                case 4:   // 5th alphanum range 0-1
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 49) {    // If alphanum > 1, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 5th asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                case 5:   // 6th alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 6th asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                case 6:   // 7th alphanum range 0-3
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 51) {    // If alphanum > 3, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 7th asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                case 7:   // 8th alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 8th asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                case 8:   // 9th Char constant '-'
                  break;
                case 9:   // 10th alphanum range 0-2
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 50) {    // If alphanum > 2, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 10th asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                case 10:   // 11th alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 11th asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                case 11:   // 12th alphanum range 0-5
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 53) {    // If alphanum > 5, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 12th asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                case 12:   // 13th alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 13th asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                case 13:   // 14th alphanum range 0-5
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 53) {    // If alphanum > 5, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 14th asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                case 14:   // 15th alphanum range 0-9
                  asciiNum++; // go to next alphanumeric
                  if (asciiNum > 57) {    // If alphanum > 9, then go to 0
                     asciiNum = 48;
                  }
                  manual_time[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 15th asciiNum is now [%d], char [%c]\n", asciiNum, manual_time[blinkInd]);
                  #endif
                  break;
                default:
                  Serial.printf("Plus, error. blinkInd is [%d]\n", blinkInd);
              }
              break;    
            default:
              Serial.println("Plus Button blinking change pending");
          }
                       
          startPlusPressed = 0;
        }
        else {
          Serial.println("Plus Button was pressed too long in Menu, ignoring..."); 
        }
    }
    else if (buttonPlusState1 == LOW) {    // Main config
        endPlusPressed = millis();
        holdTime = endPlusPressed - startPlusPressed;
  
        if (holdTime >= STARTPRESS_TIME && holdTime < LONGPRESS_TIME) {
          #ifdef DEBUG
            Serial.println("Plus Button was short pressed in the main config");
          #endif
          showLoc_flag = !showLoc_flag; // Set the opposite
        }
    }

  
}
// End of updateState_PlusButton()



// Button Minus input
// Short press: 
// Argument1: l_MinusButtonMode, argument2: l_menuObj
void updateState_MinusButton(int l_minusButtonMode, int l_menuObj) {
  // the Minus button has been just pressed
    if (buttonMinusState1 == HIGH) {
      startMinusPressed = millis();
    } 
    
    else if (buttonMinusState1 == LOW && l_minusButtonMode == 1) {    // buttonAutoState1 == LOW , the Minus button has just been released, Menu function
        endMinusPressed = millis();
        holdTime = endMinusPressed - startMinusPressed;
  
        if (holdTime >= STARTPRESS_TIME && holdTime < LONGPRESS_TIME) {
          #ifdef DEBUG
            Serial.println("Minus Button was short pressed");
          #endif
          int asciiNum;
          switch(l_menuObj) {
            case 0:   // Az offset setting
              asciiNum = (int)tmp_azOffset[blinkInd];
              switch(blinkInd) {
                case 0:   // 1st Char + or -
                  if (asciiNum == 43) {    // If letter +, set it to -
                     asciiNum = 45;
                  }
                  else if (asciiNum == 45) {    // If letter -, set it to +
                     asciiNum = 43;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 1st char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                case 1:   // 2nd alphanum range 0-9
                  asciiNum--; // go to next alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 2nd  asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                case 2:   // 3rd alphanum range 0-9
                  asciiNum--; // go to next alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 3rd asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                case 3:   // 4th alphanum range 0-9
                  asciiNum--; // go to next alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 4th asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                case 4:   // 5th Char "."
                  if (asciiNum != 46) {    // If letter is not . then error
                    Serial.printf("Error in Minus, 5th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  }
                  else {
                    asciiNum = 46;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 5th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                case 5:   // 6th alphanum range 0-9
                  asciiNum--; // go to next alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_azOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 6th asciiNum is now [%d], char [%c]\n", asciiNum, tmp_azOffset[blinkInd]);
                  #endif
                  break;
                default:
                  Serial.printf("Minus, error. blinkInd is [%d]\n", blinkInd);
              } // End Az offset setting
              break;
            case 1:   // El offset setting
              asciiNum = (int)tmp_elOffset[blinkInd];
              switch(blinkInd) {
                case 0:   // 1st Char + or -
                  if (asciiNum == 43) {    // If letter +, set it to -
                     asciiNum = 45;
                  }
                  else if (asciiNum == 45) {    // If letter -, set it to +
                     asciiNum = 43;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 1st char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                case 1:   // 2nd alphanum range 0-9
                  if (asciiNum != 48) {    // If alphanum not zero, then go to 0
                     asciiNum = 48;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 2nd  asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                case 2:   // 3rd alphanum range 0-9
                  asciiNum--; // go to next alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 3rd asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                case 3:   // 4th alphanum range 0-9
                  asciiNum--; // go to next alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 4th asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                case 4:   // 5th Char "."
                  if (asciiNum != 46) {    // If letter is not . then error
                    Serial.printf("Error in Minus, 5th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  }
                  else {
                    asciiNum = 46;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 5th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                case 5:   // 6th alphanum range 0-9
                  asciiNum--; // go to next alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_elOffset[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 6th asciiNum is now [%d], char [%c]\n", asciiNum, tmp_elOffset[blinkInd]);
                  #endif
                  break;
                default:
                  Serial.printf("Minus, error. blinkInd is [%d]\n", blinkInd);
              } // End El offset setting
              break;

            case 2:   // Freq Track setting
              asciiNum = (int)tmp_trackFreq[blinkInd];
              switch(blinkInd) {
                case 0:   // 1st Char + or -
                  asciiNum--; // 1st alphanum range 0-9
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_trackFreq[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 1st digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_trackFreq[blinkInd]);
                  #endif
                  break;
                case 1:   // 2nd alphanum range 0-9
                  asciiNum--; // go to next alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_trackFreq[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 2nd digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_trackFreq[blinkInd]);
                  #endif
                  break;
                case 2:   // 3rd alphanum range 0-9
                  asciiNum--; // go to next alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_trackFreq[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 3rd digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_trackFreq[blinkInd]);
                  #endif
                  break;
                default:
                  Serial.printf("Minus, error. blinkInd is [%d]\n", blinkInd);
              } // End Freq Track setting
              break;

            case 3:   // Speed setting
              asciiNum = (int)tmp_indexDC;
              // 1st alphanum range 0-2
                  asciiNum--; // go to previous alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 2
                     asciiNum = 50;
                  }
                  tmp_indexDC = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 1st digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_indexDC);
                  #endif
              break;
            case 4:   // Tracking accuracy setting
              asciiNum = (int)tmp_trackAccuInd;
              // 1st alphanum range 0-1
                  asciiNum--; // go to previous alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 1
                     asciiNum = 49;
                  }
                  tmp_trackAccuInd = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 1st digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_trackAccuInd);
                  #endif
              break;
            case 5:   // Grid setting
              asciiNum = (int)tmp_grid[blinkInd];
              switch(blinkInd) {
                case 0:   // 1st Char range A-R
                  asciiNum--; // go to previous letter
                  if (asciiNum < 65) {    // If letter < A, then go to R
                     asciiNum = 82;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 1st char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                case 1:   // 2nd Char range A-R
                  asciiNum--; // go to previous letter
                  if (asciiNum < 65) {    // If letter < A, then go to R
                     asciiNum = 82;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 2nd char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                case 2:   // 3rd alphanum range 0-9
                  asciiNum--; // go to next alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 3rd asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                case 3:   // 4th alphanum range 0-9
                  asciiNum--; // go to next alphanumeric
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 4th asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                case 4:   // 5th Char range A-Z
                  asciiNum--; // go to next letter
                  if (asciiNum < 65) {    // If letter < A, then go to Z
                     asciiNum = 90;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 5th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                case 5:   // 6th Char range A-Z
                  asciiNum--; // go to next letter
                  if (asciiNum < 65) {    // If letter < A, then go to Z
                     asciiNum = 90;
                  }
                  tmp_grid[blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Minus, 6th char asciiNum is now [%d], char [%c]\n", asciiNum, tmp_grid[blinkInd]);
                  #endif
                  break;
                default:
                  Serial.printf("Minus, error. blinkInd is [%d]\n", blinkInd);
              }
              break;
            case 6:   // satID1 setting
            case 7:   // satID2 setting
            case 8:   // satID3 setting
            case 9:   // satID4 setting
              asciiNum = (int)tmp_satID[l_menuObj-6][blinkInd];
              switch(blinkInd) {
                case 0:   // 1st alphanum range 0-9
                  asciiNum--; // 1st alphanum range 0-9
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_satID[l_menuObj-6][blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 1st digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_satID[l_menuObj-6][blinkInd]);
                  #endif
                  break;
                case 1:   // 2nd alphanum range 0-9
                  asciiNum--; // go to previous alphanum
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_satID[l_menuObj-6][blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 2nd digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_satID[l_menuObj-6][blinkInd]);
                  #endif
                  break;
                case 2:   // 3rd alphanum range 0-9
                  asciiNum--; // go to previous alphanum
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_satID[l_menuObj-6][blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 3rd digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_satID[l_menuObj-6][blinkInd]);
                  #endif
                  break;
                case 3:   // 4th alphanum range 0-9
                  asciiNum--; // go to previous alphanum
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_satID[l_menuObj-6][blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 4th digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_satID[l_menuObj-6][blinkInd]);
                  #endif
                  break;
                case 4:   // 5th alphanum range 0-9
                  asciiNum--; // go to previous alphanum
                  if (asciiNum < 48) {    // If alphanum < 0, then go to 9
                     asciiNum = 57;
                  }
                  tmp_satID[l_menuObj-6][blinkInd] = asciiNum;
                  #ifdef DEBUG
                    Serial.printf("Plus, 5th digit asciiNum is now [%d], char [%c]\n", asciiNum, tmp_satID[l_menuObj-6][blinkInd]);
                  #endif
                  break;
                default:
                  Serial.printf("Plus, error. blinkInd is [%d]\n", blinkInd);
              } // End satID setting
              break;
            case 10: // Store the Time set manually
              set_time_manual(manual_time);
              break;
            default:
              Serial.println("Minus Button blinking change pending");
          }
                       
          startMinusPressed = 0;
        }
        else {
          Serial.println("Minus Button was pressed too long in Menu, ignoring..."); 
        }
    }
    else if (buttonMinusState1 == LOW) {    // Main config
      endMinusPressed = millis();
      holdTime = endMinusPressed - startMinusPressed;
  
      if (holdTime >= STARTPRESS_TIME && holdTime < LONGPRESS_TIME) {
        #ifdef DEBUG
          Serial.println("Minus Button was short pressed in the main config");
        #endif

      }
    }
  
}
// End of updateState_MinusButton()


/*
// Config Menu functions
void set_WiFi_cred(int menu_obj) {
  if (menu_obj == 0 || menu_obj == 1 || menu_obj == 2) {  // grid or WiFi ssid or pass
    display_menuValue(menu_obj, inv_text);
    // ssid_mem = preferences.getString("ssid", "");
  }
}
*/


// Config Menu function set_menuValue
void set_menuValue(int menu_obj) {

    lcd_menuValue(menu_obj);
    lcd.setCursor(blinkInd,3);
    lcd.blink();
}


// ### Function fetchTLEandTime(). Connects to WiFi, fetches the TLE for the satellites and the Time from the NTP server.
void fetchTLEandTime() {
  tle_time_lcd(); // Update LCD with status
  int startNetTime;
  int currentNetTime;
  //connect to WiFi in order to get time from NTP server once
//  Serial.printf("Connecting to %s ", ssid_mem.c_str());
  Serial.printf("Connecting to %s ", wifi_ssid);
  startNetTime = millis();  // Store the start time of WiFi connection
//  WiFi.begin(ssid, password);
//  WiFi.begin(ssid_mem.c_str(), passwd_mem.c_str());
  WiFi.begin(wifi_ssid, wifi_passwd);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      currentNetTime = millis();
      if (currentNetTime - startNetTime > 15000) {
        Serial.println("WiFi connection error!");
        error_lcd(1);
        break;
      }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" CONNECTED");
    Serial.print(" IP address: ");
    Serial.println(WiFi.localIP());
    delay(100);
    //init and get the time
    Serial.print(" Init Time: ");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    delay(200);
    if(getTimeNTP() == 0)
    {
      Serial.println(" Failed to get time from NTP server! ");
      nTP_flag = false;
      error_lcd(2);
    }
    else {
      nTP_flag = true;
      selfTime_flag = false;
      tle_time_lcd_success();   // Update LCD with success status
    }
  }

  sat.site(latitude,longitude,myAlt); //set location latitude[°], longitude[°] and altitude[m]
//  unixtime = epochTime;
  timeNow = epochTime;

  // Get TLEs //
  for(SAT = 0; SAT < numSats; SAT++){
    tmp_URL = sub_URL;
    temp_satID = satID[SAT];
    tmp_URL += temp_satID;
    tmp_URL.toCharArray(satURL[SAT], sizeof(satURL[SAT]));
    Serial.printf("satID[SAT], SAT=[%d], satID=[%s]\n", SAT, satID[SAT]);
    Serial.printf("satURL[SAT]: [%s]\n", satURL[SAT]);
    getTLE(SAT);
    retrieve_satName(satNameTLE[SAT], SAT);
    // sat.init(satname,TLE1[SAT],TLE2[SAT]);     //initialize satellite parameters
    sat.init(satelName[SAT],TLE1[SAT],TLE2[SAT]);     //initialize satellite parameters
    sat.findsat(timeNow);
    // upcomingPasses[SAT] = Predict(1);
    upcomingPasses[SAT] = Predict(1, &azimStart, &elevMax);
  }
//  nextSat = nextSatPass(upcomingPasses);
//  sat.init(satname,TLE1[nextSat],TLE2[nextSat]);
//  Predict(1);

  // Print obtained TLE in serial. //
  #ifdef DEBUG
    for(SAT=0; SAT < numSats; SAT++){
      Serial.println("TLE set #:" + String(SAT));
      for(i=0;i<25;i++) {     
        Serial.print(satNameTLE[SAT][i]); 
      }
      Serial.println();
      for(i=0;i<70;i++) {     
        Serial.print(TLE1[SAT][i]); 
      }
      Serial.println();
      for(i=0;i<70;i++) {
        Serial.print(TLE2[SAT][i]);
      }
      Serial.println();
    }  
    // Serial.println("Next satellite: " + String(nextSat));
  #endif


  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println(" ");
  // Serial.printf("Disconnecting from %s ", ssid_mem.c_str());
  Serial.printf("Disconnecting from %s ", wifi_ssid);
  while (WiFi.status() == WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" DISCONNECTED");
  // End connections to WiFi 
} // End of fetchTLEandTime()


void init_test_motor() {
  delay(DIR_DELAY);
  Serial.println("select direction of movement");
  Serial.println("1.forward");
  Serial.println("2.fast_forward");
  Serial.println("3.backward");
  Serial.println("4.fast_backward");
  Serial.println("5.stop");
  Serial.println("6.soft_stop");
  Serial.println("a.start tracking");
  Serial.println("s.stop tracking");
}


//----------------------------------------------------------------
//  This function gets called about once per second, during the GPS
//  quiet time.  It's the best place to do anything that might take
//  a while: print a bunch of things, write to SD, send an SMS, etc.
//
//  By doing the "hard" work during the quiet time, the CPU can get back to
//  reading the GPS chars as they come in, so that no chars are lost.

static void doSomeWork()
{
  // Print all the things!

  trace_all( Serial, gps, fix );

  //  This is the best place to do your time-consuming work, right after
  //     the RMC sentence was received.  If you do anything in "loop()",
  //     you could cause GPS characters to be lost, and you will not
  //     get a good lat/lon.
  //  For this example, we just print the lat/lon.  If you print too much,
  //     this routine will not get back to "loop()" in time to process
  //     the next set of GPS data.

  if (fix.valid.date) {
    //NeoGPS::time_t epochTimeNowGPS;
    epochTimeGPS = (NeoGPS::clock_t) fix.dateTime + 946684800;  // Add 946.684.800 seconds to convert Y2K to POSIX 
    // epochTimeNowGPS = (NeoGPS::clock_t) epochTimeGPS;
    tv.tv_sec = epochTimeGPS; // UnixTime
    settimeofday(&tv, NULL);    // Set the system time
    #ifdef DEBUG
      Serial.println(epochTimeGPS);
    
      char wholeTime[20];
    // lDay = fix.dateTime.date;
    // lMonth = fix.dateTime.month;
    // lYear = fix.dateTime.year + 2000;
    // lHours = fix.dateTime.hours;
    // lMinutes = fix.dateTime.minutes;
    // lSeconds = fix.dateTime.seconds;
      int lDay1 = fix.dateTime.date;
      int lMonth1 = fix.dateTime.month;
      int lYear1 = fix.dateTime.year + 2000;
      int lHours1 = fix.dateTime.hours;
      int lMinutes1 = fix.dateTime.minutes;
      int lSeconds1 = fix.dateTime.seconds;

      // snprintf(wholeTime, sizeof(wholeTime), "%04d-%02d-%02d %02d:%02d:%02d", lYear, lMonth, lDay, lHours, lMinutes, lSeconds);
      snprintf(wholeTime, sizeof(wholeTime), "%04d-%02d-%02d %02d:%02d:%02d", lYear1, lMonth1, lDay1, lHours1, lMinutes1, lSeconds1);
      Serial.print(wholeTime);
      Serial.print( ',' );
    #endif
    gpsTime_flag = true;
  }
  else {
    gpsTime_flag = false;
  }
    
  if (fix.valid.location) {
    #ifdef DEBUG
      Serial.print( fix.latitude(), 6 ); // floating-point display
      // DEBUG_PORT.print( fix.latitudeL() ); // integer display
      // printL( DEBUG_PORT, fix.latitudeL() ); // prints int like a float
      Serial.print( ',' );
      Serial.print( fix.longitude(), 6 ); // floating-point display
      // DEBUG_PORT.print( fix.longitudeL() );  // integer display
      // printL( DEBUG_PORT, fix.longitudeL() ); // prints int like a float
    #endif

    latitude = fix.latitude();    // Assign the latitude of GPS
    longitude = fix.longitude();    // Assign the longitude of GPS
    udtLocation.dLatitude = latitude;
    udtLocation.dLongitude = longitude;
    char* locator=get_mh(latitude, longitude, 6);
    strcpy(gridGPS, locator);

    #ifdef DEBUG
      Serial.print( ',' );
      if (fix.valid.satellites)
        Serial.print( fix.satellites );

      Serial.println();
      Serial.print("GPS Grid Locator: ");
      //Serial.println(locator);
      Serial.println(gridGPS);
    #endif
    gpsLOC_flag = true;
  } else {
    // No valid location data yet!
    #ifdef DEBUG
      Serial.print( '?' );
    #endif
    gpsLOC_flag = false;
  }
  #ifdef DEBUG
    Serial.println();
    Serial.println();
  #endif

} // doSomeWork

/*
// Function displayGPSInfo()
void displayGPSInfo()
{ 
  Serial.print("GPS Date and Time: ");
  if (gps.date.isValid() && gps.time.isValid())
  {
    time_t epochTimeNowGPS;
    
    timeinfoGPS.Day = gps.date.day();
    Serial.print(gps.date.day());
    Serial.print("/");
    timeinfoGPS.Month = gps.date.month();
    Serial.print(gps.date.month());
    Serial.print("/");
    timeinfoGPS.Year = gps.date.year() - 1970;
    Serial.println(gps.date.year());

    // Serial.println(gps.time.value());
    timeinfoGPS.Hour = gps.time.hour();
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    timeinfoGPS.Minute = gps.time.minute();
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    timeinfoGPS.Second = gps.time.second();
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());

    epochTimeNowGPS = makeTime(timeinfoGPS);
    // rtcESP32.setTime(epochTimeNowGPS);    // Store the epoch GPS time in internal RTC. 
    setUnixtime(epochTimeNowGPS); // Set the system time.
    epochTimeGPS = epochTimeNowGPS;
    Serial.print("epochTimeGPS: ");
    Serial.println(epochTimeGPS);
    
  }
  else
  {
    Serial.println("GPS Date-Time Not Available");
  }

  if (gps.location.isValid())
  {
    Serial.print("GPS Latitude: ");
    Serial.println(gps.location.lat(), 6);
    double lat1 = gps.location.lat();
    Serial.print("GPS Longitude: ");
    Serial.println(gps.location.lng(), 6);
    double lng1 = gps.location.lng();
    Serial.print("GPS Altitude: ");
    Serial.println(gps.altitude.meters());

    char* locator=get_mh(lat1, lng1, 6);
    Serial.print("GPS Grid Locator: ");
    Serial.println(locator);
  }
  else
  {
    Serial.println("GPS Location: Not Available");
  }

  Serial.println();
  Serial.println();
  // delay(1000);
}
// End of displayGPSInfo() function
*/

/* This function sets the internal time. The timeval
   struct is seconds, milliseconds. settimeofday parameters are
   epoch_time, timezone. 
*/
/*
int setUnixtime(int32_t unixtime) {
  timeval epoch = {unixtime, 0};
  return settimeofday((const timeval*)&epoch, 0);
}
*/
// *** End functions ***

// *** SETUP function ***
void setup()
{
//  int startNetTime;
//  int currentNetTime;
  gridGPS[0] = 0;   // Initializing gridGPS to empty string.

  // MySerial_GPS.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(9600);
  Serial.flush();
  Serial.println("Begin setup");

  Serial.print("Deltanu NMEAGPS: started\n");
  Serial.print("  fix object size = ");
  Serial.println( sizeof(gps.fix()) );
  Serial.print("  gps object size = ");
  Serial.println( sizeof(gps) );
  Serial.printf("Looking for GPS device on [%s]\n", GPS_PORT_NAME);

  #ifndef NMEAGPS_RECOGNIZE_ALL
    #error You must define NMEAGPS_RECOGNIZE_ALL in NMEAGPS_cfg.h!
  #endif

  #ifdef NMEAGPS_INTERRUPT_PROCESSING
    #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
  #endif

  #if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
      !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
      !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
      !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST )

    Serial.println( F("\nWARNING: No NMEA sentences are enabled: no fix data will be displayed.") );

  #else
    if (gps.merging == NMEAGPS::NO_MERGING) {
      Serial.print  ( F("\nWARNING: displaying data from ") );
      Serial.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
      Serial.print  ( F(" sentences ONLY, and only if ") );
      Serial.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
      Serial.println( F(" is enabled.\n"
                            "  Other sentences may be parsed, but their data will not be displayed.") );
    }
  #endif

  Serial.print  ( F("\nGPS quiet time is assumed to begin after a ") );
  Serial.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
  Serial.println( F(" sentence is received.\n"
                        "  You should confirm this with NMEAorder.ino\n") );

  // trace_header( Serial );

  gpsPort.begin( 9600 );

  gpsStatus = 1; // Check GPS

  pinMode(ledPin, OUTPUT); // visual signal of chip initialization, onboard LED
  digitalWrite(ledPin, HIGH); // signal start of transfer with LED
  delay(2000);
  digitalWrite(ledPin, LOW);

  //Create a namespace called "gpio"
  preferences.begin("gpio", false);
  pinMode(buttonShiftPin, INPUT);
  pinMode(buttonAutoPin, INPUT);
  pinMode(buttonPlusPin, INPUT);
  pinMode(buttonMinusPin, INPUT);

  // read the last tracking object from flash memory
  objectState = preferences.getInt("state", 0); 
  Serial.printf("Object state before reset: %d \n", objectState);
  // set the tracking object to the last stored state
  trackingObject = objectState; 

  //Create a second namespace called "myPrefs"
  preferences.begin("myPrefs", false);

  // read the stored azimuth offset from memory
  azOffset_mem = preferences.getString("az_offset", "");
  if (azOffset_mem != "") {
    Serial.printf("Retrieved the saved azimuth offset [%s] from memory.\n", azOffset_mem);
    azOffset_mem.toCharArray(azOffset, sizeof(azOffset));
    azimuth_offset = (float)atof(azOffset);
    Serial.printf("Retrieved the saved azimuth offset, floating is [%f]\n", azimuth_offset);
  }
  else {
    Serial.println("No values saved for az offset. Using the default.");
    sprintf(azOffset, "%06.1f", azimuth_offset);
    Serial.printf("azOffset is %s\n", azOffset);
    Serial.printf("azimuth_offset is %f\n", azimuth_offset);
  }

  // read the stored elevation offset from memory
  elOffset_mem = preferences.getString("el_offset", "");
  if (elOffset_mem != "") {
    Serial.printf("Retrieved the saved elevation offset [%s] from memory.\n", elOffset_mem);
    elOffset_mem.toCharArray(elOffset, sizeof(elOffset));
    elevation_offset = (float)atof(elOffset);
    Serial.printf("Retrieved the saved elevation offset, floating is [%f]\n", elevation_offset);
  }
  else {
    Serial.println("No values saved for el offset. Using the default.");
    sprintf(elOffset, "%06.1f", elevation_offset);
    Serial.printf("elOffset is %s\n", elOffset);
    Serial.printf("elevation_offset is %f\n", elevation_offset);
  }
  

  // read the stored tracking frequency from memory
  trackFreq_mem = preferences.getString("track_freq", "");
  if (trackFreq_mem != "") {
    Serial.printf("Retrieved the saved tracking freq [%s] from memory.\n", trackFreq_mem);
    trackFreq_mem.toCharArray(trackFreq, sizeof(trackFreq));
    // int_trackFreq = atoi(trackFreq);
    sscanf(trackFreq, "%d", &int_trackFreq);
    Serial.printf("Retrieved the saved tracking freq, integer is [%d]\n", int_trackFreq);
  }
  else {
    Serial.println("No values saved for tracking freq. Using the default.");
    sprintf(trackFreq, "%03d", int_trackFreq);
    Serial.printf("trackFreq is %s\n", trackFreq);
    Serial.printf("trackFreq is %d\n", int_trackFreq);
  }


  // read the stored antenna speed from memory (0:Low, 1:Mid, 2:High)
  indexDC_mem = preferences.getChar("index_DutyCycle", '1');   // Default is '1' (ascii 49)
  indexDC = indexDC_mem - '0';
  Serial.printf("Retrieved the indexDC_mem [%d] from memory. indexDC is [%d]\n", indexDC_mem, indexDC);
  if(indexDC >= 0 && indexDC <= 2) {
    dutyCycleAz = menuDutyCycle[indexDC];
    Serial.printf("menuDutyCycle[indexDC] is [%d]\n", menuDutyCycle[indexDC]);
    dutyCycleEl = menuDutyCycle[indexDC];
  }
  else {
    Serial.println("Error on retrieving dutyCycle");
  }


  // read the stored auto tracking accuracy from memory (0:0.1 degree, 1:1 degree)
  trackAccuInd_mem = preferences.getChar("track_accuracy", '0');   // Default is '0' (ascii 48)
  trackAccuInd = trackAccuInd_mem - '0';
  Serial.printf("Retrieved the trackAccuInd_mem [%d] from memory. trackAccuInd is [%d]\n", trackAccuInd_mem, trackAccuInd);
  if(trackAccuInd >= 0 && trackAccuInd <=1) {
    Serial.printf("trackAccuInd is valid\n");
  }
  else {
    Serial.println("Error on retrieving trackAccuInd");
  }
  
  
  // read the stored grid square from flash memory
  grid_mem = preferences.getString("grid", "");
  if (grid_mem != "") {
    Serial.printf("Retrieved the saved grid [%s] from memory.\n", grid_mem);
    grid_mem.toCharArray(grid, sizeof(grid));
  }
  else {
    Serial.println("No values saved for grid. Using the default.");
  }


  // read the stored satellite Celestrak IDs from flash memory
  satID1_mem = preferences.getString("sat_id1", "");
  if (satID1_mem != "") {
    Serial.printf("Retrieved the saved satID1_mem [%s] from memory.\n", satID1_mem);
    satID1_mem.toCharArray(satID[0], sizeof(satID[0]));
  }
  else {
    Serial.println("No values saved for satID[0]. Using the default.");
  }

  // read the stored satellite Celestrak IDs from flash memory
  satID2_mem = preferences.getString("sat_id2", "");
  if (satID2_mem != "") {
    Serial.printf("Retrieved the saved satID2_mem [%s] from memory.\n", satID2_mem);
    satID2_mem.toCharArray(satID[1], sizeof(satID[1]));
  }
  else {
    Serial.println("No values saved for satID[1]. Using the default.");
  }

  // read the stored satellite Celestrak IDs from flash memory
  satID3_mem = preferences.getString("sat_id3", "");
  if (satID3_mem != "") {
    Serial.printf("Retrieved the saved satID3_mem [%s] from memory.\n", satID3_mem);
    satID3_mem.toCharArray(satID[2], sizeof(satID[2]));
  }
  else {
    Serial.println("No values saved for satID[2]. Using the default.");
  }

  // read the stored satellite Celestrak IDs from flash memory
  satID4_mem = preferences.getString("sat_id4", "");
  if (satID4_mem != "") {
    Serial.printf("Retrieved the saved satID4_mem [%s] from memory.\n", satID4_mem);
    satID4_mem.toCharArray(satID[3], sizeof(satID[3]));
  }
  else {
    Serial.println("No values saved for satID[3]. Using the default.");
  }
  
  // read the stored WiFi creds from flash memory
  ssid_mem = preferences.getString("ssid", "");
  passwd_mem = preferences.getString("password", "");
/*  if (ssid_mem == "" || passwd_mem == "") {
    Serial.println("No values saved for ssid and passwd. Using the default.");
    ssid_mem = ssid;
    passwd_mem = password;
    Serial.printf("ssid_mem is: %s , passwd_mem is: %s \n", ssid_mem.c_str(), passwd_mem.c_str());
  }
  */
  if (ssid_mem != "" && passwd_mem != "") {
    Serial.printf("Retrieved the saved ssid [%s] and pass [%s] from memory.\n", ssid_mem.c_str(), passwd_mem.c_str());
    ssid_mem.toCharArray(wifi_ssid, sizeof(wifi_ssid));
    passwd_mem.toCharArray(wifi_passwd, sizeof(wifi_passwd));
  }
  else {
    Serial.println("No values saved for wifi ssid and pass. Using the default.");
    Serial.printf("wifi_ssid [%s], wifi_passwd [%s]\n", wifi_ssid, wifi_passwd);
  }
  

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
  
//  init_display();
  init_lcd();
  init_lcd2();

  ////pinMode(ledPin, OUTPUT); // visual signal of I/O to chip: heartbeat
  ////pinMode(clockPin, OUTPUT); // SCK HSPI
  ////pinMode(CSnPin, OUTPUT); // CSn HSPI -- has to toggle high and low to signal chip to start data transfer
  ////pinMode(inputPin, INPUT); // SDA HSPI
  azimuth_hh12.initialize(hspi_clk, az_hspi_cs, hspi_miso);
  elevation_hh12.initialize(hspi_clk, el_hspi_cs, hspi_miso);


  fetchTLEandTime();  // Connects to WiFi and gets the TLE of the satellites and the Time from NTP server.

  // az_motor_init();
  #ifdef ENABLE_AZIMUTH
    motor_init(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, pwmChanAz); // Init AZ motor
  #endif
  #ifdef ENABLE_ELEVATION
    motor_init(MOTOR_EL1_PIN, MOTOR_EL2_PIN, pwmChanEl); // Init EL motor
  #endif
  init_test_motor();    // TEMP!!!

  clear_lcd();

  Serial.print("End setup");
}

int  input = 0;   // TEMP!!!


// ### loop() function 
void loop()
{
  if(nTP_flag == false || showLoc_flag == true) {
    // MySerial_GPS.flush();

    time_t now;
    // struct tm timeDetails;

  // if(gpsStatus == 1) {
    
    while (gps.available( gpsPort )) {
      fix = gps.read();
      doSomeWork();
      // gpsStatus = 2;  // Set to internal RTC if GPS gets lost in next loop.
      gpsStatus = 1;
      gPS_flag = true;
      // startGPSdata = millis();
      checkGPSdata = millis();
      //time(&now);
      //localtime_r(&now, &timeDetails);
    
      //Serial.print("The GPS current date/time is ");
      //Serial.println(&timeDetails, "%A, %B %d %Y %H:%M:%S");
    
    }

    time(&now);
    localtime_r(&now, &timeDetails);
      
    if(millis() - checkGPSdata >= 30000) {
      #ifdef DEBUG
        Serial.println("Lost GPS data. ");
      #endif
      gpsStatus = 0;  // Set to internal RTC if GPS gets lost in next loop.
      // startGPSdata = millis();
    }
  
   // }

  } // nTP_flag == false
  
  check_serial();
//  serial_menu_motor_test(input);
  input = 0;   // TEMP!!!

/*
  if((park_state[0] != 0) || (park_state[1] != 0)) {
    goto_park();  // Going to parking position
    goto_state[0] = 0;  // Disable GOTO AZ if in progress
    goto_state[1] = 0;  // Disable GOTO EL if in progress
    
  }
*/

// The following code is actually only used for serial commands 
  if (goto_state[0] != 0) {
    if (goto_az_value > float(ANT_AZ_MIN) && goto_az_value < float(ANT_AZ_MAX)) {
      request_command("AZ", "GOTO", goto_az_value);
      park_state[0] = 0; // Disable parking of azimuth if in progress.
      tracking_state = 0;   // Disable tracking
    }
    else {
      #ifdef DEBUG
        Serial.println(" Error: AZ GOTO position commanded out of limits! ");
      #endif
      request_command("AZ", "STOP", 0);
      #if defined(ENABLE_ELEVATION)
        request_command("EL", "STOP", 0);
      #endif
    }
  }
  else {
    goto_az_value = 0;
  }

  if (goto_state[1] != 0) {
    if (goto_el_value > float(ANT_EL_MIN) && goto_el_value < float(ANT_EL_MAX)) {
      request_command("EL", "GOTO", goto_el_value);
      park_state[1] = 0; // Disable parking of elevation if in progress.
      tracking_state = 0;   // Disable tracking
    }
    else {
      #ifdef DEBUG
        Serial.println(" Error: EL GOTO position commanded out of limits! ");
      #endif
      request_command("EL", "STOP", 0);
    }
  }
  else {
    goto_el_value = 0;
  }
// End of serial commands section


  // Check if Auto button has been pressed
  buttonAutoState1 = digitalRead(buttonAutoPin); // read the buttonAuto input
  if (buttonAutoState1 != lastButtonAutoState1) { // Auto button state changed
    updateState_AutoButton(autoButtonMode);
    // The above three lines did not help a lot
    if(buttonAutoState1 == HIGH) {
      lastButtonAutoState1 = buttonAutoState1;        // save state for next loop
      return;  // Go to beginning of main loop() for fast response of button change
    //
    }
  }
  lastButtonAutoState1 = buttonAutoState1;        // save state for next loop

  // Check if Shift buttonShift has been pressed (short or long pressed)
  buttonStateShift = digitalRead(buttonShiftPin); // read the buttonShift input
  if (buttonStateShift != lastbuttonStateShift) { // button state changed
     updateState_buttonShift();
     // The following three lines did not help a lot
     if(buttonStateShift == HIGH) {
       lastbuttonStateShift = buttonStateShift;        // save state for next loop
       return;  // Go to beginning of main loop() for fast response of button change
     //
     }
  }
  lastbuttonStateShift = buttonStateShift;        // save state for next loop


  // Check if Plus button has been pressed
      buttonPlusState1 = digitalRead(buttonPlusPin); // read the buttonPlus input
      if (buttonPlusState1 != lastButtonPlusState1) { // Plus button state changed
        // updateState_PlusButton(plusButtonMode, menuObject);
        updateState_PlusButton(shiftButtonMode, menuObject);
        // The above three lines did not help a lot
        if(buttonPlusState1 == HIGH) {
          lastButtonPlusState1 = buttonPlusState1;        // save state for next loop
          return;  // Go to beginning of main loop() for fast response of button change
        //
        }
      }
      lastButtonPlusState1 = buttonPlusState1;        // save state for next loop

      // Check if Minus button has been pressed
      buttonMinusState1 = digitalRead(buttonMinusPin); // read the buttonMinus input
      if (buttonMinusState1 != lastButtonMinusState1) { // Minus button state changed
        // updateState_MinusButton(minusButtonMode, menuObject);
        updateState_MinusButton(shiftButtonMode, menuObject);
        // The above three lines did not help a lot
        if(buttonMinusState1 == HIGH) {
          lastButtonMinusState1 = buttonMinusState1;        // save state for next loop
          return;  // Go to beginning of main loop() for fast response of button change
        //
        }
      }
      lastButtonMinusState1 = buttonMinusState1;        // save state for next loop

      

  if (trackingObject != objectState) {
    #ifdef DEBUG
      Serial.println("Changing object tracking ");
    #endif
    tracking_state = 0;   // Disable tracking
    #if defined(ENABLE_AZIMUTH)
      if(moving_state[0] != 0) { // If azimuth Motor is already moving
        Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);
      }
    #endif
    #if defined(ENABLE_ELEVATION)
      if(moving_state[1] != 0) {  // If elevation motor is already moving
        Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);
      }
    #endif
    // change the tracking object state
    trackingObject = objectState;

    // save the tracking object state in flash memory
    preferences.putInt("state", objectState);
    #ifdef DEBUG
      Serial.printf("Object State saved: %d \n", objectState);
    #endif
    
  }


//  prev_azAngle = az_floatAngle;   // Used for checking if absolute encoder changes value when antenna is moving.
//  prev_elAngle = el_floatAngle; 

// CSn needs to cycle from high to low to initiate transfer. Then clock cycles. As it goes high
// again, data will appear on sda

  digitalWrite(el_hspi_cs, HIGH); // Disable the EL encoder before using the AZ
  #if defined(OPTION_REVERSE_AZ_HH12_AS5045)
    az_floatAngle = 360.0 - (azimuth_hh12.heading() + azimuth_offset);  // Read the azimuth angle from the hh-12
  #else
    az_floatAngle = azimuth_hh12.heading() + azimuth_offset;  // Read the azimuth angle from the hh-12
  #endif
  if (az_floatAngle > 359.9) {
    az_floatAngle = 360.0 - az_floatAngle;
  }
  if (az_floatAngle < 0.1) {
    az_floatAngle = 360.0 + az_floatAngle;
  }
  // #ifdef DEBUG
    // Serial.print("az_floatAngle degrees: "); // and, finally, print it.
    Serial.print("#AZ"); // and finally, print it. This is used for sending the degrees to the Serial port to be captured by PC software (Easycomm protocol)
    Serial.println(az_floatAngle, 1);  // 1 decimal places
  // #endif

  digitalWrite(az_hspi_cs, HIGH); // Disable the AZ encoder before using the EL
  #if defined(OPTION_REVERSE_EL_HH12_AS5045)
    el_floatAngle = 360.0 - elevation_hh12.heading() + elevation_offset;  // Read the elevation angle from the hh-12 inc
  #else
    el_floatAngle = elevation_hh12.heading() + elevation_offset;  // Read the elevation angle from the hh-12 inc
  #endif
  // #ifdef DEBUG
    // Serial.print("el_floatAngle degrees: "); // and, finally, print it.
    Serial.print("#EL"); // and finally, print it. This is used for sending the degrees to the Serial port to be captured by PC software (Easycomm protocol)
    Serial.println(el_floatAngle, 1);  // 1 decimal places
  // #endif

//  display_angle(az_floatAngle, el_floatAngle);
//  display_AntStatus(moving_state[0], moving_state[1]);
  lcd_angle(az_floatAngle, el_floatAngle);
  lcd_AntStatus(moving_state[0], moving_state[1], tracking_state);

////  packeddata = 0; // reset both variables to zero so they don't just accumulate
////  angle = 0;

  if(nTP_flag == true) {
    if(getTimeNTP() != 0) {
      lcd_time(78, showLoc_flag); // 'N' ascii code 78
      timeNow = epochTime;
    }
    else {
      Serial.println("getTimeNTP() == 0 , should not enter here");
    }
  }
  else if(gPS_flag == true) {    // nTP_flag == false
    timeinfo = timeDetails;
    if(gpsStatus == 0) {
      lcd_time(33, showLoc_flag);  // '!' ascii code 33, to show that GPS data were lost.
    }
    else {
      lcd_time(89, showLoc_flag); // 'Y' == ascii code 89, to show that is GPS time
    }
    timeNow = epochTimeGPS;
  }
  else {
    Serial.println("Time error, cannot fetch it");
  }

//  if(nTP_flag == true || gPS_flag == true || selfTime_flag == true) {


    if (shiftButtonMode != 0) {  // Have entered the config menu
      tracking_state = 0;   // Disable auto tracking
      #if defined(ENABLE_AZIMUTH)
        if(moving_state[0] != 0) { // If azimuth Motor is already moving
          Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop Azimuth motor
        }  
      #endif
      #if defined(ENABLE_ELEVATION)
        if(moving_state[1] != 0) { // If elevation Motor is already moving
          Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop Elevation motor
        }
      #endif

/*
      // Check if Plus button has been pressed
      buttonPlusState1 = digitalRead(buttonPlusPin); // read the buttonPlus input
      if (buttonPlusState1 != lastButtonPlusState1) { // Plus button state changed
        // updateState_PlusButton(plusButtonMode, menuObject);
        updateState_PlusButton(shiftButtonMode, menuObject);
        // The above three lines did not help a lot
        if(buttonPlusState1 == HIGH) {
          lastButtonPlusState1 = buttonPlusState1;        // save state for next loop
          return;  // Go to beginning of main loop() for fast response of button change
        //
        }
      }
      lastButtonPlusState1 = buttonPlusState1;        // save state for next loop

      // Check if Minus button has been pressed
      buttonMinusState1 = digitalRead(buttonMinusPin); // read the buttonMinus input
      if (buttonMinusState1 != lastButtonMinusState1) { // Minus button state changed
        // updateState_MinusButton(minusButtonMode, menuObject);
        updateState_MinusButton(shiftButtonMode, menuObject);
        // The above three lines did not help a lot
        if(buttonMinusState1 == HIGH) {
          lastButtonMinusState1 = buttonMinusState1;        // save state for next loop
          return;  // Go to beginning of main loop() for fast response of button change
        //
        }
      }
      lastButtonMinusState1 = buttonMinusState1;        // save state for next loop
*/
      if (menuObject >= 0 && menuObject <= 11) {
        lcd_MenuText(menuObject);
        set_menuValue(menuObject);
      }
      else {
        Serial.printf("Invalid menu object [%d]\n", menuObject);
      }
      
    } // End of Config Menu
    else {  // Main function
      // if(nTP_flag == true || gPS_flag == true) {
      if(nTP_flag == true || gPS_flag == true || selfTime_flag == true) {
      
      #ifdef DEBUG
        Serial.printf("trackingObject: %d \n", trackingObject);
      #endif
      if (trackingObject == 0) {  // Sun tracking
        #ifdef FEATURE_SUN_TRACKING
          getSunPos();
//          display_objPos(sun_azim, sun_elev);
          lcd_objPos(sun_azim, sun_elev);
          // display_sunPos(sun_azim, sun_elev);
//          display_objText(trackingObject);
          lcd_objText(trackingObject);
          obj_azim = sun_azim;
          obj_elev = sun_elev;
        #endif
      }
      else if (trackingObject == 1) {  // Moon tracking
        #ifdef FEATURE_MOON_TRACKING
          getMoonPos();
//          display_objPos(moon_azimuth, moon_elevation);
          lcd_objPos(moon_azimuth, moon_elevation);
          // display_moonPos(moon_azimuth, moon_elevation);
//          display_objText(trackingObject);
          lcd_objText(trackingObject);
          obj_azim = (float)moon_azimuth;
          obj_elev = (float)moon_elevation;
        #endif
      }
      #ifdef FEATURE_SATELLITE_TRACKING
        else if (trackingObject >= 2 && trackingObject <= maxObj - 1) {  // Satellite tracking
          //sat.findsat(timeNow);
          SAT = trackingObject - 2;
          // sat.init(satname,TLE1[SAT],TLE2[SAT]);     //initialize satellite parameters
          sat.init(satelName[SAT],TLE1[SAT],TLE2[SAT]);     //initialize satellite parameters
          sat.findsat(timeNow);
          // upcomingPasses[SAT] = Predict(1);
          upcomingPasses[SAT] = Predict(1, &azimStart, &elevMax);
          #ifdef DEBUG
            invjday(sat.satJd , timeZone,true, year1, mon, day1, hr, min1, sec);
            Serial.println("\nLocal time: " + String(day1) + '/' + String(mon) + '/' + String(year1) + ' ' + String(hr) + ':' + String(min1) + ':' + String(sec));
            Serial.println("azimuth = " + String( sat.satAz) + " elevation = " + String(sat.satEl) + " distance = " + String(sat.satDist));
            Serial.println("latitude = " + String( sat.satLat) + " longitude = " + String( sat.satLon) + " altitude = " + String( sat.satAlt));
          #endif
          obj_azim = (float)(sat.satAz);
          obj_elev = (float)(sat.satEl);
          // if(int(sat.satEl) > 0) {    // if the satellite is above horizon
          if(int(obj_elev) > 0) {    // if the satellite is above horizon
            // display_objPos(float(sat.satAz), float(sat.satEl));
//            display_objPos(obj_azim, obj_elev);
//            display_objText(trackingObject);
            lcd_objPos(obj_azim, obj_elev);
            lcd_objText(trackingObject);
            #ifdef DEBUG
              Serial.print("sat obj_azim: ");
              Serial.println(String(obj_azim));
            #endif
          }
          else {    // satellite is below horizon, show when it will rise
            if (satelRise(upcomingPasses)) {
//              display_clock(nextRise[nextSat]);   // Print the current remaining time before rise
//              display_objText(trackingObject);
              // lcd_clock(nextRise[SAT]);   // Print the current remaining time before rise
              lcd_clock(nextRise[SAT], azimStart, elevMax);   // Print the current remaining time before rise
              lcd_objText(trackingObject);
            }
            else {
//              display_clock(0);
              lcd_clock(0, 0, 0);
            }
          }
          
        }
      #endif
      else if (trackingObject == maxObj) {    // Last object, the PARK position
        obj_azim = (float)az_park;
        obj_elev = (float)el_park;
//        display_objPos(obj_azim, obj_elev);
//        display_objText(trackingObject);
        lcd_objPos(obj_azim, obj_elev);
        lcd_objText(trackingObject);
        if (tracking_state == 0) {
          park_state[0] = 1;  // Enable azimuth to PARK
          park_state[1] = 1;  // Enable elevation to PARK
        } 
        else if ((park_state[0] == 0) && (park_state[1] == 0)) {
          tracking_state = 0;   // Disable auto tracking
        }
      }
      else {
        #ifdef DEBUG
          Serial.println(" Invalid tracking object. ");
        #endif
        tracking_state = 0; // Disable tracking
      }
      }   // End if(nTP_flag == true || gPS_flag == true) 
    }   // End Main function
  
    if(tracking_state != 0) {
      if (az_floatAngle > float(ANT_AZ_MIN) && az_floatAngle < float(ANT_AZ_MAX) && el_floatAngle > float(ANT_EL_MIN) && el_floatAngle < float(ANT_EL_MAX)) {
        func_track_object(obj_azim, obj_elev);
    //    if ((park_state[0] == 0) && (park_state[1] == 0)) {
    //      tracking_state = 0;   // Disable auto tracking
    //    }
      }
      else {
        #ifdef DEBUG
          Serial.println(" Error: ANT position out of limits! Disabling tracking... ");
        #endif
        tracking_state = 0;   // Disable auto tracking
        #if defined(ENABLE_AZIMUTH)
          if(moving_state[0] != 0) { // If azimuth Motor is already moving
            Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);
          }
        #endif
        #if defined(ENABLE_ELEVATION)
          if(moving_state[1] != 0) {  // If elevation motor is already moving
            Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);
          }
        #endif
      }
    }

    // Check if absolute encoder value changes while motor is moving.
    // If angle does not change and the timeout is reached then stop the motors (to avoid any damage in the system)
    #if defined(ENABLE_AZIMUTH)
      if(moving_state[0] != 0) {  // If azimuth motor is moving
        currentMovingTimeAz = millis();
      
        diffMovingTimeAz = currentMovingTimeAz - startMovingTime[0];
        #ifdef DEBUG
          Serial.printf("startMovingTime[0]: %d\n", startMovingTime[0]);
          Serial.printf("diffMovingTimeAz: %d\n", diffMovingTimeAz);
        #endif
      
        az_intAngle = round(az_floatAngle * 10);    // round(az_floatAngle * trackAccuFactor[trackAccuInd]);
        int diff_AzAngle = prev_intAzAngle - az_intAngle;
        #ifdef DEBUG
          Serial.printf("DIFF angle: %d\n", diff_AzAngle);
        #endif
        if (diff_AzAngle == 0) {
          if(diffMovingTimeAz > timeout_stop*1000) {
            #if defined(ENABLE_AZIMUTH)
              Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop AZ motor
              goto_state[0] = 0;
            #endif
            #if defined(ENABLE_ELEVATION)
              Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop EL motor
              goto_state[1] = 0;
            #endif
            tracking_state = 0;   // Disable auto tracking
            //#ifdef DEBUG
              Serial.println("ERROR: Check Az Motor or Az Encoder!!!");
            //#endif
            lcd_showError(0);
          }    
        }
        else {
          startMovingTime[0] = currentMovingTimeAz;
        }
        prev_intAzAngle = round(az_floatAngle * 10);      
      }
    #endif

    #if defined(ENABLE_ELEVATION)
      if(moving_state[1] != 0) {  // If elevation motor is moving
        currentMovingTimeEl = millis();
        
        diffMovingTimeEl = currentMovingTimeEl - startMovingTime[1];
        #ifdef DEBUG
          Serial.printf("startMovingTime[1]: %d\n", startMovingTime[1]);
          Serial.printf("diffMovingTimeEl: %d\n", diffMovingTimeEl);
        #endif
        
        el_intAngle = round(el_floatAngle * 10);
        int diff_ElAngle = prev_intElAngle - el_intAngle;
        #ifdef DEBUG
          Serial.printf("DIFF EL angle: %d\n", diff_ElAngle);
        #endif
        if (diff_ElAngle == 0) {
          if(diffMovingTimeEl > timeout_stop*1000) {
            #if defined(ENABLE_AZIMUTH)
              Motor_Soft_Stop(MOTOR_AZ1_PIN, MOTOR_AZ2_PIN, 0, pwmChanAz, &dutyCycleAz);  // Stop AZ motor
              goto_state[0] = 0;
            #endif
            #if defined(ENABLE_ELEVATION)
              Motor_Soft_Stop(MOTOR_EL1_PIN, MOTOR_EL2_PIN, 1, pwmChanEl, &dutyCycleEl);  // Stop EL motor
              goto_state[1] = 0;
            #endif
            tracking_state = 0;   // Disable auto tracking
            //#ifdef DEBUG
              Serial.println("ERROR: Check EL Motor or EL Encoder!!!");
            //#endif
            lcd_showError(1);
          }
        }
        else {
          startMovingTime[1] = currentMovingTimeEl;
        }
        prev_intElAngle = round(el_floatAngle * 10);      
      }
    #endif
//  }

//  else {
//    Serial.println(" Failed to get time fron either NTP server or GPS. ");
//  }
//  display_DISPLAY();
  
  delay(500); // wait for 1/2 second for no particular reason
}
