// **** ---------------------- Features and Options, enable them below ------------------------------------------------
// Disable by commenting out
#define FEATURE_MOON_TRACKING
#define FEATURE_SUN_TRACKING
#define ENABLE_AZIMUTH
#define ENABLE_ELEVATION

#define DEFAULT_GRID "JO99AH"		// Maidenhead grid Stockholm
#define DEFAULT_LATITUDE 59.3293	// Latitude Stockholm
#define DEFAULT_LONGITUDE 18.0686	// Longitude Stockholm

#define TRACKOBJ_PIN 34   // GPIO-34 will be connected to button, to be used to change tracking object
#define LONGPRESS_TIME 4000

#define WIFI_SSID "Filips Wi-Fi Network"
#define WIFI_PASS "09011977filip"

#define NTP_SERVER "pool.ntp.org"
#define GMTOFFSET_SEC 0   // We want UTC time
#define DAYLIGHTOFFSET_SEC 0 

// OLED Params
// I2C SCL GPI22
// I2C SDA GPI21
#define OLED_RESET 0  // GPIO0
#define SCREEN_WIDTH 64 // OLED display width, in pixels
#define SCREEN_HEIGHT 48 // OLED display height, in pixels

// HH-12 absolute encoders connections params
// HSPI interface is used. ESP32 is master, HH-12 (for Az) and HH-12INC (for El) are slaves
#define hspi_clk_pin 14     // Common for both AZ and EL
#define az_hspi_cs_pin 15   // CS for AZ
#define el_hspi_cs_pin 5    // CS for El
#define hspi_miso_pin 12    // Common for both AZ and EL

// Az DC motor pins
#define MOTOR_AZ1_PIN 26  // Motor Az pins
#define MOTOR_AZ2_PIN 27
#define DIR_DELAY 500 // Brief delay

// El DC motor pins
#define MOTOR_EL1_PIN 33  // Motor El pins
#define MOTOR_EL2_PIN 25

// Setting PWM properties
#define PWM_FREQ  30000
#define PWM_CHAN1 0
#define PWM_CHAN2 1
#define PWM_RESOLUTION 8 // 8 bits
#define DUTY_CYCLE 70       // Max 255 for 8 bits resolution

// Ant PARK position
#define PARK_AZ 180
#define PARK_EL 89

// Antenna Azimuth and Elevetion LIMITS in degrees
#define ANT_AZ_MAX 290
#define ANT_AZ_MIN 90
#define ANT_EL_MAX 90
#define ANT_EL_MIN 7

#define TRACK_FREQ 30   // Set the tracking frequency in seconds (how often the motors will update position)

#define OPTION_REVERSE_AZ_HH12_AS5045
// #define OPTION_REVERSE_EL_HH12_AS5045
