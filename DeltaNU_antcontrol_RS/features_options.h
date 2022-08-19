// **** ---------------------- Features and Options, enable them below ------------------------------------------------
// Disable by commenting out
// #define DEBUG

#define FEATURE_MOON_TRACKING
#define FEATURE_SUN_TRACKING
#define FEATURE_SATELLITE_TRACKING
#define ENABLE_AZIMUTH
#define ENABLE_ELEVATION

#define DEFAULT_GRID "JO99AH"		// Maidenhead grid Stockholm
#define DEFAULT_LATITUDE 59.3293	// Latitude Stockholm
#define DEFAULT_LONGITUDE 18.0686	// Longitude Stockholm

#define TIMEZONE 1    // UTC+1

#define RXD2 16   // UART2 RX for GPS
#define TXD2 17   // UART2 TX for GPS

#define SHIFT_PIN 34   // GPIO-34 will be connected to "Shift" button, to be used to change tracking object
#define AUTO_PIN 35   // GPIO-35 will be connected to "Auto" button, to be used to start or stop the auto tracking
#define PLUS_PIN 36   // GPIO-36 will be connected to "Plus" button, to be used for changing values in menu
#define MINUS_PIN 39  // GPIO-39 will be connected to "Minus" button, to be used for changing values in menu
#define LONGPRESS_TIME 3000   // milliseconds
#define STARTPRESS_TIME 30  // milliseconds

#define COMMAND_BUFFER_SIZE 26

// #define WIFI_SSID "Filips Wi-Fi Network"
#define WIFI_SSID "FRITZ!Box 7590 UT"
#define WIFI_PASS "09011977filip"

#define NTP_SERVER "pool.ntp.org"
#define GMTOFFSET_SEC 0   // We want UTC time
#define DAYLIGHTOFFSET_SEC 0 

#define CELESTRAK_IP "104.168.149.178" //Web address to get TLE (CELESTRAK)
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

// RS-485 connection params (for receiving AZ and EL angles)
#define RXD1 18   // UART1 RX GPIO18
#define TXD1 19   // UART1 TX GPIO19
#define SerialTxRxControl 4  // UART1 TX-RX Control GPIO4

// Az DC motor pins
#define MOTOR_AZ1_PIN 26  // Motor Az pins. The DIR for MD13S H-bridge
#define MOTOR_AZ2_PIN 27  // the PWM for MD13S H-bridge
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
#define LOW_DUTY_CYCLE 130
#define MID_DUTY_CYCLE 190
#define HIGH_DUTY_CYCLE 255

// Ant PARK position
#define PARK_AZ 180
#define PARK_EL 89

// Antenna Azimuth and Elevetion LIMITS in degrees
#define ANT_AZ_MAX 270
#define ANT_AZ_MIN 90
#define ANT_EL_MAX 90
#define ANT_EL_MIN 7

#define TRACK_FREQ 30   // Set the default tracking frequency in seconds, can be updated via the MENU (how often the motors will update position)

#define TIMEOUT_STOP 5   // Set the timeout in seconds for stopping the motors if absolute encoder value does not change

#define TIMEOUT_RS485 1000   // Set the timeout in milliseconds for reading data from RS-485

#define ENCODER_FLAG 1    // 0 for angle data via local SPI connection, 1 for angle data via RS-485

#define ENCODER_AZ_BITS 1    // 0 for 12-bit absolute encoders (HH-12, AS5045), 1 for 14-bit encoders (AS5048A or B) via RS-485
#define ENCODER_EL_BITS 0    // 0 for 12-bit absolute encoders (HH-12, AS5045), 1 for 14-bit encoders (AS5048A or B) via RS-485

#define OPTION_REVERSE_AZ 0   // 1: Reverse
#define OPTION_REVERSE_EL 0

#define OPTION_REVERSE_AZ_HH12_AS5045
// #define OPTION_REVERSE_EL_HH12_AS5045
