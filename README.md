# dnu-AntCtrl
Antenna control system using arduino type microcontroller board (ESP32), absolute encoders (HH-12, HH-12INC and AS5045 12-bit as well as AS5048A ans AS5048B 14-bits are suported) for antenna position indicators and H-bridges for controlling Azimuth and Elevation DC motors. An LCD 4x20 is used to display information like Date-Time (in UTC), Antenna position Azimuth and Elevation (in degrees) and object to track (sun, moon, 4 satellites and park position). 4 buttons are used for operation and menu configuration.

It reads antenna position from absolute encoders, one for Azimuth and another for Elevation position. Encoders are connected to ESP32 via SPI interface (HH-12s are two slaves, ESP32 is master) or to an external / separate arduino microcontroller which sends the angle data via RS-485 to the ESP32.

It gets accurate UTC time from NTP server (via WiFi connection) at boot and then uses ESP32 internal timer. If WiFi is not available then a GPS board (optional) like the neo-6M (connected via UART interface) can be used for fetching the time. If neither WiFi nor GPS are available then the time can be set manually through the config menu or a command via serial interface.

It can retrieve the TLE data from 4 predefined satellites via WiFi connection and stores the data to the internal SPIFFS filesystem. The TLE data are retrieved from the filesystem at next boot. The TLE data can be updated via WiFi by selecting the corresponding config menu. The chosen satellites can be changed by entering the corresponding NORAD number under the config menu. 

It calculates tracking objects (SUN, MOON and four satellites) position from QTH set in config file in coordinates or maidenhead grid format (optional use location from GPS). QTH maidenhead grid can be also set manually via the config menu. 

It shows the Date-Time, antenna Az and El position and tracking object position in LCD display (connected to ESP32 via I2C interface). If the chosen  satellite is not over the horizon then it shows the rise time and the starting Azimuth when "visible". 

It uses a button to start auto tracking an object (sun, moon or satellite) by automatically move the antenna via the connected H-Bridges.

It uses buttons to enter the config menu and configure "Az offset", "El offset", "Tracking Freq", "Antenna speed", "Tracking accuracy", "Grid locator", NORAD No of 4 satellites, "antenna error timeout", "Time Set Manually", "encoder flag" (option to choose local SPI or RS485 for retrieving angle data from the absolute encoders), "Encoder_az_bits" (12 or 14), "Encoder_el_bits" (12 or 14), "option_reverse_az", "option_reverse_el". The values of these parameters (except the "Time Set Manually") are stored in the internal EEPROM.

It uses UART port to connect to PC. Accepts Easycomm II protocol commands to move the antenna from an external software application.

It retrieves antenna posision from absolute encoders (connected to a separate microcontroller) through RS-485 connection, to cover long cable distance.
