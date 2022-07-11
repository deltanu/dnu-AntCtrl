# dnu-AntCtrl
Antenna control system using arduino type microcontroller board (ESP32), absolute encoders (currently HH-12 and HH-12INC are suported) for antenna position indicators and H-bridges for controlling Azimuth and Elevation DC motors. An LCD 4x20 is used to display information like Date-Time (in UTC), Antenna position Azimuth and Elevation (in degrees) and object to track (sun, moon, 4 satellites and park position). 4 buttons are used for operation and menu configuration.

It reads antenna position from absolute encoders HH-12, one for Azimuth and another for Elevation position. Encoders are connected to ESP32 via SPI interface (HH-12s are two slaves, ESP32 is master).

It gets accurate UTC time from NTP server (via WiFi connection) at boot and then uses ESP32 internal timer. If WiFi is not available then a GPS board (optional) like the neo-6M (connected via UART interface) can be used for fetching the time. If neither WiFi nor GPS are available then the time can be set manually through the config menu or a command via serial interface.

It retrieves the TLE data from 4 predefined satellites (via WiFi connection) at boot. The chosen satellites can be changed by entering the corresponding NORAD number under the config menu. 

It calculates tracking objects (SUN, MOON and four satellites) position from QTH set in config file in coordinates or maidenhead grid format (optional use location from GPS). QTH maidenhead grid can be also set via the config menu. 

It shows the Date-Time, antenna Az and El position and tracking object position in LCD display (connected to ESP32 via I2C interface). If the chosen  satellite is not over the horizon then it shows the rise time and the starting Azimuth when "visible". 

It uses a button to start tracking an object (sun, moon or satellite) by automatically move the antenna via the connected H-Bridges.

It uses buttons to enter the config menu and configure "Az offset", "El offset", "Tracking Freq", "Antenna speed", "Tracking accuracy", "Grid locator", NORAD No of 4 satellites, "Time Set Manually". The values of these parameters (except the "Time Set Manually") are stored in the internal EEPROM.

It uses UART port to connect to PC. Accepts Easycomm I protocol commands to move the antenna. 

(Pending) It sends antenna posision through RS-485 connection, to cover long cable distance.
