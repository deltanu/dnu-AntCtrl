# dnu-AntCtrl
Antenna control using arduino type microcontroller board (ESP32), absolute encoders (e.g. HH-12) for antenna position indicators and H-bridges for controlling Azimuth and Elevation DC motors. An LCD 4x20 is used to display information like time, Antenna position Azimuth and Elevation (in degrees) and object to track (sun, moon, 4 satellites and park position). 4 buttons are used for operation and menu configuration.
Read antenna position from absolute encoders HH-12, one for Azimuth and another for Elevation position. Encoders are connected to ESP32 via SPI interface (HH-12s are two slaves, ESP32 is master).
Get accurate UTC time from NTP server (via WiFi connection) at boot and use internal ESP32 internal timer. If WiFi is not available then a GPS board (optional) like the neo6M (connected via UART interface) can be used for fetching the time. If neither WiFi nor GPS are available then the time can be set manual through the config menu or a command via serial interface.
Retrieve the TLE data from 4 predefined satellites (via WiFi connection) at boot. The chosen satellites can be changed by entering the corresponding NORAD number under the config menu. 
Calculate tracking objects (SUN, MOON and four satellites) position from QTH set in config file in coordinates or maidenhead grid format (optional use location from GPS).
Show the time, antenna Az and El position and tracking object position in OLED or LCD display (connected to ESP32 via I2C interface). If showing a satellite which is not over the horizon then show the rise time and the starting Azimuth when "visible". 
Use buttons to move antenna in order to track an object (sun, moon or satellite).
Use buttons to enter menu and configure "Az offset", "El offset", "Tracking Freq", "Antenna speed", "Track accuracy", "Grid locator", NORAD No of 4 satellites, "Set Time manually". The values of these parameters (except the "Time Set Manually") are stored in the internal EEPROM.
Configure UART to connect to PC. Accept Easycomm I protocol commands to move the antenna. 
Send antenna posision through RS-485 connection, to cover long cable distance (Pending).
