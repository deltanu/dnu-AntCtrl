# dnu-AntCtrl
Antenna control using arduino type microcontroller board (ESP32) and absolute encoders (e.g. HH-12) for antenna position indicators.
Read antenna position from absolute encoders HH-12, one for Azimuth and another for Elevation position. Encoders are connected to ESP32 via SPI interface (HH-12s are two slaves, ESP32 is host).

Get accurate UTC time from NTP server (via WiFi connection) at boot and use internal ESP32 internal timer (temporary). Pending to connect RTC board and optional GPS board.
Retrieve the TLE data from 4 predefined satellites (via WiFi connection) at boot. The chosen satellites can be changed by entering the corresponding NORAD number under the menu. 
Calculate tracking objects (SUN, MOON and four satellites) position from QTH set in config file in coordinates or maidenhead grid format (optional use location from GPS).

Show the time, antenna Az and El position and tracking object position in OLED or LCD display (connected to ESP32 via I2C interface).

Send commands to move antenna in order to track an object (sun, moon or satellite).

Use buttons to enter menu and configure "Az offset", "El offset", "Tracking Freq", "Antenna speed", "Track accuracy", "Grid locator", NORAD No of 4 satellites.

Send antenna posision through RS-485 connection, to cover long cable distance.

Configure UART to connect to PC.
