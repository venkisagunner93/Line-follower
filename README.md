# Line-follower
Arduino code for a line follower

This is a PID controller for an Arduino based line follower. The code was developed with the following hardware.

1. Arduino UNO microcontroller R3
2. QTRRC polulu IR line sensor
3. SainSmart buck converter
4. Turnigy Lipo 1800mAh 3S 20C battery
5. L298P Keyes motor driver shield
6. DF robot chassis

A modification was made to the QTRsensors library. The thresholding of values when the sensor sees all white is modified
in QTRSensors.cpp. Replace the QTRSensors.cpp in your local Arduino Libraries. Brief description of the library modification:

How it actually works:

If the last value from the line sensor is to the right of (num_sensors-1)*1000/2, the current sensor value will be the maximum
value. The sensor will output minimum value, if it sees otherwise.

How the mod works:


