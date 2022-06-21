# Air-mouse-accelerometer-gyroscope
Air mouse using STM32F407G-DISC1 and STEVAL-MKI197V1, which has a LSM6DSOX module and is used for its accelerometer and gyroscope.

# Physical connections between STM32F407G-DISC1 and STEVAL-MKI197V1:

STEVAL-MKI197V1 pins:      |       STM32F407G-DISC1 pins:
 -GND                      to       GND
 -both SDx                 to       GND
 -both SCx                 to       GND
 -VDD                      to       3V
 -VDDIO                    to       3V
 -CS                       to       PE2
 -SCL                      to       PB3
 -SDA                      to       PB5
 -SDO                      to       PB4
 
