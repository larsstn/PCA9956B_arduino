# PCA9956B Arduino Driver Library

Library to interface PCA9956B 24-channel LED driver with Arduino etc.

The code is based on Adafruit-PWM-Servo-Driver-Library for PCA 9685. (https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)

I just modified some register addresses and functions.

The code was successfully tested on an ESP32, but should work on any Arduino board.

For further information refer to the PCA9956B data sheet: https://www.nxp.com/docs/en/data-sheet/PCA9956B.pdf
or feel free to email me (github@larss.de)

Adafruit-PWM-Servo-Driver-Library copyright information:

> This is a library for our Adafruit 16-channel PWM & Servo driver, shield or FeatherWing
> 
> Pick one up today in the adafruit shop!
>   * https://www.adafruit.com/products/815
>   * https://www.adafruit.com/product/1411
>   * https://www.adafruit.com/product/2928
> 
> These drivers use I2C to communicate, 2 pins are required to interface.
> 
> Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!
> 
> Written by Limor Fried/Ladyada  for Adafruit Industries. BSD license, check license.txt for more information. 
> 
> All text above must be included in any redistribution 
