
Introduction
============

(Vanilla) Python module for the NXP FXAS21002C gyroscope.

Intended for use with Beaglebone or similar platforms
which contain a standard CPython implementation and I2C
bus access through standard Linux OS facilities.

Based on the original Adafruit FXAS21002C C/C++ driver from:
https://github.com/adafruit/Adafruit_FXAS21002C

Dependencies
=============

This driver depends on nothing outside of a typical Python installation,
since it includes the minimal components of the following libraries
required for the lower-level I2C interfacing:

- Adafruit_GPIO (https://github.com/adafruit/Adafruit_Python_GPIO)
- Adafruit_PureIO (https://github.com/adafruit/Adafruit_Python_PureIO)


Usage Example
=============

See examples/simpletest.py for an example of the usage.
