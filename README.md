# Asserv-2015
PID loop control of Werdroid 2015 for the Cup of Robotics / Eurobot

Contains PCB file (Fritzing format with gerber) and code for Teensy (Arduino-like). Use two Maxon DEC 50-5 controllers for brushless motors.

This module communicate with serial over USB to get instructions and push informations about coders, pwm, etc. The protocol send and receive only one C-structure in each direction containing all information needed. A timer of 5ms synchronize coders, calculations and communications.

![](https://raw.githubusercontent.com/werdroid/Asserv-2015/master/pcb_asserv2015/pcb_asserv2015.png)

The Asservissement.cpp file comes from [Club INTech](https://github.com/ClubINTech/TechTheFruit/tree/master/Arduino/CarteAsservissement)
