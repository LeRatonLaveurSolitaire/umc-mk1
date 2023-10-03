# Universal Motor Controller mk1 (UMC-mk1)
This is the repository of a universal motor controller (like washing machine motor) based on a arduino nano, a encoder for speed selection, I2C LCD screen for displaying data and a dimmer module for command. A PID loop enable a precise control of the motor. The speed is mesured from the tachometer and filterd by a LM358P.

# Pinout

  A4 -> SDA
  A5 -> SCL
  D2 -> Zero Crossing Detection
  D3 -> motor_speed
  D4 -> fire_pin
  D5 -> Encoder
  D6 -> Encoder
  D7 -> Encoder
  D8 -> GND
  D9 -> GND

# Material

* 1 * Arduino nano
* 4 * 10K resistor (0805)
* 1 * 22K resistor (0805)
* 1 * 10nF condensator (0805)
* 1 * 100nF condensator (THT format)
* 2 * screw terminal connector
* 1 * LM328p
* 1 * 8 pins chip holder
* 2 * fuse holder
* 1 * board
* 1 * Dimmer module with cables
* 1 * power cable
* 1 * universal motor with cables
* 1 * I2C LCD screen with cables
* Dupont connectors

# Speed detection

The motor speed is detected with the motor tachometer trimed between 0 and 5V by the LM358p and captured using the Timer 1 and interuptions.

# PID

The PID variables can be changed at the beggening of the scetch in order to obtain the wanted behavior

# Fire control

The thriac activation is controled by the Timer 2 with the OCR2A as control variable 


