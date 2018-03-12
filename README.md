# Simple, modular avionics for homebuilders #

This is the very start of the project... I am just comitting the basic
framework for now.

USE AT YOUR OWN RISK!

## Supported hardware ##

There are currently three supported hardware drivers:

Adafruit_PCD8544 (actually, any Adafruit_GFX display should work) for Nokia
5110 style displays. Currently only hardware SPI is supported - but this is
just the constructor that needs to be copied...

Adafruit_BNO055 for BNO055 IMU

SFE_BMP180 for pressure (you can figure out your same way to connect a pipe
to this sensor... some people place it in a small, sealed box... i took a 90
degree irrigation fitting, cut off one barb, heated the cut end until soft,
then squished it over sensor (careful to avoid sense hole), held in place
with cable tie, and fixed with epoxy)

## Connecting the hardware ##

There are plenty of howtos on the web for connecting each of these devices -
I will not repeat them here - just provide some basic advice.

5110LCD - this is a 3.3V device. VCC must be 3.3V. The backlight can be
connected to 5V through a resistor (I use 680 Ohm). The inputs are 5V
tolerant, but it is best to connect them through a 1k resistor, just to be
safe.

I2C devices.  All the I2C devices are 3.3V. There are many different
breakout boards, so you need to follow the instructions specific to the
board you acquire. Most have built in 3.3V regulators, and can be connected
to a 5V supply. Some have built in level shifters, and can safely be used
with 5V Arduinos.  Some do not have level shifters, but can be used on 5V
Arduinos with some care.  Since I2C is an open drain output, the HIGH
voltage is set through pull up resistors.  Every breakout board I have seen
has built in pull-up resistors to 3.3V. So the I2C device will see 0-3.3V
which is perfect. BUT the Arduino has a silly restricition on the I2C pins,
that it requires 0.7*VCC (or 3.5V to guarantee that it will read HIGH).
Ideally you should use a level shifter. But you can also reduce VCC to the
arduino by passing the 5V input through a 1N4001 diode (or similar).  When
powered from the USB port, it already has a series diode, so works fine. If
powering from an external 5V source, then add a diode so that VCC decreases
to around 4.4V.
