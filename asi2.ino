/*
 * Copyright 2018 Justin Schoeman
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this 
 * software and associated documentation files (the "Software"), to deal in the Software 
 * without restriction, including without limitation the rights to use, copy, modify, 
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to 
 * permit persons to whom the Software is furnished to do so, subject to the following 
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * Basic sample insrument(s)...
 */
 
// Arduino builtins
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

// Other libraries required for this build
#include <SoftwareWire.h>
#include <SFE_BMP180.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// always include debug.h first
// to compile out debugging, turn off the _DEBUG define in debug.h
#include "debug.h"
// initialise the debugger - pass it any object that inherits Print
dbgi(Serial); // init debugger to Serial

// next, module.h, as everything after this uses modules
#include "module.h"

// shared config memory object
#include "config.h"
// always one global object
config CFG = config();

/*
 * There is a shared structure that stores all sensor and instrument outputs.
 * 
 * We use a structure like this, as the actual sensors/processors/displays
 * may not be on this device.  The idea is to add an RS485 protocol to exchange
 * these data structures between a number of boards...
 * 
 * Basic module class and shared data structures are defined in module.h .
 */
struct inst_s {
  // modules
  pressure_t p_pitot;
  pressure_t p_static;
  qnh_t qnh = {1023};
  // virtual instruments
  altitude_t alt;
  vsi_t vsi;
  asi_t asi;
  gyro_t gyro;
  compass_t compass;
  slip_t slip;
  gyro_t agyro;
} inst;

// set up busses
SoftwareWire W2 = SoftwareWire(7,8,false,true);

/*
 * Set up modules.  There are 3 arrays of modules:
 * 1) displays (actual display units)
 * 2) display modules (take shared instrument values 
 *     and render them to one of the above displays)
 * 3) modules (sensors, processors and communication)
 * 
 * TODO: UI
 * Basic plan is to add module methods to:
 * a) query list of UI widgets
 * b) method to apply a value to a widget
 * (Widgets will be identifiable by a const, so they
 * can be manually attached to input devices if required.)
 * 
 */

/*
 *  Display Modules
 *  
 *  Instantiate display object for each display, then
 *  place it in a NULL terminated array.
 *  
 *  They need to be instantiated into individual global
 *  objects, as we need a reference to them later for
 *  the display objects.  Not entirely sure why looking 
 *  up the object in this array fails for static 
 *  declarations.
 *  
 */
#include "display_helper.h"
// instantiate d0
display_pcd8544 d0 = display_pcd8544("d00", 5, 4, 3);
// set up array of all defined displays
display * displays[] = {
  &d0,
  NULL
};

/*
 * Modules
 * 
 * Set up each module inside a NULL terminated array.
 * 
 */
#include "pressure_bmp180.h"
#include "pressure_tools.h"
#include "gyro_bno055_comp.h"
#include "gyro_tools.h"
module * modules[] = {
  //new pressure_bmp180("Ppt", inst.p_pitot, W2),
  //new pressure_bmp180("Pst", inst.p_static),
  new gyro_bno055("GY0", inst.agyro, inst.gyro, inst.compass, inst.slip, BNO055_ADDRESS_B),
  //new altitude("Alt", inst.p_static, inst.qnh, inst.alt),
  //new vsi("VSI", inst.alt, inst.vsi),
  //new asi("ASI", inst.p_pitot, inst.p_static, inst.asi),
  NULL
};

/*
 * Display Modules
 * 
 * NULL terminated array of display modules.
 * 
 * Each display module is attached to one display.  The display
 * driver will call each display module in turn to render the 
 * display before updating.
 * 
 */
display_module * display_modules[] = {
  new number_display(*displays[0], inst.p_pitot.value, "PIT", 0, 0),
  new number_display(*displays[0], inst.p_static.value, "STA", 0, 8),
  new number_display(*displays[0], inst.qnh.value, "QNH", 0, 16),
  new number_display(*displays[0], inst.alt.value, "ALT", 0, 24),
  //new number_display(*displays[0], inst.gyro.pitch, "GP", 0, 0),
  //new number_display(*displays[0], inst.agyro.pitch, "AP", 0, 8),
  //new number_display(*displays[0], inst.gyro.roll, "GR", 0, 16),
  //new number_display(*displays[0], inst.agyro.roll, "AR", 0, 24),
  //new number_display(*displays[0], inst.compass.value, "CT", 0, 24),
  new number_display(*displays[0], inst.vsi.value, "VSI", 0, 32),
  new number_display(*displays[0], inst.asi.value, "ASI", 0, 40),
  new simple_ah(*displays[0], inst.gyro, "AH", 84-48, 0, 47, 47),
  new simple_ah(*displays[0], inst.agyro, "AHA", 0, 0, 47, 47),
  NULL  
};

/*
 * Traditional Arduino setup function - calls individual module
 * setup functions...
 */
#include "module_helper.h"
void setup()
{
  uint8_t i;

  // if we want serial debug, then init serial port first
  Serial.begin(115200);
  dbgln(F("REBOOT"));

  // initialise and check module system
  ModuleHelper::init();

  // setup all displays
  ModuleHelper::setup_displays();

  // custom setup per display
  d0.priv.setTextWrap(false);
  d0.priv.setRotation(2);

  // after setup, add debug display if required
  //dbgd(d0);

  // setup all modules (including display modules)
  ModuleHelper::setup_modules();
}

/*
 * Traditional Arduino loop
 */
void loop()
{
  // run modules
  ModuleHelper::run_modules();
}

