#ifndef _GYRO_BNO055_H_
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
 
#define _GYRO_BNO055_H_

/*
 * This is the old driver that uses BNO055 internal SensorFusion algorithm - do not
 * use - it sucks...
 * 
 * Not doing any further work/cleanup on this module - leaving it here for reference.
 * 
 */

#define BNO055_TIME_REINIT 10000
#define BNO055_TIME_POLL 100

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "module.h"

class gyro_bno055: public module {
  public:
    gyro_bno055(const char * id, gyro_t& gyro_, compass_t& compass_, slip_t& slip_, uint8_t addr_):
       module(id), gyro(gyro_), compass(compass_), slip(slip_), bno(55, addr_) {};
     
     void setup(void) {
       gyro.upd = -1;
       compass.upd = -1;
       slip.upd = -1;
       
       if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF)) {
         dbgln(F("Ooops, no BNO055 detected"));
         state = 0;
         nextT = millis() + BNO055_TIME_REINIT;
         return;
       }
       
       state = 1;
       restore_cal();
       
       bno.setExtCrystalUse(true);
       nextT = millis() + BNO055_TIME_POLL;
     };
     
     void loop(unsigned long& ms) {
       if(ms < nextT) return;
       if(state == 0) {
         setup();
         return;
       }
       imu::Quaternion q = bno.getQuat();
       imu::Vector<3> euler = q.toEuler();
       gyro.pitch = euler.y() * 180 / M_PI;
       gyro.roll = -euler.z() * 180 / M_PI;

       compass.value = -90 - euler.x() * 180 / M_PI;
       if(compass.value < 0) compass.value += 360;
       
       // fake slip as calibration status
       uint8_t s1, s2, s3, s4;
       int foo;
       // fixme - for testing save first good cal after reboot
       bno.getCalibration(&s1, &s2, &s3, &s4);
       if(s1 == 3 && s2 == 3 && s3 == 3 && s4 == 3) {
        // fully calibrated
         bump(gyro.upd);
         bump(compass.upd);
         bump(slip.upd);
         // test
         if(state == 2) {
           // first good cal after recal after boot...
           state = 3;
           save_cal();
         }
         //while(1) {};
       } else {
         // fixme - cant calibrate accelerometer in situ - need to alert this
         if(s1 == 0 || s2 == 0 || s3 == 0 || s4 == 0) {
           // one calibration completely lost
           if(state == 1) {
             // no recal after boot
             state = 2;
           }
         }
         bump_bad(gyro.upd);
         bump_bad(compass.upd);
         bump_bad(slip.upd);
       }
       foo = s1;
       foo *= 10;
       foo += s2;
       foo *= 10;
       foo += s3;
       foo *= 10;
       foo += s4;
       slip.value = foo;

       nextT = ms + BNO055_TIME_POLL;
     };
  private:
    void restore_cal(void) {
      uint8_t caldata[NUM_BNO055_OFFSET_REGISTERS];
      
      CFG.fetch(id, NUM_BNO055_OFFSET_REGISTERS);
      CFG.read(0, caldata, NUM_BNO055_OFFSET_REGISTERS);
      if(!CFG.uninit) {
        // have valid cal data
        bno.setSensorOffsets(caldata);
        dbgln(F("GOTCALDATA"));
        //delay(10000);
      }
   };
  
    void save_cal(void) {
      uint8_t caldata[NUM_BNO055_OFFSET_REGISTERS];
      uint8_t i;
      
      CFG.fetch(id, NUM_BNO055_OFFSET_REGISTERS);
      bno.getSensorOffsets(caldata);
      CFG.write(0, caldata, NUM_BNO055_OFFSET_REGISTERS);
      //dbgln(F("SAVECALDATA"));
      //delay(10000);
   };
  
    Adafruit_BNO055 bno;
    gyro_t& gyro;
    compass_t& compass;
    slip_t& slip;
    uint8_t state;
    unsigned long nextT;
};

#endif

