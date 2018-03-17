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
 
#ifndef _GYRO_BNO055_COMP3_H_
#define _GYRO_BNO055_COMP3_H_

/*
 * VERSION 3 UPDATE
 * 
 * OK, tried a version with two BNO055's rotated 180 degrees, on the assumption
 * that some of the non-linearities would cancel out. Surprisingly enough, it made
 * things worse... A little fiddling showed that even the smallest misalignment 
 * between the IMUs causes nasty cross-axis errors...
 * 
 * That also showed that the dependencies in the rotation calculations where adding 
 * to the error, and these errors were then integrated, at which point things went
 * really pear shaped.
 * 
 * So I changed it to a quaternion based solution (which was a fun learning experience
 * on its own), so there we reduce transformations - especially ones sensitive to noise.
 * 
 * Final result is a lot simpler and more compact.  The gyros still lose sync 
 * occasionally, but results are a lot better.
 * 
 * The only problem is calculating the gravity quaternion - the method I use gives a 
 * random raw component, so the complementary filter ruins the gyro's yaw value. Not
 * an issue now, but I want to use this for a compass too.
 * 
 * Browsing for solutions, I found this:
 * 
 * http://www.mdpi.com/1424-8220/15/8/19302/pdf
 * 
 * And this solution is already half way there.  Will try using thos gravity conversions
 * next...
 * 
 * Also added a 1000s time constant high pass filter on the gyros, which has removed 
 * drift, and tuned the other filters.  Results are looking quite good so far!
 * 
 * OLD NOTES:
 * 
 * This file changes daily... Not sure it will ever be useful as-is...
 * 
 * Built in sesor fusion sucks
 * 
 * Was planning on doing Kalman filters, but I am too lazy to learn how to build/tune
 * a proper model.
 * 
 * Instead I will use a complementary filter.
 * 
 * THIS WAS THE INITIAL PLAN - BASIC PRINCIPLES ARE USED, BUT NEW DETAILS ARE LATER
 * 
 * Use BNO055 internal low pass filters to get rid of as much noise as possible.
 * Human conscious response time tops out at 20ms - so I would guess even PIO cannot
 * really exceed 5Hz - so set LPFs as close to this as possible.
 * 
 * Use a crude HPF filter tuned to around 0.01Hz - 100 seconds to remove DC bias
 * 
 * Tune complementary filter to sync to accel over similar period - but bias accel so 
 * that anything other than 1G total is largely rejected (if there is measurable 
 * acceleration, we are obviously not straight and level).
 * 
 * Aaarrggghhh
 * 
 * Sensor quality is really crap. The moment I pull Gs, the gyros go out of whack.
 * Try running Sensor Fusion to compensate the gyro/accel values, and then use
 * complementary filters to produce the actual output...
 * 
 * CURRENT DESIGN...
 * 
 * OK.  BNO055 has very good internal calibration and compensation algorithms
 * for the sensors - it is only the fusion algorithm that is really broken.
 * 
 * So, we use fusion mode for sensor calibration and compensation, then do our
 * own fusion.
 * 
 * Sensor output rate is 100Hz. It has internal 2nd order digital filters on the
 * sensors.  Beyond that, the documentation is really useless.  I would assume
 * the firmware was written by someone with a basic knowledge of digital systems,
 * so the filter cut-off should be lower than 50Hz. Hopefully at least one decade
 * lower, so I would _expect_ it would be ~5Hz, which is pretty much what I wanted 
 * anyway...
 * 
 * We sample the accelerometer and gyro at 100Hz.  The accelerometer we just average.
 * The gyro we integrate the rotation in Earth relative coordinates. Then use a 
 * complementary filter to join the two.
 * 
 * Timing is tight, so except for what must be done at 100Hz, we break everything
 * else up into steps.
 * 
 * TODO: This works quite well, and might even be working better than expected...
 * The display update takes too long, and we miss a big chunk of gyro readings.
 * Arduino libs are glorriously bad at multitasking or using interrupts, so maybe
 * try use this as a sample for distributing the code between two boards?
 * 
 */

// if device init failed, wait this time (ms) before trying again...
#define BNO055_TIME_REINIT 10000
// Poll device at 100Hz data rate in fusion mode
#define BNO055_TIME_POLL 10
// alpha for gyro averaging IIR filter
#define GYRO_DNO_ACCEL_ALPHA 3.0

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "module.h"
#include "quaternion.h"

class gyro_bno055: public module {
  public:
    gyro_bno055(const char * id, gyro_t& agyro_, gyro_t& gyro_, compass_t& compass_, slip_t& slip_, uint8_t addr_):
       module(id), agyro(agyro_), gyro(gyro_), compass(compass_), slip(slip_), bno(55, addr_) {};
     
    void setup(void) {
      gyro.upd = -1;
      compass.upd = -1;
      slip.upd = -1;
      
      if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF)) {
        dbgln(F("Ooops, no BNO055 detected"));
        delay(5000);
        state = 0;
        nextT = millis() + BNO055_TIME_REINIT;
        return;
      }
      
      state = 1;
      compass.value = 0;

      // test - use pin 9 to manually turn aiding on and off...
      pinMode(9, INPUT_PULLUP);

      // restore calibration from config memory
      restore_cal();
      bno.setExtCrystalUse(true);
      //gyroT = micros();
      nextT = millis() + 1000; // give it a second to settle...
    };
         
    void loop(unsigned long& ms) {
      uint8_t i;

      // loop until we catch up...
      // may end up duplicating samples, but duplicating a rotation is better than losing a sample period entirely
      // better not put too much in this loop, or it becomes infinite...
      while(millis() > nextT) {
        nextT += BNO055_TIME_POLL;

        // state 0 == wait for device init
        if(state == 0) {
          setup();
          return;
        }
      
        // always bump gyro
        // do this first so any subsequent state processing
        // is based on the latest numbers
  
        // get gyro vector
        V = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // grab gyros
  
        // delta T - always use 100Hz - as that is the rate at which the BNO is producing updated samples (even if we have some jitter when reading them)
        const float dt = BNO055_TIME_POLL/1000.0;

        // quaternian integration of angular velocity vector (roll all constant multiplications into one at the end)
        // https://stackoverflow.com/questions/46908345/integrate-angular-velocity-as-quaternion-rotation
        // this is a small deltaT approximation!
        qW = quaternion(V.x(), V.y(), V.z());  // * (M_PI/180);
        qGL.avg(qW, 100000.0);
        qW -= qGL;
        qG = qG + qG * qW * ((dt*M_PI)/360);  //(dt/2);
        qG.norm();
  
        // also grab and average accelerometers at high rate...
        V = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        #define BNO_ALPHA_ACCEL 20.0
        if(qT.Vmag < 0) {
          // not yet initialised?
          qT = quaternion(V.x(), V.y(), V.z());
          Amag = qT.mag();
        } else {
          qT.avg(quaternion(V.x(), V.y(), V.z()), BNO_ALPHA_ACCEL);
        }
        //qT.dump();

        // state 1 == get gravity magnitude and normalise vector
        if(state == 1) {
          // record average average gravity // gets qT magnitude, so this is cached in the object from here
          Amag = (Amag * GYRO_DNO_ACCEL_ALPHA + qT.mag()) / (GYRO_DNO_ACCEL_ALPHA + 1.0);
          // convenient 10ms dump
          qGL.dump();
        }
  
        // state 2 == get Acceleration quaternion
        if(state == 2) {
          // produce a quaternion describing the rotation from vertical to the average accelerometer value
          //https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
          // Half Way quaternion solution - hard coded for rotation to vertical
          qA[0] = qT[3] + qT.Vmag;
          qA[1] = qT[2];
          qA[2] = -qT[1];
          qA[3] = 0;
          qA.norm();
          //qA.dump();
          if(first_run) {
            memcpy(qG.V, qA.V, sizeof(qG.V));
            first_run = false;
          }
        }
  
        // calculate acceleration roll/pitch
        if(state == 3) {
          //pitch
          agyro.pitch = qA.pitch();
          dbg(agyro.pitch); dbgln(" apitch");
        }
        if(state == 4) {
          //roll
          agyro.roll = -qA.roll();
          dbg(agyro.roll); dbgln(" aroll");
        }
  
        // calculate gyro roll/pitch
        if(state == 5) {
          //pitch
          gyro.pitch = qG.pitch();
          dbg(gyro.pitch); dbgln(" gpitch");
        }
        if(state == 6) {
          //roll
          gyro.roll = -qG.roll();
          dbg(gyro.roll); dbgln(" groll");
        }
        
        // complement
        if(digitalRead(9) && state == 7) {
          float comp_alpha = abs(Amag - 9.8);
          comp_alpha *= 10.0;
          comp_alpha *= comp_alpha * comp_alpha * comp_alpha;
          comp_alpha += 100;

          dbg("alpha: "); dbgln(comp_alpha);
          qA.dump();
          qG.dump();

          // qA can negate (which is still the same quaternion)
          // try a very coarse negater...
          if((qA[0] > 0 && qG[0] < 0) || (qA[0] < 0 && qG[0] > 0)) {
            qG *= -1;
          }
  
          // just pull the vector components towards each other
          uint8_t i;
          for(i = 0; i < 4; i++)
            qG[i] = (qG[i] * comp_alpha + qA[i]) / (comp_alpha + 1.0);
          qG.norm();
  
          //compass.value = Amag;//comp_alpha;
          //slip.value = mgy;
        }
  
        // run the calibration on the next off phase
        if(state == 8) {
          uint8_t s1, s2, s3, s4;
          int foo;
          // fixme - for testing save first good cal after reboot
          bno.getCalibration(&s1, &s2, &s3, &s4);
          if(s1 & s2 & s3 & s4 == 3) {
            // fully calibrated
            bump(gyro.upd);
            bump(compass.upd);
            bump(slip.upd);
            // test
            //save_cal();
            //while(1) {}
          } else {
            // fixme - cant calibrate accelerometer in situ - need to alert this
            bump_bad(gyro.upd);
            bump_bad(compass.upd);
            bump_bad(slip.upd);
          }
          // test - fake calibration status into slip vector
          foo = s1;
          foo *= 10;
          foo += s2;
          foo *= 10;
          foo += s3;
          foo *= 10;
          foo += s4;
          slip.value = foo;
          dbg("cal: "); dbgln(foo);
        }
        
        state++;
        if(state >= 11) state = 1;

        // test
#if 1
        Serial.print("millis: ");
        Serial.print(state);
        Serial.print("-");
        Serial.println(millis()-ms);
#endif
      }
    };

  private:
    
    void restore_cal(void) {
      uint8_t caldata[NUM_BNO055_OFFSET_REGISTERS];
      
      CFG.fetch(id, NUM_BNO055_OFFSET_REGISTERS);
      CFG.read(0, caldata, NUM_BNO055_OFFSET_REGISTERS);
      if(!CFG.uninit) {
        // have valid cal data
        bno.setSensorOffsets(caldata);
        //dbgln(F("GOTCALDATA"));
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
    gyro_t& agyro;
    gyro_t& gyro;
    compass_t& compass;
    slip_t& slip;
    uint8_t state;
    bool first_run = true;
    unsigned long nextT;
    //unsigned long gyroT

    imu::Vector<3> V;
    quaternion qA;
    float Amag;
    quaternion qG;
    quaternion qGL; // extremely long term average
    quaternion qW;
    quaternion qT;
};

#endif

