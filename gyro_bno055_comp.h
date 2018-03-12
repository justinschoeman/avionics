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
 
#ifndef _GYRO_BNO055_COMP_H_
#define _GYRO_BNO055_COMP_H_

/*
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
      gyroT = micros();
      nextT = millis() + BNO055_TIME_POLL;
    };
         
    void loop(unsigned long& ms) {
      uint8_t i;

      // wait for out poll interval
      if(ms < nextT) return;
      
      // state 0 == wait for device init
      if(state == 0) {
        setup();
        return;
      }
      
      // set next run time up top, as we may need to go back to a state...
      nextT = ms + BNO055_TIME_POLL;

      // always bump gyro
      // do this first so any subsequent state processing
      // is based on the latest numbers

      // get gyro vector
      V = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // grab gyros

      // calculate delta T (consistently use timestamp immediately after read - should be representative)
      unsigned long gms = micros(); // and timestamp as soon as possible
      float dt = gms - gyroT; // calculate time delta
      dt /= 1000000.0;
      gyroT = gms;

      // translate the angular velocity vector to Earth reference and rotate the gyro vector
      
      // rotate x
      tmpf = -V.x() * dt * M_PI / 180;
      R[0][0] = 1;
      R[0][1] = 0;
      R[0][2] = 0;
      R[1][0] = 0;
      R[1][1] = cos(tmpf);
      R[1][2] = -sin(tmpf);
      R[2][0] = 0;
      R[2][1] = -R[1][2];
      R[2][2] = R[1][1];
      rotate();

      // rotate y
      tmpf = -V.y() * dt * M_PI / 180;
      R[0][0] = cos(tmpf);
      //R[0][1] = 0;
      R[0][2] = sin(tmpf);
      //R[1][0] = 0;
      R[1][1] = 1;
      R[1][2] = 0;
      R[2][0] =-R[0][2];
      R[2][1] = 0;
      R[2][2] = R[0][0];
      rotate();

      // rotate z
      tmpf = -V.z() * dt * M_PI / 180;
      R[0][0] = cos(tmpf);
      R[0][1] = -sin(tmpf);
      R[0][2] = 0;
      R[1][0] = -R[0][1];
      R[1][1] = R[0][0];
      //R[1][2] = 0;
      R[2][0] = 0;
      //R[2][1] = 0;
      R[2][2] = 1;
      rotate();

      //Serial.println("gyro: ");
      //for(i = 0; i < 3; i++) Serial.println(GV[i]);

      // track yaw
      // http://www.euclideanspace.com/physics/kinematics/angularvelocity/
      // Wz = yawrate * cos(roll) * cos(pitch) - pitchrate * sin(roll)
      tmpf = V.z()*M_PI/180 * cos(gyro.roll) * cos(gyro.pitch) - V.y()*M_PI/180 * sin(gyro.roll);
      //gyro.yaw_rate = (gyro.yaw_rate * 4 + tmpf) / 5;
      //gyro.yaw += tmpf * dt;
      //compass.value = (compass.value * 4 + tmpf*60) / 5;
      //compass.value += tmpf * dt;

      // also grab and average gyros at high rate...
      V = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      #define BNO_ALPHA_ACCEL 5.0
      AVT[0] = (AVT[0] * BNO_ALPHA_ACCEL + V.x()) / (BNO_ALPHA_ACCEL + 1);
      AVT[1] = (AVT[1] * BNO_ALPHA_ACCEL + V.y()) / (BNO_ALPHA_ACCEL + 1);
      AVT[2] = (AVT[2] * BNO_ALPHA_ACCEL + V.z()) / (BNO_ALPHA_ACCEL + 1);

      // average 1.3ms for the above set of rotations...
      //gms = micros() - gms;
      //Serial.print("micros: ");
      //Serial.println(gms);


      // state 1 == get Acceleration vector
      if(state == 1) {
        //V = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        //AV[0] = V.x();
        //AV[1] = V.y();
        //AV[2] = V.z();
        // snapshot the average for the following calculations
        memcpy(AV, AVT, sizeof(AV));
      }

      // state 2 == get gravity magnitude and normalise vector
      if(state == 2) {
        // get magnitude
        tmpf = 0;
        for(i = 0; i < 3; i++) tmpf += AV[i]*AV[i];
        tmpf = sqrt(tmpf);
        if(tmpf == 0 || isnan(tmpf) || isinf(tmpf) || tmpf > 100) {
          // bad read...
          state = 1;
          return;
        }
        // normalise
        for(i = 0; i < 3; i++)
          AV[i] /= tmpf;
        //AV[0] /= tmpf;
        //AV[1] /= tmpf;
        //AV[2] /= tmpf;
        if(first_run) {
          memcpy(GV, AV, sizeof(GV));
          first_run = false;
        }
        // record average average gravity
        Amag = (Amag * GYRO_DNO_ACCEL_ALPHA + tmpf) / (GYRO_DNO_ACCEL_ALPHA + 1.0);
      }

      // calculate acceleration roll/pitch
      if(state == 3) {
        //pitch
        // https://www.nxp.com/docs/en/application-note/AN3461.pdf
        agyro.pitch = -atan2(AV[0], sqrt(AV[2] * AV[2] + AV[1] * AV[1]));
      }
      if(state == 4) {
        //roll
        // https://www.nxp.com/docs/en/application-note/AN3461.pdf
        agyro.roll = atan2(-AV[1], (AV[2] < 0 ? -1.0 : 1.0) * sqrt(AV[2] * AV[2] + 0.01 * AV[0] * AV[0]));
      }

      // calculate gyro roll/pitch
      if(state == 5) {
        //pitch
        // https://www.nxp.com/docs/en/application-note/AN3461.pdf
        gyro.pitch = -atan2(GV[0], sqrt(GV[2] * GV[2] + GV[1] * GV[1]));
      }
      if(state == 6) {
        //roll
        // https://www.nxp.com/docs/en/application-note/AN3461.pdf
        gyro.roll = atan2(-GV[1], (GV[2] < 0 ? -1.0 : 1.0) * sqrt(GV[2] * GV[2] + 0.01 * GV[0] * GV[0]));
      }
      
      // complement
      if(digitalRead(9) && state == 7) {
        float comp_alpha = abs(Amag - 9.8);
        comp_alpha *= 10.0;
        comp_alpha *= comp_alpha * comp_alpha * comp_alpha;
        comp_alpha += 10;

        // just pull the vector components towards each other
        uint8_t i;
        for(i = 0; i < 3; i++)
          GV[i] = (GV[i] * comp_alpha + AV[i]) / (comp_alpha + 1.0);

        //compass.value = Amag;//comp_alpha;
        //slip.value = mgy;
      }

      // run the calibration on the next off phase
      if(state == 8) {
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
      }
      
      state++;
      if(state >= 11) state = 1;

      // test
#if 0
      Serial.print("millis: ");
      Serial.print(state);
      Serial.print("-");
      Serial.println(millis()-ms);
#endif
    };

  private:
    // simple helper
    // rotate gyro vector GV with rotation matrix R
    // GV'= R.GV
    void rotate(void) {
      float TV[3];
      uint8_t i;
      for(i = 0; i < 3; i++) {
        TV[i] = GV[0] * R[i][0];
        TV[i] += GV[1] * R[i][1];
        TV[i] += GV[2] * R[i][2];
      }
      //for(i = 0; i < 3; i++) GV[i] = TV[i];
      memcpy(GV, TV, sizeof(GV));
    }
    
    void restore_cal(void) {
      uint8_t caldata[NUM_BNO055_OFFSET_REGISTERS];
      uint8_t i;
      
      CFG.fetch(id, NUM_BNO055_OFFSET_REGISTERS);
      CFG.read(0, caldata, NUM_BNO055_OFFSET_REGISTERS);
      // FIXME this is silly - must really be a helper in config class
      for(i = 0; i < NUM_BNO055_OFFSET_REGISTERS; i++) {
        if(caldata[i] != 255) {
          i = 0;
          break;
        }
      }
      if(!i) {
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
    unsigned long gyroT;

    imu::Vector<3> V;
    float AVT[3] = { 0 };
    float AV[3] = { 0 };
    float Amag = 9.8; // acceleration magnitude m.s-2
    float GV[3] = { 0 };
    float tmpf;
    float R[3][3];
};

#endif

