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
 *  Interface to BMP180 and extract pressue
 *  
 *  NB!!! Too much pressure drift to reliably use for airspeed!!!
 */
#ifndef _PRESSURE_BMP180_H_
#define _PRESSURE_BMP180_H_

// interval to attempt reinit if device not detected
#define TIME_REINIT 30000L
// interval between temperature measurements
#define TIME_TEMP 10000L
// interval between pressure measurements
#define TIME_PRESSURE 10L
// samples to average over
#define AVG_SAMPLES 10.0


#include <SFE_BMP180.h>
#include <SoftwareWire.h>

#include "debug.h"
#include "module.h"

class pressure_bmp180: public module
{
  public:
    pressure_bmp180(const char * id, pressure_t& pi): module(id), p(pi) {
      dev = SFE_BMP180();
    }
    pressure_bmp180(const char * id, pressure_t& pi, SoftwareWire& w): module(id), p(pi) {
      dev = SFE_BMP180(w);
    }
  
    void setup(void) {
      if(!dev.begin()) {
        dbgln(F("Pressure init fail (disconnected?)\n\n"));
        set_uninit();
        return;
      }
      p.upd = -1;
      state = 0;
      nextT = millis(); // schedule next temp reading immediately
      nextP = nextT; // schedule next pressure reading immediately
      p.value = 0; // mark pressure as not yet set...
    }

    void loop(unsigned long& ms) {
      // not initialised?
      if(state < 0) {
        if(ms >= nextT) {
          // reinit counter expired
          dbgln(F("reinit..."));
          setup();
        }
        return;
      }
      // running...
      // temperature...
      if(state == 0 && ms >= nextT) {
        // time for a temperature reading
        state = dev.startTemperature();
        if(state <= 0) {
          dbgln(F("error starting temp reading..."));
          set_uninit();
          return;
        }
        nextT = millis() + state + 1;
        state = 1;
        return;
      }
      if(state == 1 && ms >= nextT) {
        state = dev.getTemperature(T);
        if(state <= 0) {
          dbgln(F("erore retrieving temp reading..."));
          set_uninit();
          return;
        }
        dbg(F("temp: "));
        dbgln(T);
        nextT = millis() + TIME_TEMP;
        state = 0;
        return;
      }
      // pressure...
      if(state == 0 && ms >= nextP) {
        // time for a pressure reading
        state = dev.startPressure(3);
        if(state <= 0) {
          dbgln(F("error starting pressure reading..."));
          set_uninit();
          return;
        }
        nextP = millis() + state + 1;
        state = 2;
        return;
      }
      if(state == 2 && ms >= nextP) {
        double P;
        state = dev.getPressure(P,T);
        if(state <= 0) {
          dbgln(F("erore retrieving pressure reading..."));
          set_uninit();
          return;
        }
        if(p.value == 0) {
          p.value = P;
        } else {
          p.value *= AVG_SAMPLES;
          p.value += P;
          p.value /= AVG_SAMPLES + 1.0;
        }
        dbg(id);
        dbg(" ");
        dbgln(p.value);
        p.ts = millis();
        bump(p.upd);
        nextP = p.ts + TIME_PRESSURE;
        state = 0;
        return;
      }

    }
    
  private:
    void set_uninit(void) {
      p.upd = -1;
      state = -1;
      nextT = millis() + TIME_REINIT; // used for reinit counter if status<0
    }

    pressure_t& p;
    SFE_BMP180 dev;
    int8_t state; // -1 uninit, 0 = init, 1 = temp requested
    double T; // temperature reading
    unsigned long nextT; // timestamp for next temperature reading - will also be used for reinit timer
    unsigned long nextP; // tmpestamp for next pressure reading
};

#endif

