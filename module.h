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
 * This is actually the core of the whole thing.  Simple modules which contain
 * basic Arduino setup and loop functions.
 * 
 * Just add id storage and some basic helpers...
 */
 
#ifndef _MODULE_H_
#define _MODULE_H_

#include <Arduino.h>

class module
{
  public:
    module(const char * c) {
      int i;
      for(i = 0; i < 3  && c && c[i]; i++) id[i] = c[i]; 
      for(; i < 3; i++) id[i] = 0;
      id[3] = 0;
      //dbgln(id); 
    };
    virtual void setup(void) = 0; // call from setup
    virtual void loop(unsigned long& ms) = 0; // share ms counter from main loop

    char id[4]; // add extra 0 byte so we can make it printable
    static bool id_match(const char * id1, const char * id2) {
      uint8_t i;
      if(!id1 || ! id2) return 0;
      for(i = 0; i < 3; i++) {
        if(id1[i] == 0 && id2[i] == 0) return 1; // end of both strings, and we haven mismatched yet
        if(id1[i] != id2[i]) return 0; // mismatch
      }
      return 1;
    };
  
  protected:
    static void bump(int8_t & u) {
      u++;
      if(u < 0) u = 0;
    }
    static void bump_bad(int8_t & u) {
      u--;
      if(u >= 0) u = -1;
    }
};

// data interchange structures for modules
// basic pressure structure
typedef struct {
    int8_t upd; // track status - increments with each update, cycling 0->127. negative on failure
    float value; // pressure in hPa
    unsigned long ts; // timestamp of last update
} pressure_t;

typedef struct {
    int8_t upd; // track status - tracks source pressure status
    float value; // altitude in feet
    unsigned long ts; // timestamp of last update
} altitude_t;

// basic qnh structure
typedef struct {
    float value; // pressure in hPa
} qnh_t;

typedef struct {
    int8_t upd; // track status - tracks source pressure status
    float value; // vs in feet/minute
} vsi_t;

typedef struct {
    int8_t upd; // track status - tracks source pressure status
    float pitch; // pitch in radians 0 = level, positive = up
    float roll; // roll in radians 0 = level, positive = right
    //float yaw; // yaw in radians 0 is indeterminate
    //float yaw_rate; // short term average yaw rate in radians per second
} gyro_t;

typedef struct {
    int8_t upd; // track status - tracks source pressure status
    float value; // heading in degrees
} compass_t;

typedef struct {
    int8_t upd; // track status - tracks source pressure status
    float value; // slip in degrees, positive = right
} slip_t;


typedef struct {
    int8_t upd; // track status - tracks source pressure status
    float value; // asi in kts
} asi_t;

#endif

