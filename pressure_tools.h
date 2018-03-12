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
 
#ifndef _PRESSURE_TOOLS_H_
#define _PRESSURE_TOOLS_H_

// a selection of basic tools and modules for pressure instruments.

class altitude: public module {
  public:
    altitude(const char * id, pressure_t& p, qnh_t & q, altitude_t& a): module(id), pressure(p), qnh(q), alt(a) {};
    void setup(void) {
      alt.upd = 0;
      alt.ts = 0;
    };
    void loop(unsigned long& ms) {
      double d;

      // no maths, if not updated...
      //dbg(".");
      if(pressure.upd < 0 || alt.upd == pressure.upd) return;
      
      d = pressure.value;
      d /= qnh.value;
      d = pow(d, 1.0/5.255);
      d = 1-d;
      alt.value = d * 44330.0 * 3.28084;
      alt.upd = pressure.upd;
      alt.ts = pressure.ts;
      dbgln(alt.upd);
      dbgln(alt.ts);
      
    };
  private:
    pressure_t& pressure;
    qnh_t & qnh;
    altitude_t& alt;
};

// altitude RMS noise from datasheet
// FIXME - CONFIGURE FROM SENSOR!
// multiply by 4 to get outer absolute range from RMS...
#define ALT_NOISE 0.5*4.0

class vsi: public module {
  public:
    vsi(const char * id, altitude_t& a, vsi_t& v): module(id), alt(a), vsii(v) {};
    void setup(void) {
      vsii.upd = 0;
      vsii.value = 0;
      last_alt = 0;
      last_ts = 0;
      //dbgln(F("VSI SETUP"));
      //dbgln(last_alt);
    };
    void loop(unsigned long& ms) {
      if(alt.upd < 0) {
        vsii.upd = -1;
        return;
      }
      if(vsii.upd == alt.upd) return; // no update yet - dont run
      if(abs(alt.value - last_alt) > ALT_NOISE || alt.ts - last_ts >1000) {
        if(last_alt == 0) last_alt = alt.value;
        float d = alt.value - last_alt;
        if(abs(d) < ALT_NOISE) {
          vsii.value = 0.0;
        } else {
          d *= 60.0*1000.0; // 60 seconds - 1 minute
          d /= alt.ts - last_ts;
          vsii.value *= 9;
          vsii.value += d;
          vsii.value /= 10;
          vsii.value = round(vsii.value);
        }
        last_alt = alt.value;
        last_ts = alt.ts;
      }
      vsii.upd = alt.upd;
    };
  private:
    altitude_t& alt;
    vsi_t& vsii;
    float last_alt;
    long last_ts;
};

class asi: public module {
  public:
    asi(const char * id, pressure_t& pp, pressure_t& ps, asi_t& a): module(id), pitot(pp), stat(ps), asii(a) {};
    void setup(void) {
      asii.upd = 0;
      asii.value = 0;
      CFG.fetch(id, sizeof(cal));
      CFG.read(0, &cal, sizeof(cal));
    };
    void loop(unsigned long& ms) {
      int8_t t;
      // test
      //if(ms > 30000 && ms < 31000) set_zero();
      if(pitot.upd < 0 || stat.upd < 0) {// do not run unless both are valid
        asii.upd = -1;
        return;
      }
      t = pitot.upd + stat.upd;
      if(t < 0) t = 0;
      if(asii.upd == t) return; // no update yet - dont run
      // http://www.eaa1000.av.org/technicl/instcal/instcal.htm
      const float rho=0.0023769;
      const float psl=2116.22;
      const float f2k=3600.0/6076.0;
      float d = pitot.value - stat.value - cal; // fixme - cal
      float s = d > 0 ? 1.0 : -1.0;
      d *= s;
      d /= psl;
      d += 1.0;
      d = pow(d, 2.0/7.0);
      d -= 1.0;
      d *= 7.0 * psl;
      d /= rho;
      d = sqrt(d);
      d *= f2k;
      d *= s;
      d = round(d);
      asii.value *= 9.0;
      asii.value += d;
      asii.value /= 10.0; 
      asii.upd = t;
    };
  private:
    void set_zero(void) {
      cal = pitot.value - stat.value;
      CFG.fetch(id, sizeof(cal));
      CFG.write(0, &cal, sizeof(cal));
    };

    pressure_t& pitot;
    pressure_t& stat;
    asi_t& asii;
    float cal;
};

#endif

