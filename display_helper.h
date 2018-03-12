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
 
#ifndef _DISPLAY_HELPER_H_
#define _DISPLAY_HELPER_H_

/*
 * helpers for the various display modules...
 * make sure libraries are included before
 * including this file
 * 
 * TODO:
 * When using multiple displays, we will need to share the frame buffer - so we
 * need to add a flag to tell display modules when they need to do a complete 
 * redraw, and when they can just update the required areas...
 * 
 */

#include "module.h"

// default display update rate 10Hz
#define DISPLAY_INTERVAL 100L

/* 
 * main process defines an array of display modules
 * which will be rendered in display loop
 * 
 * A display module is just a module with an extra display attribute
 * (used to link the module to the display it should render on)
 * 
 */
class display;
class display_module: public module {
  public:
    display_module(const char * id, display& d_): module(id), d(d_) {};
    display& d;
};

extern display_module * display_modules[];


/* 
 *  
 * this is ugly shit...
 * 
 * Adafruit_GFX class does not have sufficient methods to make the PCD8544 class even display anything
 * so we can't abstract display types directly.
 * Introspection does not work on arduino, so can't cast back up.
 * I can't figure out how to get siblings to inherit a single base class copy (diamond problem)
 * So end result is a container object which holds the base class and abstract 
 * container object which contains the abstract view of it (with additional required methods)
 * 
 * display is the generalised display class, which must have all functions required to operate all displays
 * (note: only operate - instantiate and configure is done in the derived class)
 */
class display: public module {
  public:
    display(const char * id, Adafruit_GFX& g): module(id), gfx(g) {
      nextflush = millis();
    };
    // standard Arduino setup and loop, called from ModuleHelper
    virtual void setup(void) {}; // for module.h
    virtual void loop(unsigned long& ms) {
      // by default, only update display every 100ms
      if(ms >= nextflush) {
        uint8_t i;
        for(i = 0; display_modules[i]; i++) {
          if(&(display_modules[i]->d) == this) {
            display_modules[i]->loop(ms);
          }
        }
        flush();
        nextflush = ms + DISPLAY_INTERVAL;
      }
    };
    
    // these are our abstracted functions to make devices useful
    
    // flush framebuffer to display (actually display what we have been drawing)
    // default empty implementation, so only override if the display driver
    // requires such a function...
    virtual void flush(void) {};

    // clear display - default is basic blank rectangle - override if driver provides
    // a more efficient method
    virtual void clear(void) {gfx.fillRect(0, 0, gfx.width(), gfx.height(), 0);}; // default clear - write white to entire area
    Adafruit_GFX& gfx;
    unsigned long nextflush;
};

#ifdef _ADAFRUIT_PCD8544_H

/*
 * provide an abstraction of PCD8544 driver
 * add extra constructors as required.
 * 
 */
class display_pcd8544: public display {
  public:
    //display_pcd8544(const char * id, Adafruit_PCD8544& d): priv(d), display(id, d) {};
    display_pcd8544(const char * id, uint8_t dc, uint8_t cs, uint8_t rs): priv(dc, cs, rs), display(id, priv) {};
    void setup(void) {
      // setup display
      priv.begin();
      // FIXME - read from config
      // FIXME - add UI to adjust
      priv.setContrast(40);
      priv.clearDisplay();   // clears the screen and buffer
    }
    void flush(void) {priv.display();};
    void clear(void) {priv.clearDisplay();}
    Adafruit_PCD8544 priv;
};
#endif

/*
 * a display module to render text+number to display
 * 
 * FIXME: should not be in this file...
 */
class number_display: public display_module {
  public:
    number_display(display& d, float& p, const char * txt, int16_t x, int16_t y): display_module(txt, d), p(p), txt(txt), x(x), y(y){};
    void setup(void) {};
    void loop(unsigned long& ms) {
      d.gfx.fillRect(x, y, d.gfx.width(), 8, 0);
      d.gfx.setCursor(x, y);
      d.gfx.print(txt);
      d.gfx.print(" ");
      d.gfx.println(p);
    };

    float& p;
    const char * txt;
    int16_t x;
    int16_t y;
};



#endif
