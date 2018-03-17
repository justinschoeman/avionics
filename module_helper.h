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
 * Basic helper functions which will be used by the main sketch to setup and run the
 * modules.
 * 
 * Defined in class context just to neatly package in a isolated name space.
 * 
 * Must be included after displays[] modules[] and display_modules[] are defined...
 */
 
#ifndef _MODULE_HELPER_H_
#define _MODULE_HELPER_H_

// main sketch must define these
extern display * displays[];
extern display_module * display_modules[];
extern module * modules[];

class ModuleHelper {
  public:
    // basic sanity checks - for now, just check that IDs are unique...
    static void init(void) {
      uint8_t i;
      for(i = 0; modules[i]; i++)
        check_id(modules[i]->id);
      for(i = 0; displays[i]; i++)
        check_id(displays[i]->id);
      for(i = 0; display_modules[i]; i++)
        check_id(display_modules[i]->id);
    };

    // run setup methods for each display
    static void setup_displays(void) {
      uint8_t i;
      for(i = 0; displays[i]; i++)
        displays[i]->setup();
    };

    // run setup method for the rest of the modules
    // (may need to do displays first, if they need custom setups...
    static void setup_modules(void) {
      uint8_t i;
      for(i = 0; modules[i]; i++)
        modules[i]->setup();
      for(i = 0; display_modules[i]; i++)
        display_modules[i]->setup();
    };

    static uint8_t dc;
    static void run_modules(void) {
      uint8_t i;
      unsigned long ms;

      // run all modules
      ms = millis();
      //dbg("xxx "); dbgln(ms);
      for(i = 0; modules[i]; i++)
        modules[i]->loop(ms);

      // run one display per tick
      displays[dc]->loop(ms);
      if(!displays[++dc]) dc = 0;

#if 0
      if(millis() > ms) {
        //Serial.print(" ");
        Serial.println(millis() - ms);
      }
#endif
    };
  
  private:
    static void check_id(const char * id) {
      uint8_t c = count_id(id);
      if(c == 0 || c > 1) {
        // FIXME - assert
        Serial.print(F("Invalid ID: "));
        Serial.print(id);
        Serial.print(" ");
        Serial.println(c);
        while(1) {}
      }
    };
  
    static uint8_t count_id(const char * id) {
      uint8_t i;
      uint8_t c;
      c = 0;
      for(i = 0; modules[i]; i++)
        c += module::id_match(modules[i]->id, id);
      for(i = 0; displays[i]; i++)
        c += module::id_match(displays[i]->id, id);
      for(i = 0; display_modules[i]; i++)
        c += module::id_match(display_modules[i]->id, id);
      return c; 
    };
};

uint8_t ModuleHelper::dc;

#endif

