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
 
#ifndef _CONFIG_H_
#define _CONFIG_H_

/*
 * This is used to store non-volatile configuration for all modules.
 * 
 * Each module has a 3 character id (which must be unique) - the modules
 * use this id to query the config store to get their own config storage.
 * 
 * There is ONE instance of the config object, and it is SHARED by modules.
 * If a module returns from a setup or loop function, then it must call
 * 'fetch' again before read or write.
 * 
 * Use 'fetch' to get the config storage for our id.
 * 'read' and 'write' to read or write data in the modules storage space.
 * Newly allocated data is all FFs - it is up to the module to use this fact
 * to identify uninitialised config).
 * 
 * For basic Arduinos, this is implemented as a very simple filesystem type
 * thingy on top of the EEPROM.  Should add extra drivers for SD card or
 * IIC connected EEPROM, etc.
 * 
 * FIXME: Storage length cannot change once allocated - need to implement
 * a way to change this...
 * 
 * FIXME: Need a way to reset all storage.
 * 
 * FIMXE: Need to abstract this and move EEROM specific driver to a 
 * separate file
 */

/*
 * For EEPROM filesystem structure, we chain blocks with a 4 byte header.
 * First 3 bytes are the id, and the 4th is the config space length
 * (obviously limited to 256 bytes in this implementation).
 * 
 * The three bytes is a magic id.
 */

class config {
  public:
    config() {
      cid[0] = 0;
    };

    /*
     * fetch existing, or create new storage for id of length l
     */
    void fetch(const char * id, uint8_t l) {
      if(compare_id(id)) return; // we are already pointing to id
      // always scan from the start
      base = 0;
      get_cid();
      if(!compare_id("aFi")) {
        // check start of EEPROM for our magic - if not set, then init
        dbg(F("init eeprom"));
        EEPROM[0] = 'a'; // magic
        EEPROM[1] = 'F';
        EEPROM[2] = 'i';
        EEPROM[3] = 0; // id of first entry is empty - only need to clear first byte to mark an empty string
      }
      while(1) {
        get_cid(); // fetch next header
        len = EEPROM[base++]; // fetch len
        if(compare_id(id)) {
          // found existing id
          if(len != l) {
            // fixme - need some sort of assert
            Serial.println(F("EEPROM CORRUPT"));
            while(1) {}
          }
          // length matches
          return;
        }
        // did not match
        // end of chain?
        if(cid[0] == 0) {
          dbg(F("NEW CFG ID "));
          dbgln(id);
          if(l >= EEPROM.length() - base) {
            // fixme assert!
            Serial.println(F("CFG OUT OF SPACE"));
            while(1) {}
          }
          base -= 4; // we already read a header, so rewind to the start of it again...
          uint8_t i;
          for(i = 0; i < 3; i++)
            EEPROM[base++] = id[i]; // should actually length check - but not really seriosus if we excede bounds
          EEPROM[base++] = l;
          for(len = 0; len < l; len++) EEPROM.update(base + len, (uint8_t)255); // all newly allocated space is FF
          EEPROM[base + len] = 0; // 0 next header - empty id
          return;
        }
        base += len; // lext header
      }
    };

    /*
     * read from config store begining at offset o (0 being the start of this modules storage range)
     * read l bytes into address starting at t
     */
    void read(uint8_t o, void * t, uint8_t l) {
      uint8_t i;
      uint8_t * b = (uint8_t *)t; // cast void to byte array to make sure of casting later
      if(o + l > len) {
        // fixme assert
        Serial.println(F("cfg read beyond len"));
        while(1) {}
      }
      for(i = 0; i < l; i++) b[i] = EEPROM[base + o + i];
    };

    /*
     * write to config store begining at offset o (0 being the start of this modules storage range)
     * write l bytes from address starting at t
     */
    void write(uint8_t o, void * t, uint8_t l) {
      uint8_t i;
      uint8_t * b = (uint8_t *)t; // cast void to byte array to make sure of casting later
      if(o + l > len) {
        // fixme assert
        Serial.println(F("cfg write beyond len"));
        while(1) {}
      }
      for(i = 0; i < l; i++) EEPROM.update(base + o + i, b[i]);
    };

  private:
    bool compare_id(const char * id) {
      // compare id to our current config id
      return module::id_match(id, cid);
    };

    void get_cid(void) {
      uint8_t i;
      for(i = 0; i < 3; i++)
        cid[i] = EEPROM[base++];
    }
    
    char cid[3];
    int base;
    uint8_t len;
};

extern config CFG;

#endif
