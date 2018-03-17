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
 
#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "display_helper.h"

// comment out the following line to remove all debugging
#define _DEBUG

/*
 * dbgi(x) - set debug target object (anything that inherits from the Print class) - object must already be initialised/started
 * dbgd(d) - set debug target to a display object
 * dbx(x) - print x
 * dbgln(x) - print x with newline
 * 
 */

#ifdef _DEBUG

typedef struct {
  Print& p;
#ifdef _ADAFRUIT_GFX_H
  display * d;
#endif  
} _dbg_t;
extern _dbg_t _dbg; // one global debug object

// init basic debugger with print stream
#define dbgi(x) _dbg_t _dbg = {x, NULL};

// add debugger display device
#define dbgd(x) _dbg.d = &x

// print/ln
#ifndef _ADAFRUIT_GFX_H
#define dbg(x) _dbg.p.print(x)
#define dbgln(x) _dbg.p.println(x)
#else
// GFX custom versions, so we dont need to build if gfx not included...
// crude function to wrap back to top and clear line before printing...
#define SCROLLY 8
void _dbgd_up(void) {
    int y;
    y = _dbg.d->gfx.getCursorY();
    if(y >= _dbg.d->gfx.height()) {
        y = 0;
        _dbg.d->gfx.setCursor(0, 0);
    }
    if(_dbg.d->gfx.getCursorX() == 0) _dbg.d->gfx.fillRect(0, y, _dbg.d->gfx.width(), SCROLLY*2, 0);
}
#define dbg(x) if(_dbg.d) {_dbgd_up(); _dbg.d->gfx.print(x);} else {_dbg.p.print(x);}
#define dbgln(x) if(_dbg.d) {_dbgd_up(); _dbg.d->gfx.println(x);} else {_dbg.p.println(x);}
#endif

#else
// not defined _DEBUG - macro expand to empty instructions

#define dbgi(x)
#define dbgd(x)
#define dbg(x)
#define dbgln(x)

#endif

#endif
