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
 
#ifndef _GYRO_TOOLS_H_
#define _GYRO_TOOLS_H_

/*
 * Very simple (and hopelessly incomplete) tool to render gyros
 * 
 * TODO:
 * finish fill/ground drawing
 * draw aircraft picture
 * draw graduations
 */
class simple_ah: public display_module {
  public:
    simple_ah(display& d, gyro_t & gyro, const char * id, int x, int y, int w, int h): display_module(id, d), gyro(gyro) {
      x0 = x + w/2;
      y0 = y + h/2;
      if(w > h)
        r = h/2;
       else
        r = w/2;
    };

    void setup(void) {
    };

    void loop(unsigned long& ms) {
      // circle drawing basics
      int x = r-1;
      int y = 0;
      int dx = 1;
      int dy = 1;
      int err = dx - (r << 1);

      // quick and dirty for testing
      // we won't actually draw the horizon line, but we will fill the circle
      // with vertical coloured lines, breaking between sky and ground colour
      // at the horixon line

      // horizon line
      float ho = gyro.pitch * r / (M_PI/4); // 45 is full scale
      float ra = gyro.roll;
      int hx1, hy1, hx2, hy2;
      if(ho > -r && ho < r) {
        // if the horizon is visible
        float dang = acos(ho/r);
        hx1 = r * sin(ra - dang);
        hx2 = r * sin(ra + dang);
        hy1 = r * cos(ra - dang);
        hy2 = r * cos(ra + dang);
        d.gfx.drawLine(x0+hx1, y0+hy1, x0+hx2, y0+hy2, 1);
        // vert ref
        hx1 = r * sin(ra);
        hy1 = r * cos(ra);
        d.gfx.drawLine(x0, y0, x0+hx1, y0+hy1, 1);
      }

      // draw circle (midpoint algorithm)
      // FIXME - add vertical lines between pixel pairs for fill
      while (x >= y) {
        d.gfx.drawPixel(x0 + x, y0 + y, 1);
        d.gfx.drawPixel(x0 + x, y0 - y, 1);
        d.gfx.drawPixel(x0 + y, y0 + x, 1);
        d.gfx.drawPixel(x0 + y, y0 - x, 1);
        d.gfx.drawPixel(x0 - y, y0 + x, 1);
        d.gfx.drawPixel(x0 - y, y0 - x, 1);
        d.gfx.drawPixel(x0 - x, y0 + y, 1);
        d.gfx.drawPixel(x0 - x, y0 - y, 1);

        if (err <= 0)
        {
            y++;
            err += dy;
            dy += 2;
        }
        
        if (err > 0)
        {
            x--;
            dx += 2;
            err += dx - (r << 1);
        }
    }    
  };
    
  private:
    gyro_t& gyro;
    int x0;
    int y0;
    uint8_t r;
};


#endif

