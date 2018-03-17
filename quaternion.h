#ifndef _QUATERNION_H_
#define _QUATERNION_H_

class quaternion {
  public:
    quaternion(void): Vmag(-1), V{0,0,0,0} {};
    quaternion(float w, float x, float y, float z): Vmag(-1), V{w,x,y,z} {};
    quaternion(float x, float y, float z): Vmag(-1), V{0,x,y,z} {};
    
    float mag(void) {
      float t;
      char i;
      t = 0;
      for(i = 0; i < 4; i++)
        t += V[i] * V[i];
      Vmag = sqrt(t);
      return Vmag;
    }

    float norm(void) {
      float t;
      char i;
      t = mag();
      for(i = 0; i < 4; i++)
        V[i] /= t;
      return t;
    }

    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    float pitch(void) {
      //float sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
      float sinp = +2.0 * (V[0] * V[2] - V[3] * V[1]);
      if (fabs(sinp) >= 1)
        return copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      else
        return asin(sinp);
    }

    float roll(void) {
      //float sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
      float sinr = +2.0 * (V[0] * V[1] + V[2] * V[3]);
      //float cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
      float cosr = +1.0 - 2.0 * (V[1] * V[1] + V[2] * V[2]);
      return atan2(sinr, cosr);
    }

    void avg(const quaternion& q, const float& alpha) {
      char i;
      for(i = 0; i < 4; i++) {
        V[i] *= alpha;
        V[i] += q[i];
        V[i] /= alpha + 1;
      }
    }

    quaternion operator*(const quaternion& q) {
      quaternion o;
      o[0] = V[0]*q[0]-V[1]*q[1]-V[2]*q[2]-V[3]*q[3];
      o[1] = V[0]*q[1]+V[1]*q[0]+V[2]*q[3]-V[3]*q[2];
      o[2] = V[0]*q[2]-V[1]*q[3]+V[2]*q[0]+V[3]*q[1];
      o[3] = V[0]*q[3]+V[1]*q[2]-V[2]*q[1]+V[3]*q[0];

      return o;
    }
  
    quaternion operator*(const float& f) {
      quaternion o;
      char i;
      for(i = 0; i < 4; i++)
        o[i] = V[i] * f;
      return o;
    }
  
    quaternion& operator*=(const float& f) {
      char i;
      for(i = 0; i < 4; i++)
        V[i] *= f;
      return *this;
    }
  
    quaternion operator+(const quaternion& q) {
      quaternion o;
      char i;
      for(i = 0; i < 4; i++)
        o.V[i] = V[i] + q[i];
      return o;
    }

    quaternion& operator-=(const quaternion& q) {
      char i;
      for(i = 0; i < 4; i++)
        V[i] -= q[i];
      return *this;
    }
  
    float& operator[](const int& i) {
      return V[i];
    }

    const float& operator[](const int& i) const { // need this too when dereferencing const class objects above
      return V[i];
    }

    void dump(void) {
#ifdef _DEBUG
      char i;
      for(i = 0; i < 4; i++) {
        dbg(V[i]);
        dbg(" ");
      }
      dbgln(" ");
#endif
    }
    float V[4];
    float Vmag;
};

#endif

