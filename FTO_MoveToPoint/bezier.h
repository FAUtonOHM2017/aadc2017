#ifndef BEZIER_H
#define BEZIER_H

class bezier {

public:
  bezier(){};
  struct vec2 {
      float x, y;
      vec2(float x, float y) : x(x), y(y) {}
      vec2(): x(0), y(0) {}

      vec2 operator + (vec2 b) {
          return vec2(this->x + b.x, this->y + b.y);
      }

      vec2 operator - (vec2 b) {
          return vec2(this->x - b.x, this->y - b.y);
      }

      vec2 operator * (float s) {
          return vec2(s * this->x, s * this->y);
      }
  };


  vec2 getBezierPoint( vec2* , int , float);

};

#endif
