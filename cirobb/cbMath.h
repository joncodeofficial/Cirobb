/*************************************************************************
 * Copyright (c) 2019-2026 Jonathan Peña
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Jonathan Peña makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 **************************************************************************/

#ifndef CBMATH_H
#define CBMATH_H

#include <cfloat>
#include <math.h>

typedef float real;
const real PI = 3.141592653589f;
const real RAD = PI / 180;
const real DEG = 180 / PI;
#if !defined(min) || !defined(max)
#define min(a, b) (((a) > (b)) ? (b) : (a))
#define max(a, b) (((a) < (b)) ? (b) : (a))
#endif
#define absf(a) (((a) < 0) ? -(real)(a) : (a))
#define pow2(a) ((a) * (a))
struct Vec2 {
  real x, y;

  Vec2() : x(0), y(0) { ; }

  Vec2(real _x, real _y) : x(_x), y(_y) { ; }

  void Set(const real &_x, const real &_y) {
    x = _x;
    y = _y;
  }

  // Vector Magnitude --> Pythagoras Theorem |C| = sqrt(A^2 + B^2)
  real Magnitude() const { return sqrtf(x * x + y * y); }

  // Vector Squared Magnitude -> Pythagoras Theorem C^2 = A^2 + B^2
  real SquareMagnitude() const { return x * x + y * y; }

  // Unit vector: cos(θ) = x / |v|, sin(θ) = y / |v|
  Vec2 normalize() const {
    real mag = Magnitude();
    return mag != 0.0f ? *this * (1.0f / mag) : Vec2();
  }

  void operator+=(const Vec2 &v) {
    x += v.x;
    y += v.y;
  }

  void operator-=(const Vec2 &v) {
    x -= v.x;
    y -= v.y;
  }

  void operator*=(const real &s) {
    x *= s;
    y *= s;
  }

  Vec2 operator+(const Vec2 &v) const { return Vec2(x + v.x, y + v.y); }

  Vec2 operator-(const Vec2 &v) const { return Vec2(x - v.x, y - v.y); }

  Vec2 operator-() const { return Vec2(-x, -y); }

  void SetAbs() {
    x = absf(x);
    y = absf(y);
  }

  Vec2 GetAbs() const { return Vec2(absf(x), absf(y)); }

  real operator*(const Vec2 &v) const { return (x * v.x + y * v.y); }

  Vec2 operator*(const real &s) const { return Vec2(x * s, y * s); }

  void SetZero() { x = y = 0.0f; }
};

// Dot product: x1 * x2 + y1 * y2
//   == 0 perpendicular (right angle)
//   >  0 same direction (acute)
//   <  0 opposite direction (obtuse)
inline real Dot(const Vec2 &v1, const Vec2 &v2) { return v1.x * v2.x + v1.y * v2.y; }

// Cross product (2D): x1 * y2 - y1 * x2, the determinant of the 2x2 matrix [v1 v2]
//   != 0 not parallel, one solution
//   == 0 parallel or the same vector, no solution or infinitely many
// k = i X j = -(j X i) --> counterclockwise
// Cross(v1, v2) = Dot(v1 Perpendicular, v2)
inline real Cross(const Vec2 &v1, const Vec2 &v2) { return v1.x * v2.y - v1.y * v2.x; }

// Cross Product between Vector and Scalar - Counterclockwise Direction
inline Vec2 Cross(const Vec2 &v, const real &s) { return Vec2(s * v.y, -s * v.x); }

inline real Clamp(const real &min, const real &max, const real &v) {
  if (v < min) return min;
  if (v > max) return max;
  return v;
}

/*
The 2x2 rotation matrix (counterclockwise):

  |cos(θ) -sin(θ)|
  |sin(θ)  cos(θ)|

Multiplying it by a vector gives the new components of the vector:

  |cos(θ) -sin(θ)| |x|
  |sin(θ)  cos(θ)| |y|

   x =  cos(θ) * x + sin(θ) * y
   y = -sin(θ) * x + cos(θ) * y
*/
class Mat2 {
private:
  real m00, m01;
  real m10, m11;

public:
  Mat2() : m00(0.0f), m01(0.0f), m10(0.0f), m11(0.0f) { ; }

  Mat2(const real &rad) {
    m00 = cosf(rad);
    m01 = -sinf(rad);
    m10 = -m01;
    m11 = m00;
  }

  Vec2 Rotate(const real &rad, const Vec2 &v) {
    m00 = cosf(rad);
    m01 = -sinf(rad);
    m10 = -m01;
    m11 = m00;

    return Vec2(v.x * m00 + v.y * m01, v.x * m10 + v.y * m11);
  }

  Vec2 Rotate(const Vec2 &v) const { return Vec2(v.x * m00 + v.y * m01, v.x * m10 + v.y * m11); }

  Vec2 operator*(const Vec2 &v) const { return Vec2(v.x * m00 + v.y * m01, v.x * m10 + v.y * m11); }

  Vec2 Column0() const { return Vec2(m00, m10); }

  Vec2 Column1() const { return Vec2(m01, m11); }

  void Transpose() {
    real temp = m01;
    m01 = m10;
    m10 = temp;
  }

  void SetTranspose(Vec2 &v) const {
    real x = v.x, y = v.y;
    v.x = x * m00 + y * m10;
    v.y = x * m01 + y * m11;
  }

  Vec2 Transpose(const Vec2 &v) const { return Vec2(v.x * m00 + v.y * m10, v.x * m01 + v.y * m11); }
};

#endif
