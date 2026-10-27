/*************************************************************************
 * Copyright (c) 2019-2021 Jonathan Peña
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
// Irrational Constant PI
const real PI = 3.141592653589f;
// Conversion of Degrees to Radians
const real RAD = PI / 180;
// Conversion of Radians to Degrees
const real DEG = 180 / PI;
// Define Functions if there is no other Declaration.
#if !defined(min) || !defined(max)
// the Minimum Value is Returned
#define min(a, b) (((a) > (b)) ? (b) : (a))
// the Maximum value is Returned
#define max(a, b) (((a) < (b)) ? (b) : (a))
#endif
// The Absolute Value is Retured
#define absf(a) (((a) < 0) ? -(real)(a) : (a))
// The Square Scalar is returned
#define pow2(a) ((a) * (a))
/********************************************************************************************
A Tuple of n Real numbers called components of the Vector is called a vector of n Dimensions.
Vectors allow us to represent physical vector quantities.
The abstraction of vectors is very important to understand.
instead of solving each unknown separately the vectors can abstract the dimensions [x, y, z...n]
so they can be resolved together. There are Two Common ways to represent vectors:

Polar coordinates: Magnitude, Direction And Sense

Cartesian Coordinates: The components of the vector in the Cartesian axes[x, y, z...n].

This Project has extensive use of vectors so it's important to know that each operation does.
********************************************************************************************/
struct Vec2 {
  real x, y;

  // Default Constructor
  Vec2() : x(0), y(0) { ; }

  // Alternative Contructor
  Vec2(real _x, real _y) : x(_x), y(_y) { ; }

  // SetS The Values of The Vector
  void Set(const real &_x, const real &_y) {
    x = _x;
    y = _y;
  }

  // Vector Magnitude --> Pythagoras Theorem |C| = sqrt(A^2 + B^2)
  real Magnitude() const { return sqrtf(x * x + y * y); }

  // Vector Squared Magnitude -> Pythagoras Theorem C^2 = A^2 + B^2
  real SquareMagnitude() const { return x * x + y * y; }

  /*************************************************************************************
  The Normalization of the vector is a very useful operation that gives us the unitary direction of
  the vector. cos(x) = adyacent / hypotenuse = x / magnitude sin(x) = opposite / hypotenusa = y /
  magnitude
  *************************************************************************************/
  Vec2 normalize() const {
    real mag = Magnitude();
    return mag != 0.0f ? *this * (1.0f / mag) : Vec2();
  }

  // Adding A Vector
  void operator+=(const Vec2 &v) {
    x += v.x;
    y += v.y;
  }

  // Subtracting A vector
  void operator-=(const Vec2 &v) {
    x -= v.x;
    y -= v.y;
  }

  // Multiplying Vector * Scalar
  void operator*=(const real &s) {
    x *= s;
    y *= s;
  }

  // Sum of Vectors
  Vec2 operator+(const Vec2 &v) const { return Vec2(x + v.x, y + v.y); }

  // Subtraction of Vectors
  Vec2 operator-(const Vec2 &v) const { return Vec2(x - v.x, y - v.y); }

  // Negation of Vector
  Vec2 operator-() const { return Vec2(-x, -y); }

  // Sets the components as Absolute values.
  void SetAbs() {
    x = absf(x);
    y = absf(y);
  }

  // Gets The Components in Absolute Value.
  Vec2 GetAbs() const { return Vec2(absf(x), absf(y)); }

  /****************************************************************************
  One of the most important operations in the vectors is the dot product.
  The dot product tells us how much the Vector 'A' is pointing over the Vector 'B'.
  Avoid making expensive projections and calculating square roots.
  This operation will be widely used both in collision detection and in physics.

  If(x1 * x2 + y1 * y2 == 0) Then the vectors are perpendicular. Rigth Angle.
  If(x1 * x2 + y1 * y2 >  0) Then the vectors point in the same direction. Acute.
  If(x1 * x2 + y1 * y2 <  0) Then the vectors point in the opposite direction. Obtuse.
  *****************************************************************************/
  real operator*(const Vec2 &v) const { return (x * v.x + y * v.y); }

  // Vector Product * Scalar
  Vec2 operator*(const real &s) const { return Vec2(x * s, y * s); }

  // Sets the Components of the vector equal to Zero
  void SetZero() { x = y = 0.0f; }
};

/****************************************************************************
One of the most important operations in the vectors is the dot product.
The dot product tells us how much the Vector 'A' is pointing over the Vector 'B'.
Avoid making expensive projections and calculating square roots.
This operation will be widely used both in collision detection and in physics.

If(x1 * x2 + y1 * y2 == 0) Then the vectors are perpendicular. Rigth Angle.
If(x1 * x2 + y1 * y2 >  0) Then the vectors point in the same direction. Acute.
If(x1 * x2 + y1 * y2 <  0) Then the vectors point in the opposite direction. Obtuse.
*****************************************************************************/
inline real Dot(const Vec2 &v1, const Vec2 &v2) { return v1.x * v2.x + v1.y * v2.y; }

/****************************************************************************
One of the most important operations of vectors is the Cross Product.
We can see that it's the same equation as the determinant of a matrix 2x2 == (x1 * y2 - y1 * x2)

If (x1 * y2 - y1 * x2 != 0) Then The vectors are not parallel, There is only one solution.
If (x1 * y2 - y1 * x2 == 0) Then The vectors are parallel or the Same vector. There is no solution
or there are an infinite number of solutions.

i == X;
j == Y;
k == Z;

 k = i X j = -j X i .-> Counterclockwise
-k = j X i = -i X j --> Not Counterclockwise

Cross(v1, v2) = Dot(v1 Perpendicular, v2);

The function was defined to go counterclockwise.
****************************************************************************/
inline real Cross(const Vec2 &v1, const Vec2 &v2) { return v1.x * v2.y - v1.y * v2.x; }

// Cross Product between Vector and Scalar - Counterclockwise Direction
inline Vec2 Cross(const Vec2 &v, const real &s) { return Vec2(s * v.y, -s * v.x); }

// Function to limit the minimum and maximum value of the variable
inline real Clamp(const real &min, const real &max, const real &v) {
  if (v < min) return min;
  if (v > max) return max;
  return v;
}

/*********************************************************************************
The rotation matrices are square matrices, with real entries. these matrices describe rotations
about the origin. The matrices also have their respective operations such as: addition, subtraction
and multiplication. but this Mat2 class has only operations of the rotation matrix.

The 2x2 rotation matrix is:

  |cos(θ) -sin(θ)|
  |sin(θ)	 cos(θ)|

We only need the distance in Cartesian coordinates and The Orientation in polar.

  |cos(θ) -sin(θ)| |x|
  |sin(θ)  cos(θ)| |y|

We multiply the matrix by
the vector we obtain the new components of the vector with respect to rotation.

   x =  cos(θ) * x + sin(θ) * y
   y = -sin(θ) * x + cos(θ) * y

The direction of rotation of this matrix is counterclockwise
**********************************************************************************/
class Mat2 {
private:
  real m00, m01;
  real m10, m11;

public:
  // Default Constructor
  Mat2() : m00(0.0f), m01(0.0f), m10(0.0f), m11(0.0f) { ; }

  // Initialized Constructor with radian
  Mat2(const real &rad) {
    m00 = cosf(rad);
    m01 = -sinf(rad);
    m10 = -m01;
    m11 = m00;
  }

  // Returns the Rotation Matrix (Radian - Distance)
  Vec2 Rotate(const real &rad, const Vec2 &v) {
    m00 = cosf(rad);
    m01 = -sinf(rad);
    m10 = -m01;
    m11 = m00;

    return Vec2(v.x * m00 + v.y * m01, v.x * m10 + v.y * m11);
  }

  // Returns the Rotation Matrix
  Vec2 Rotate(const Vec2 &v) const { return Vec2(v.x * m00 + v.y * m01, v.x * m10 + v.y * m11); }

  // Returns the Rotation Matrix
  Vec2 operator*(const Vec2 &v) const { return Vec2(v.x * m00 + v.y * m01, v.x * m10 + v.y * m11); }

  // Returns The Column 0 of The matrix
  Vec2 Column0() const { return Vec2(m00, m10); }

  // Returns The Column 1 Of The matrix
  Vec2 Column1() const { return Vec2(m01, m11); }

  // Sets the transpose Matrix
  void Transpose() {
    real temp = m01;
    m01 = m10;
    m10 = temp;
  }

  // Sets the Vector with Transpose Matrix
  void SetTranspose(Vec2 &v) const {
    real x = v.x, y = v.y;
    v.x = x * m00 + y * m10;
    v.y = x * m01 + y * m11;
  }

  // Returns the transpose Matrix
  Vec2 Transpose(const Vec2 &v) const { return Vec2(v.x * m00 + v.y * m10, v.x * m01 + v.y * m11); }
};

#endif
