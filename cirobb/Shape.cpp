/*************************************************************************
 * Copyright (c) 2019-2021 Jonathan Peña
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Jonathan Peña makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 **************************************************************************/

#include "Shape.h"

/**********************************************************************************************************************
* Although there are phyisics engines that add the masses manually to avoid certain problems such
as: Low convergence of the solver when a heavy object is on top of a light object. The mass can also
be proportional to the geometric Area. For example:

* Circle Area = Radius * PI * PI

* Rectangle Area = width * height

These are the two geometric Areas that this physics engine uses to Obtain the relative masses of the
shapes.

*There is also the Inertia tensor: The Inertia tensor reflects the mass distribution of a Body or a
system of rotating particles, It can also be said that it's the resistance of the body to the
Rotation movement.

inertia Tensor matrix = |X  0  0|
                        |0  Y  0|
                        |0  0  Z|

2D physics engines have 3 Degrees of Freedom 2 of translation [X, Y, 0] and 1 of Rotation [0, 0, Z].
It Rotates on the Z Axis.

inertia Tensor in 2D is a Scalar.

Equation: Inertia = mass * Distance^2

1- Inertia Tensor of a Circle = mass * Radius * Radius / 2.0

2- Inertia Tensor of a Rectangle = mass * (x * x + y * y) / 12.0

***********************************************************************************************************************/

Circle::Circle(const real &_radius) {
  type = circle;
  radius = _radius;
}

Shape *Circle::newShape() { return new Circle(this->radius); }

void Circle::CalculateMassInertia(const real &density) {
  body->m = PI * radius * radius * density;
  body->invm = body->m != 0.0f ? 1.0f / body->m : 0.0f;
  body->I = body->m * radius * radius * 0.5f;
  body->invI = body->I != 0.0f ? 1.0f / body->I : 0.0f;
}

//************************************* O B B ***************************************
//************************************* O B B ***************************************

OBB::OBB(const Vec2 &_width) {
  type = obb;
  width = _width;
}

OBB::OBB(const real &x, const real &y) {
  type = obb;
  width.Set(x, y);
}

Shape *OBB::newShape() { return new OBB(this->width); }

void OBB::CalculateMassInertia(const real &density) {
  body->m = width.x * width.y * density;
  body->invm = body->m != 0.0f ? 1.0f / body->m : 0.0f;
  body->I = body->m * width.SquareMagnitude() / 12.0f;
  body->invI = body->I != 0.0f ? 1.0f / body->I : 0.0f;
}