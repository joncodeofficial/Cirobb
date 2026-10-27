/*************************************************************************
 * Copyright (c) 2019-2026 Jonathan Peña
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Jonathan Peña makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 **************************************************************************/

#include "shape.h"

/*
The mass is proportional to the geometric area:

* Circle Area = Radius * PI * PI

* Rectangle Area = width * height

The inertia tensor is the resistance of the body to rotation. A 2D body has 3 degrees of freedom
(X, Y translation and Z rotation), so the inertia tensor is a scalar.

Inertia = mass * Distance^2

1- Inertia of a Circle = mass * Radius * Radius / 2.0

2- Inertia of a Rectangle = mass * (x * x + y * y) / 12.0
*/

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