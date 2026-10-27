/*************************************************************************
 * Copyright (c) 2019-2026 Jonathan Peña
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Jonathan Peña makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 **************************************************************************/

#ifndef SHAPE_H
#define SHAPE_H

#include "rigidBody.h"
#include <stdio.h>

enum typeShape { circle, obb };

struct Shape {
  real radius;
  Vec2 width;
  typeShape type;
  RigidBody *body;
  virtual Shape *newShape() = 0;
  virtual void CalculateMassInertia(const real &) = 0;
};

struct Circle : virtual public Shape {
  Circle(const real &_radius);

  ~Circle() { ; }

  Shape *newShape() override;

  void CalculateMassInertia(const real &density) override;
};

struct OBB : virtual public Shape {
  OBB(const Vec2 &_width);

  OBB(const real &x, const real &y);

  ~OBB() { ; }

  Shape *newShape() override;

  void CalculateMassInertia(const real &density) override;
};

#endif
