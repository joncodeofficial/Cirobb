/*************************************************************************
 * Copyright (c) 2019-2021 Jonathan Peña
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
  // Circle characteristics
  real radius;
  // OBB characteristics
  Vec2 width;
  // Type of Shape
  typeShape type;
  // Rigid Body
  RigidBody *body;
  // Virtual Function To create Dynamic Shape
  virtual Shape *newShape() = 0;
  // Virtual Function to Compute Mass And Inertia
  virtual void CalculateMassInertia(const real &) = 0;
};

//*********************************** C I R C L E ***********************************
//*********************************** C I R C L E ***********************************

struct Circle : virtual public Shape {
  // Initialized Constructor
  Circle(const real &_radius);

  // Default Destructor
  ~Circle() { ; }

  Shape *newShape() override;

  void CalculateMassInertia(const real &density) override;
};

//************************************* O B B ***************************************
//************************************* O B B ***************************************

struct OBB : virtual public Shape {
  // Initialized Constructo with Vector
  OBB(const Vec2 &_width);

  // Initialized Constructor with scalars
  OBB(const real &x, const real &y);

  // Default Destructor
  ~OBB() { ; }

  Shape *newShape() override;

  void CalculateMassInertia(const real &density) override;
};

#endif
