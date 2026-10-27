/*************************************************************************
 * Copyright (c) 2019-2026 Jonathan Peña
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Jonathan Peña makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 **************************************************************************/

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "cbMath.h"

struct Shape;

struct RigidBody {
  Vec2 position;
  Vec2 velocity;
  Vec2 force;
  real orientation;
  real angularVelocity;
  real torque;
  real m;               // Mass
  real invm;            // Inverse Mass
  real I;               // Inertia
  real invI;            // Inverse Inertia
  real friction;
  real restitution;
  real linearDamping;
  real angularDamping;
  real gravityScale;

  Shape *shape; // Circle, OBB

  RigidBody(Shape &, const Vec2 &, const real &);

  void Dynamic(const real &Density);

  void Static();
};
#endif
