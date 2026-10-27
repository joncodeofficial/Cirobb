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

RigidBody::RigidBody(Shape &shape, const Vec2 &_position, const real &_orientation) {
  position = _position;
  orientation = _orientation;
  this->shape = shape.newShape();
  this->shape->body = this;
}

void RigidBody::Dynamic(const real &density) {
  this->angularVelocity = 0.0f;
  this->torque = 0.0f;
  this->velocity.SetZero();
  this->force.SetZero();
  // Values in the range [0..1]
  this->friction = 0.3f;
  this->restitution = 0.3f;
  this->angularDamping = 0.02f;
  this->linearDamping = 0.0f;
  this->gravityScale = 1.0f;
  this->shape->CalculateMassInertia(density);
}

void RigidBody::Static() {
  this->angularVelocity = 0.0f;
  this->torque = 0.0f;
  this->velocity.SetZero();
  this->force.SetZero();
  // Values in the range [0..1]
  this->friction = 1.0f;
  this->restitution = 0.0f;
  this->angularDamping = 0.0f;
  this->linearDamping = 0.0f;
  this->gravityScale = 0.0f;
  this->shape->CalculateMassInertia(0);
}
