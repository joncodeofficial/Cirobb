/*************************************************************************
* Copyright (c) 2019-2025 Jonathan Peña
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Jonathan Peña makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty. 
**************************************************************************/


#ifndef MANIFOLD_H
#define MANIFOLD_H


#include <map>
#include "Shape.h"


struct FeatureID
{
  int8_t refEdge;
  int8_t incEdge;
  int8_t clipIndex;

  FeatureID(void) : refEdge(0), incEdge(0), clipIndex(0) {;}
  FeatureID(int8_t _r, int8_t _i, int8_t _c) : refEdge(_r), incEdge(_i), clipIndex(_c) {;}

  bool operator==(const FeatureID& o) const { return refEdge == o.refEdge && incEdge == o.incEdge && clipIndex == o.clipIndex; }
};



struct Contact
{
  Contact(void) : Pn(0.0f), Pt(0.0f), bias(0.0f), restitution(0.0f), warmPoint(FLT_MAX, 0.0f) {;}

  Vec2 position; // Position of Contact Point
  Vec2 warmPoint; // Position of Warm Contact Point
  FeatureID id; // Feature ID for Warm Starting
  real penetration; // Penetration Depth
  real Pn; // Accumulated Normal Impulse
  real Pt; // Accumulated Tangent Impulse
  real massNormal;   // Normal Effective Mass
  real massTangent; // Tangent Effective Mass
  real bias; // Bias For Baumgarte Stabilization
  real restitution; // Restitution To Bounce
};



struct PostPosition
{
  real oldOrientationA; // Old Orientation A
  real oldOrientationB; // Old Orientation B
  Vec2 oldPointA[2]; // Old Contact Point A
  Vec2 oldPointB[2]; // Old Contact Point B
  
  PostPosition(void) : oldOrientationA(0.0f), oldOrientationB(0.0f) {;}
};



struct ManifoldKey
{
  RigidBody* A, *B;
  
  ManifoldKey(RigidBody* _A, RigidBody* _B) : A(_A), B(_B) {;}
};



struct Manifold
{
  enum {MAX_POINTS = 2};
  
  int numContacts;
  
  Contact contacts[MAX_POINTS];
  
  PostPosition postPosition;
  
  real u; // Friction Mixed
  
  real e; // Restitution Max
  
  RigidBody* A; RigidBody* B;
  
  Vec2 normal; // Normal Vector
  
  Manifold(RigidBody*, RigidBody*);
  
  void Update(Contact contacts[MAX_POINTS], const int&);
  
  void PreStep(const real&);
  
  void WarmStarting(void);
  
  void ApplyImpulse(void);
  
  void ApplyCorrection(void);
  
  real RecalculatePenetration(Vec2&, Vec2&, const int&);
};



// This is used by std::set. Erin catto
inline bool operator < (const ManifoldKey& a1, const ManifoldKey& a2)
{
  return (a1.A < a2.A || a1.A == a2.A && a1.B < a2.B);
}

#endif
