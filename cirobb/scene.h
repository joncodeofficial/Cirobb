/*************************************************************************
* Copyright (c) 2019-2026 Jonathan Peña
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Jonathan Peña makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty. 
**************************************************************************/


#ifndef SCENE_H
#define SCENE_H

#include <map>
#include <vector>
#include "manifold.h"

enum {BAUMGARTE = 0, NGS};

struct Scene
{
  static int CorrectionType; // Baumgarte Stabilization  "o"  NonLinear-GaussSeidel
  
  Vec2 gravity; // Gravity Vector
  
  int iterPos, iterVel;
  
  std::vector<RigidBody*> bodies;
  
  std::map<ManifoldKey, Manifold> manifolds;
  
  Scene(const Vec2& _gravity, const int& _iterVel, const int& _iterPos) : gravity(_gravity), iterVel(_iterVel), iterPos(_iterPos) {;}
  
  void Clear();
  
  void Add(RigidBody*);
  
  void Step(const real&);
  
  void BroadPhase();
};

#endif