/*************************************************************************
 * Copyright (c) 2019-2026 Jonathan Peña
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Jonathan Peña makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 **************************************************************************/

#include "scene.h"

typedef std::pair<ManifoldKey, Manifold> ManPair;
typedef std::map<ManifoldKey, Manifold>::iterator ManifoldAux;

int Scene::CorrectionType = NGS;

void Scene::Add(RigidBody *body) { bodies.push_back(body); }

void Scene::Clear() {
  bodies.clear();
  manifolds.clear();
}

// Contact Persistence Algorithm
void Scene::BroadPhase() {
  const int length = static_cast<int>(bodies.size());

  for (int i = 0; i < length; i++) {
    RigidBody *bi = bodies[i];

    for (int j = i + 1; j < length; j++) {
      RigidBody *bj = bodies[j];

      if (bi->m + bj->m == 0.0f) continue;

      ManifoldKey key(bi, bj);
      Manifold newMan(bi, bj);

      if (newMan.numContacts > 0) {
        ManifoldAux iter = manifolds.find(key);

        if (iter == manifolds.end()) {
          manifolds.insert(ManPair(key, newMan));
        } else {
          iter->second.normal = newMan.normal;
          iter->second.A = newMan.A;
          iter->second.B = newMan.B;
          iter->second.postPosition = newMan.postPosition;
          iter->second.Update(newMan.contacts, newMan.numContacts);
        }
      } else manifolds.erase(key);
    }
  }
}

/*
Semi-Implicit (Symplectic) Euler:

v = v + P * M⁻¹ + Fext * M⁻¹ * Δt
X = X + C * Δt
ω = ω + L * I⁻¹ + τext * I⁻¹ * Δt
θ = θ + ω * Δt
*/
void Scene::Step(const real &dt) {
  BroadPhase();

  for (auto *b : bodies) {
    b->velocity += (b->force * b->invm + gravity * b->gravityScale) * dt;
    b->angularVelocity += b->torque * b->invI * dt;
    b->velocity *= pow(0.97f, b->linearDamping);                          // min = 0.97f && max = 1;
    b->angularVelocity *= pow(0.97f, b->angularDamping);                  // min = 0.97f && max = 1;
  }

  for (ManifoldAux temp = manifolds.begin(); temp != manifolds.end(); temp++) {
    temp->second.PreStep(dt);
  }

  for (ManifoldAux temp = manifolds.begin(); temp != manifolds.end(); temp++) {
    temp->second.WarmStarting();
  }

  for (int i = 0; i < iterVel; i++) // (SI/PGS) Sequential-Impulse/Projected-Gauss-Seidel
  {
    for (ManifoldAux temp = manifolds.begin(); temp != manifolds.end(); temp++) {
      temp->second.ApplyImpulse();
    }
  }

  if (CorrectionType == NGS) {
    for (int i = 0; i < iterPos;
         i++) // (NGS) Newton-Ranpson-Gauss-Seidel OR Non-Linear-Gauss-Seidel
    {
      for (ManifoldAux temp = manifolds.begin(); temp != manifolds.end(); temp++) {
        temp->second.ApplyCorrection();
      }
    }
  }

  for (auto *b : bodies) {
    b->position += b->velocity * dt;
    b->orientation += b->angularVelocity * dt;
    b->force.SetZero();
    b->torque = 0.0f;
  }
}
