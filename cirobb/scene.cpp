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

int Scene::CorrectionType = NONE;

void Scene::Add(RigidBody *body) { bodies.push_back(body); }

void Scene::Clear() {
  bodies.clear();
  manifolds.clear();
}

// Contact Persistence Algorithm Reference Erin catto.
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

/**********************************************************************************************
There are many ways to integrate the kinematic equations of motion. But the semi-implicit or
(symplectic) Euler It is the most widely used method in physics engines in Real-Time due to its
stability and speed.

Semi-Implicit Euler or Symplectic Euler

Linear movement equations

v = v + P * M⁻¹ + Fext * M⁻¹ * Δt

X = X + C * Δt

Rotational movement equations

ω = ω + L * I⁻¹ + τext * I⁻¹ * Δt

θ = θ + ω * Δt
***********************************************************************************************/
void Scene::Step(const real &dt) {
  BroadPhase();

  for (auto *b : bodies) {
    b->velocity += (b->force * b->invm + gravity * b->gravityScale) * dt; // v = v + Fext * M⁻¹ * Δt
    b->angularVelocity += b->torque * b->invI * dt;                       // ω = ω + τext * I⁻¹ * Δt
    b->velocity *= pow(0.97f, b->linearDamping);                          // min = 0.97f && max = 1;
    b->angularVelocity *= pow(0.97f, b->angularDamping);                  // min = 0.97f && max = 1;
  }

  for (ManifoldAux temp = manifolds.begin(); temp != manifolds.end(); temp++) {
    temp->second.PreStep(dt);
  }

  for (ManifoldAux temp = manifolds.begin(); temp != manifolds.end(); temp++) {
    // v = v + P * M⁻¹
    // ω = ω + L * I⁻¹
    temp->second.WarmStarting();
  }

  for (int i = 0; i < iterVel; i++) // (SI/PGS) Sequential-Impulse/Projected-Gauss-Seidel
  {
    for (ManifoldAux temp = manifolds.begin(); temp != manifolds.end(); temp++) {
      // v = v + P * M⁻¹
      // ω = ω + L * I⁻¹
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
    b->position += b->velocity * dt;           // x = x + v * Δt
    b->orientation += b->angularVelocity * dt; // θ = θ + ω * Δt
    b->force.SetZero();
    b->torque = 0.0f;
  }
}
