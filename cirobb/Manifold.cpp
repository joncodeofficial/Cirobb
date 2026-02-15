/*************************************************************************
* Copyright (c) 2019-2025 Jonathan Peña
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Jonathan Peña makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty. 
**************************************************************************/


#include "Collision.h"
#include "Scene.h"


Manifold::Manifold(RigidBody* _A, RigidBody* _B) : numContacts(0)
{
  A = _A; B = _B;
  
  this->u = sqrtf(A->friction * B->friction); // Mixed Friction
  
  this->e = max(A->restitution, B->restitution); // Max Restitution
  
  Dispatcher[A->shape->type][B->shape->type](*this, A->shape, B->shape);
}



/****************************************************************************************************************
if the manifold exists in the list, we check if there is any contact point that can be Warm Starting
This can be achieved with a distance heuristic, id, Etc. Reference Erin Catto
*****************************************************************************************************************/
void Manifold::Update(Contact* newContacts, const int& numNewContacts)
{
 
  Contact mergedContacts[2];

  for (int i = 0; i < numNewContacts; i++)
  {
    mergedContacts[i] = newContacts[i];
    mergedContacts[i].warmPoint = mergedContacts[i].position;

    for(int j = 0; j < numContacts; j++)
    {
      if(newContacts[i].id == contacts[j].id)
      {
      	mergedContacts[i].Pn = contacts[j].Pn;
      	mergedContacts[i].Pt = contacts[j].Pt;
      	mergedContacts[i].warmPoint = Vec2(FLT_MAX, 0.0f);
      	break;
      }
    }
  }
  numContacts = numNewContacts;
  
  for(int i = 0; i < numContacts; i++) contacts[i] = mergedContacts[i];
}



void Manifold::WarmStarting(void)
{
  Vec2 tangent = Cross(normal, -1.0f);
  
  for(int i = 0; i < numContacts; i++)
  {
    Contact* c = contacts + i;
    
    Vec2 ra = A->position - c->position;
    Vec2 rb = B->position - c->position;
    
    Vec2 P = normal * c->Pn + tangent * c->Pt;
    
    A->velocity -= P * A->invm;
    B->velocity += P * B->invm;
    A->angularVelocity -= Cross(P, ra) * A->invI;
    B->angularVelocity += Cross(P, rb) * B->invI;
  }
}



/***********************************************************************************************************************
This PreStep Method calculates the normal and tangent inverse effective mass of each Contact point and Baumgarte Stabilization.

Equation: A = J * M⁻¹ * Jt

* Jt = transposed Jacobian

* Only in 2D the inverse effective mass is =

                 J                                M⁻¹                       Jt
[nx, ny, n X ra, -nx, -ny, -n X rb] |m1⁻¹  0      0     0     0       0| | nx    |
                                    |0     m1⁻¹   0     0     0       0| | ny    |
                                    |0     0      I1⁻¹  0     0       0| | n X ra|  
                                    |0     0      0     m2⁻¹  0       0| |-nx    |  
                                    |0     0      0     0     m2⁻¹    0| |-ny    |    
                                    |0     0      0     0     0    I2⁻¹| |-n X rb|

* The X in this case Represents the Cross Product.

* This is only 1 constraint with 2 rigid bodies involved.

* The final result will be a scalar that is equal to: A⁻¹ = m1⁻¹ + m2⁻¹ + (n X r1)^2 * I1⁻¹ + (n X r2)^2 * I2⁻¹

* The same goes for the "A" but with the tangent vector.

* Equation: At⁻¹ = m1⁻¹ + m2⁻¹ + (t X r1)^2 * I1⁻¹ + (t X r2)^2 * I2⁻¹

* Then we have the Baumgarte Stabilization that transforms the "Position Error" into a "Velocity Error".
 
* Velocity Bias = Baumgarte = x / Δt * ϵ

* After calculating the value of Baumgarte Stabilization, We use the same Solver that resolves the Velocity Constraint to resolves penetration.
*******************************************************************************************************************************************/
void Manifold::PreStep(const real& dt)
{
  const real k_slop = 0.004f;
  const real k_biasFactor = 0.1f;
  
  real massLinear = A->invm + B->invm;
  
  for(int i = 0; i < numContacts; i++)
  {
    Contact* c = contacts + i;
    
    Vec2 ra = A->position - c->position;
    Vec2 rb = B->position - c->position;
    
    c->massNormal = massLinear + pow2(Cross(normal, ra)) * A->invI + pow2(Cross(normal, rb)) * B->invI;
    
    c->massNormal = 1.0f / c->massNormal;
    
    c->massTangent = massLinear + pow2(Dot(normal, ra)) * A->invI + pow2(Dot(normal, rb)) * B->invI; // Cross(tangent, r)^2 = Dot(normal, r)^2 : in 2D
    
    c->massTangent = 1.0f / c->massTangent;
    
    Vec2 dv = B->velocity + Cross(rb, B->angularVelocity) - A->velocity - Cross(ra, A->angularVelocity);
    
    real vn = dv * normal;
    
    c->restitution = vn < -1.0f ? vn * e : 0.0f;
    
    if(Scene::CorrectionType == BAUMGARTE) // Baumgarte Stabilization
    {
      c->bias = min(0.0f, c->penetration + k_slop) * k_biasFactor / dt;
    }
  }
}



/****************************************************************************
 Non-Penetration Constraint = (v2 - v1) * J >= 0

 Coulomb Friction law |λt| <= uλn.

 We Resolve the Non-penetration Constraint resolving: x = -b * A⁻¹
 
 b = (v2 - v1) * J  Relative normal Velocity 

 A = J * M⁻¹ * Jt  Inverse Effective Mass 

 x = λ = |P| = -b * A⁻¹; // The Lambda is constraint impulse signed magnitude.

 P = Jt * λ;
 
 L = P X r; 

 v2 = v1 + P * m⁻¹

 ω2 = ω1 + L * I⁻¹
*****************************************************************************/
void Manifold::ApplyImpulse(void)
{
  // Friction Constraint based on Coulomb law: |λt| <= uλn "OR" -uλn <= λt <= uλn
  
  Vec2 tangent = Cross(normal, -1.0f); // Vector Tangent
  
  for(int i = 0; i < numContacts; i++)
  {
    Contact* c = contacts + i; 
    
    Vec2 ra = A->position - c->position;
    Vec2 rb = B->position - c->position;
    
    Vec2 dv = B->velocity + Cross(rb, B->angularVelocity) - A->velocity - Cross(ra, A->angularVelocity); // (v2 - v1) Δv Relative Velocity
    
    real vt = dv * tangent; // Tangent Relative Velocity
    
    real dPt = -vt * c->massTangent; // Ax + b = 0 --> x = -b * A⁻¹; 
    
    real MaxPt = u * c->Pn; // MaxPt = uλn
    real Pt0 = c->Pt;
    c->Pt = Clamp(-MaxPt, MaxPt, Pt0 + dPt); // |λt| <= uλn  "OR"  -uλn <= λt <= uλn
    dPt = c->Pt - Pt0;
    
    Vec2 P = tangent * dPt; // Jt * λ
    
    A->velocity -= P * A->invm;
    B->velocity += P * B->invm;
    A->angularVelocity -= Cross(P, ra) * A->invI;
    B->angularVelocity += Cross(P, rb) * B->invI;
  }
  
  // Non-Penetration Constraint = (v2 - v1) * J >= 0
  
  for(int i = 0; i < numContacts; i++)
  {
    Contact* c = contacts + i; 
    		
    Vec2 ra = A->position - c->position;
    Vec2 rb = B->position - c->position;
    
    Vec2 dv = B->velocity + Cross(rb, B->angularVelocity) - A->velocity - Cross(ra, A->angularVelocity); // (v2 - v1) Δv Relative Velocity
    
    real vn = dv * normal; // Normal Relative Velocity
    
    real dPn = -(vn + c->restitution + c->bias) * c->massNormal; // Ax + b = 0 --> x = -b * A⁻¹; 
    
    real Pn0 = c->Pn;
    c->Pn = max(c->Pn + dPn, 0.0f); // Accumulated Impulse & (v2 - v1) * J / M⁻¹ >= 0
    dPn = c->Pn - Pn0;
    
    Vec2 P = normal * dPn; // Jt * λ
    
    A->velocity -= P * A->invm; 
    B->velocity += P * B->invm;
    A->angularVelocity -= Cross(P, ra) * A->invI;
    B->angularVelocity += Cross(P, rb) * B->invI;
  }
}



 real Manifold::RecalculatePenetration(Vec2& normal, Vec2& position, const int& i)
 {
  Shape* sA = this->A->shape;
  Shape* sB = this->B->shape; 
  
  PostPosition p = postPosition;
  
  Vec2 distance = B->position - A->position;
  
  if(sA->type + sB->type == 0) // CircleToCircle
  {
    real magnitude = distance.Magnitude();
    normal = this->normal;
    position = A->position + normal * ((sA->radius - sB->radius + magnitude) * 0.5f);
    return magnitude - sA->radius - sB->radius;
  }
  else
  if(sA->type + sB->type == 1) // CircleToOBB || OBBToCircle
  {
    Mat2 rotB(B->orientation - p.oldOrientationB);
    normal = rotB * this->normal;
    Vec2 point2 = rotB * p.oldPointB[i];
    real magnitude = normal * (distance + point2);  
    position = A->position + normal * magnitude;
    return magnitude - sA->radius;
  }
  else
  if(sA->type + sB->type == 2) // OBBToOBB
  {	
    Mat2 rotA(A->orientation - p.oldOrientationA);
    Mat2 rotB(B->orientation - p.oldOrientationB);
    normal = rotB * this->normal;
    Vec2 point1 = rotA * p.oldPointA[i];
    Vec2 point2 = rotB * p.oldPointB[i];
    real penetration = normal * (distance + point2 - point1);
    position = A->position + point1 + normal * penetration;	
    return penetration;
  }
  return 0;
}

 

/****************************************
  C(x + dx) ~ = C(x) + J * dx 
  
  dx = M ^ -1 * J ^ T * λ
  
  C(x) + J * M ^ -1 * J ^ T * λ = 0
  
  We Resolve for the lambda λ
  
  λ = -C(x) / J * M ^ -1 * J ^ T
****************************************/
void Manifold::ApplyCorrection(void)
{
  const real k_slop = 0.002f;
  const real k_biasFactor = 0.2f;
  
  real massLinear = A->invm + B->invm;
  
  Vec2 XA = A->position;
  Vec2 XB = B->position;
  real θA = A->orientation;
  real θB = B->orientation;
  
  Vec2 contactPoint, normal;
  
  for(int i = 0; i < numContacts; i++)	
  {
    real penetration = RecalculatePenetration(normal, contactPoint, i);
    
    Vec2 ra = A->position - contactPoint;
    Vec2 rb = B->position - contactPoint;
    
    real massNormal = massLinear + pow2(Cross(normal, ra)) * A->invI + pow2(Cross(normal, rb)) * B->invI;
    
    Vec2 C = normal * -min(penetration + k_slop, 0) * (k_biasFactor / massNormal); // Jt * λ
    
    XA -= C * A->invm;
    XB += C * B->invm;
    θA -= Cross(C, ra) * A->invI;
    θB += Cross(C, rb) * B->invI;
  }
  
  A->position = XA;
  B->position = XB;
  A->orientation = θA;
  B->orientation = θB;
}
