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


/************************************************************************************************
Collision detection is one of the most important and problematic parts of physics engines.
Collision detection gives us information about: normal, penetration, Contact Points, ETC.
There are two types of Collision Detection: (DCD)Discrete Collision Detection and (CCD) Continuous Collision Detection.
The Collision Detection type Found in this File.cpp is only (DCD).
There are three Phases of Collision Detection: Broad Phase, Medium Phase and Narrow Phase.
The Collision Detection Found in this File.cpp is only Narrow Phase.
The Broad Phase is a brute force algorithm of complexity O (n^2). and has no Middle Phase.

Thank you very much Dirk Gregorius.
*************************************************************************************************/






/************************************************************************************************
Collision Detection Circle to Circle is one of the fastest and easiest. All this due to its properties. The circle is a highly symmetrical Shape: 
each line through the center forms a line of reflection symmetry and has rotational symmetry around the center for each angle.
so if(there is a point on the circunference and the origin is the center of the circle) then Radius = sqrt ((x - cx)^2 + (y - cy)^2). 
we taking advantage of the property that the circles are highly symmetrical We can obtain the minimum distance subtracting the centers and Radii. 
so if (|c2 - c1| <= r1 + r2)) then there is a collision and we have to get this information for the solver.
When a circle collides with any convex polygon a Single Contact Point is sufficient.

* Penetration = |c2 - c1| - r1 - r2

* Normal = (c2 - c1) / |c2 - c1|

* ContactPoint = c1 + n * ((r1 - r2 + |c2 - c1|) * 0.5f) --> The point in the middle. This is a way of do it.

There is a degenerate case and it happens when the centers of the circles coincide resulting in |c2 - c1| = 0. This case must be manipulated manually or ignored.
*************************************************************************************************/
void CircleToCircle(Manifold& m, Shape* c1, Shape* c2)
{
  // Distance from center to center
  Vec2 distance = c2->body->position - c1->body->position;
  
  // Sum of Radii
  real radius = c2->radius + c1->radius;
  
  // Squared Magnitude |c2 - c1|^2
  real magnitude = distance.SquareMagnitude();
  
  // Since we are only interested in knowing if there is a Collision.
  // We can avoid the square root by raising the two members of the equation to power 2.
  if(magnitude > radius * radius) return;
  
  // Magnitude |c2 - c1|
  magnitude = sqrtf(magnitude);
  
  m.numContacts = 1;
  m.contacts[0].penetration = magnitude - radius;
  m.normal = magnitude ? distance * (real(1) / magnitude) : Vec2(0, 1);
  m.contacts[0].position = c1->body->position + m.normal * ((c1->radius - c2->radius + magnitude) * real(0.5));
}




/************************************************************************************************
Collision Detection Circle to OBB(Oriented bounding box) Can be simplified to a Collision Detection
Circle to AABB(Axis Aligned Bounding Box) by moving the OBB to its local space Transforming it in an AABB.
So we have to find the closest point to the circle. After obtaining the closest point we have a problem of the form |c2 - c1| <= r1 + r2.
But since a point has radius = 0 the problem is simplified to |c2 - c1| <= r1.
if there is a collision we have to get our information for the Solver and then return it to the world space. Just one point of contact is sufficient.

There are two cases:

Shallow Penetration: The center of the circle is outside the OBB.

Deep Penetration: The center of the circle is inside the OBB.

Case = |c2 - c1| == 0 ? Deep Penetration : Shallow Penetration

if(We have a case of Shallow Penetration) then We have all the necessary information to calculate: normal, Penetration and Contact point.

if(We have a case of Deep Penetration) then we have to find the axis of minimum penetration to calculate: Normal, Penetration and Contact Point.

*************************************************************************************************/
// Collision Detection Circle to Oriented Rectangle
void CircleToOBB(Manifold& m, Shape* c1, Shape*	b1)
{
  Mat2 Rot(-b1->body->orientation);
  
  Vec2 distance = c1->body->position - b1->body->position;
  
  Vec2 localSpace = Rot * distance;
  
  Vec2 width = b1->width * real(0.5);
  
  real dx = absf(localSpace.x) - width.x;
  
  if(dx > c1->radius) return;
  
  real dy = absf(localSpace.y) - width.y;
  
  if(dy > c1->radius) return;
  
  Vec2 closest; // Closest Point
  
  // 4 Conditions to Find the Closest Point of the Circle.
  if(-localSpace.x > width.x) closest.x =  dx; else
  if( localSpace.x > width.x) closest.x = -dx;
  if(-localSpace.y > width.y) closest.y =  dy; else
  if( localSpace.y > width.y) closest.y = -dy;
  
  real magnitude = closest.SquareMagnitude();
  
  if(magnitude > c1->radius * c1->radius) return;
  
  if(magnitude) // Shallow Penetration
  {
    magnitude = sqrtf(magnitude); 
    m.normal = closest * (real(1) / magnitude);		
  }
  else // Deep Penetration
  {
    magnitude = max(dx, dy);
    m.normal = dx > dy ? Vec2(localSpace.x < 0 ? 1 : -1, 0) : Vec2(0, localSpace.y < 0 ? 1 : -1);
  }
  
  m.numContacts = 1;
  m.contacts[0].penetration = magnitude - c1->radius;
  Rot.SetTranspose(m.normal);
  m.contacts[0].position = c1->body->position + m.normal * magnitude;
  
  // Post-Projection Position
  m.postPosition.oldPointB[0] = distance + m.normal * magnitude;
  m.postPosition.oldOrientationB = b1->body->orientation;
}




// Collision Detection Oriented Rectangle To Circle
void OBBToCircle(Manifold& m, Shape* A, Shape* B)
{
  m.A = B->body; 
  m.B = A->body; 
  CircleToOBB(m, B, A);
}




/*******************************************************************************************
Collision Detection OBB To OBB It is the most Problematic method we will see in this file.cpp.
This Method uses the SAT (Separation Axis Theorem), finding the Minimum and Maximum to obtain the Minimum Penetration Axis.
Collision detection OBB to OBB also has a Complete set of Problems such as: 
Inconsistent Points, Flip-Flop , Similar Edges, Rotation Rounding Errors, Etc.
OBBs (Oriented Bounding Boxes) have certain Optimizations due to their Simplicity. Although There are 8 normals that must be tested, 
each OBB has 2 parallel edges that reduce it to 4 normals that must be tested.
We can also transform the collision detection Problem into an AABBToOBB Test.
**********************************************************************************************/
inline static int FindIncidentEdgeIndex(const real& projection1, const real& projection2)
{
  int i = projection1 < 0.0f ? 1 : -1;
  
  if(projection1 * i >  projection2) return 1;
  if(projection1 * i > -projection2) return 3;
  
  return 1 - i;
}



inline static Vec2 ClipSegment(Vec2& A, Vec2& B, Vec2& C, const Vec2& n)
{
  Vec2 BA = B - A;
  Vec2 CA = C - A;
  
  real t = (n.x * CA.y - n.y * CA.x) / (n.x * BA.y - n.y * BA.x);
  
  return (0 > t || t > 1)? A : A + BA * t;
}



// Collision Detection Oriented Rectangle To Oriented Rectangle
void OBBToOBB(Manifold& m, Shape* b1, Shape* b2)
{
  Mat2 Rot1(b2->body->orientation - b1->body->orientation);
  
  Mat2 Rot2(-b1->body->orientation);
   
  Vec2 hA = b1->width * real(0.5);
  Vec2 hB = b2->width * real(0.5);
  
  Vec2 hB1 = Rot1 * hB;
  Vec2 hB2 = Rot1 * Vec2(-hB.x, hB.y);	 
  
  Vec2 localSpace = Rot2.Rotate(b2->body->position - b1->body->position);
  
  real Bx = hA.x + max(absf(hB1.x), absf(hB2.x));
  
  if(localSpace.x * localSpace.x > Bx * Bx) return;
  
  real By = hA.y + max(absf(hB1.y), absf(hB2.y));
  
  if(localSpace.y * localSpace.y > By * By) return;
  
  Vec2 normalB = -Rot1.Column0(); 
  
  Vec2 vertsA[] = {Vec2(-hA.x, hA.y), -hA, Vec2(hA.x, -hA.y), hA};
  
  Vec2 vertsB[] = {localSpace + hB2, localSpace - hB1, localSpace - hB2, localSpace + hB1};
  
  real Ax1 = Dot(normalB, vertsA[0]); 
  real Ax2 = Dot(normalB, vertsA[1]);
  real Bx1 = Dot(normalB, vertsB[3]);
  real Bx2 = Dot(normalB, vertsB[1]);
  
  real Ax = max(absf(Ax1), absf(Ax2));
  
  if((-Ax + Bx1) * (Ax + Bx2) > 0) return; 
  
  real Ay1 = Cross(normalB, vertsA[0]);
  real Ay2 = Cross(normalB, vertsA[1]); 
  real By1 = Cross(normalB, vertsB[3]);
  real By2 = Cross(normalB, vertsB[1]);
  
  real Ay = max(absf(Ay1), absf(Ay2));
  
  if((-Ay + By1) * (Ay + By2) > 0) return;
  
  Vec2 axes[] = {Vec2(-1, 0), Vec2(0, -1), normalB, Cross(normalB, -1)};
  
  real v1[] = {Bx + localSpace.x, By + localSpace.y, Ax - Bx1, Ay - By1};
  real v2[] = {Bx - localSpace.x, By - localSpace.y, Ax + Bx2, Ay + By2};
  
  int axisIndex = 0, numContacts = 0; 
  
  real penetration = min(v1[0], v2[0]); 
  
  for(int i = 1; i < 4; i++)
  {
    real minimum = min(v1[i], v2[i]);
  
    if(minimum <= penetration + 0.007f)
    {
      axisIndex = i; penetration = minimum;
    }
  }
  
  Vec2 ref[2], inc[2];
  int refEdgeIdx, incEdgeIdx;
  
  int side = v1[axisIndex] < v2[axisIndex] ? 1 : -1;
  
  Vec2 normal = axes[axisIndex] * real(side); //Localspace normal 
  
  if(axisIndex < 2)
  {
    m.A = b2->body; m.B = b1->body;
    if(side == -1) axisIndex += 2;
    ref[0] = vertsA[axisIndex];
    ref[1] = vertsA[(axisIndex + 1) % 4];
    refEdgeIdx = axisIndex;
    axisIndex = FindIncidentEdgeIndex(normal * axes[2], normal * axes[3]);
    incEdgeIdx = axisIndex;
    inc[0] = vertsB[axisIndex];
    inc[1] = vertsB[(axisIndex + 1) % 4];
    normal *= -1;
  }
  else
  {
    if(side == -1) axisIndex -= 2;
    ref[0] = vertsB[axisIndex];
    ref[1] = vertsB[(axisIndex + 1) % 4];
    refEdgeIdx = axisIndex;
    axisIndex = FindIncidentEdgeIndex(normal.x, normal.y);
    incEdgeIdx = axisIndex;
    inc[0] = vertsA[axisIndex];
    inc[1] = vertsA[(axisIndex + 1) % 4];
  }
  
  m.normal = Rot2.Transpose(normal); //Worldspace normal
  
  Vec2 ref0Point = Rot2.Transpose(ref[0]);
  
  for(int i = 0; i < 2; i++)
  {
    Vec2 clipPoint = ClipSegment(inc[1 - i], inc[i], ref[i], normal);
  
    penetration = normal * (ref[0] - clipPoint);
  
    if(penetration <= 0)
    {
      Rot2.SetTranspose(clipPoint);
      m.contacts[numContacts].penetration = penetration;
      m.contacts[numContacts].position = b1->body->position + clipPoint + m.normal * penetration;
      m.contacts[numContacts].id = FeatureID(refEdgeIdx, incEdgeIdx, i);
  	  
      // Post-Projection Position
      m.postPosition.oldPointA[numContacts] = b1->body->position + clipPoint - m.A->position;
      m.postPosition.oldPointB[numContacts] = b1->body->position + ref0Point - m.B->position;
      numContacts++;
    }
  }
  // Post-Projection Position
  m.postPosition.oldOrientationA = m.A->orientation;
  m.postPosition.oldOrientationB = m.B->orientation;
  m.numContacts = numContacts;
}
