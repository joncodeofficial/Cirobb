/*************************************************************************
 * Copyright (c) 2019-2026 Jonathan Peña
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Jonathan Peña makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 **************************************************************************/

#include "collision.h"

// Narrow phase only, discrete collision detection (DCD). The broad phase is brute force O(n^2).
// Thank you very much Dirk Gregorius.

/*
Circle to Circle: there is a collision if |c2 - c1| <= r1 + r2

* Penetration = |c2 - c1| - r1 - r2

* Normal = (c2 - c1) / |c2 - c1|

* ContactPoint = c1 + n * ((r1 - r2 + |c2 - c1|) * 0.5f) --> The point in the middle

Degenerate case: the centers coincide, |c2 - c1| = 0. Handle it manually or ignore it.
*/
void CircleToCircle(Manifold &m, Shape *c1, Shape *c2) {
  Vec2 distance = c2->body->position - c1->body->position;

  real radius = c2->radius + c1->radius;

  real magnitude = distance.SquareMagnitude();

  // Only a collision test is needed, so compare squared values and avoid the square root
  if (magnitude > radius * radius) return;

  magnitude = sqrtf(magnitude);

  m.numContacts = 1;
  m.contacts[0].penetration = magnitude - radius;
  m.normal = magnitude != 0.0f ? distance * (real(1) / magnitude) : Vec2(0, 1);
  m.contacts[0].position =
      c1->body->position + m.normal * ((c1->radius - c2->radius + magnitude) * real(0.5));
}

/*
Circle to OBB: move the circle to the local space of the OBB (an AABB) and find the closest point.
Since a point has radius = 0 the problem reduces to |c2 - c1| <= r1. One contact point is enough.

Case = |c2 - c1| == 0 ? Deep Penetration : Shallow Penetration

* Shallow: the center of the circle is outside the OBB, so the closest point gives the normal,
  penetration and contact point.

* Deep: the center of the circle is inside the OBB, use the axis of minimum penetration.
*/
void CircleToOBB(Manifold &m, Shape *c1, Shape *b1) {
  Mat2 Rot(-b1->body->orientation);

  Vec2 distance = c1->body->position - b1->body->position;

  Vec2 localSpace = Rot * distance;

  Vec2 width = b1->width * real(0.5);

  real dx = absf(localSpace.x) - width.x;

  if (dx > c1->radius) return;

  real dy = absf(localSpace.y) - width.y;

  if (dy > c1->radius) return;

  Vec2 closest; // Closest Point

  // 4 Conditions to Find the Closest Point of the Circle.
  if (-localSpace.x > width.x) closest.x = dx;
  else if (localSpace.x > width.x) closest.x = -dx;
  if (-localSpace.y > width.y) closest.y = dy;
  else if (localSpace.y > width.y) closest.y = -dy;

  real magnitude = closest.SquareMagnitude();

  if (magnitude > c1->radius * c1->radius) return;

  if (magnitude != 0.0f) // Shallow Penetration
  {
    magnitude = sqrtf(magnitude);
    m.normal = closest * (real(1) / magnitude);
  } else // Deep Penetration
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

void OBBToCircle(Manifold &m, Shape *A, Shape *B) {
  m.A = B->body;
  m.B = A->body;
  CircleToOBB(m, B, A);
}

/*
OBB to OBB uses SAT (Separating Axis Theorem) to find the axis of minimum penetration.
Each OBB has 2 pairs of parallel edges, so 4 normals are tested instead of 8.
Known problems: inconsistent points, flip-flop, similar edges, rotation rounding errors.
*/
inline static int8_t FindIncidentEdgeIndex(const real &projection1, const real &projection2) {
  int8_t i = projection1 < 0.0f ? 1 : -1;

  if (projection1 * static_cast<real>(i) > projection2) return 1;
  if (projection1 * static_cast<real>(i) > -projection2) return 3;

  return static_cast<int8_t>(1 - i);
}

inline static Vec2 ClipSegment(Vec2 &A, Vec2 &B, Vec2 &C, const Vec2 &n) {
  Vec2 BA = B - A;
  Vec2 CA = C - A;

  real t = (n.x * CA.y - n.y * CA.x) / (n.x * BA.y - n.y * BA.x);

  return (0 > t || t > 1) ? A : A + BA * t;
}

void OBBToOBB(Manifold &m, Shape *b1, Shape *b2) {
  Mat2 Rot1(b2->body->orientation - b1->body->orientation);

  Mat2 Rot2(-b1->body->orientation);

  Vec2 hA = b1->width * real(0.5);
  Vec2 hB = b2->width * real(0.5);

  Vec2 hB1 = Rot1 * hB;
  Vec2 hB2 = Rot1 * Vec2(-hB.x, hB.y);

  Vec2 localSpace = Rot2.Rotate(b2->body->position - b1->body->position);

  real Bx = hA.x + max(absf(hB1.x), absf(hB2.x));

  if (localSpace.x * localSpace.x > Bx * Bx) return;

  real By = hA.y + max(absf(hB1.y), absf(hB2.y));

  if (localSpace.y * localSpace.y > By * By) return;

  Vec2 normalB = -Rot1.Column0();

  Vec2 vertsA[] = {Vec2(-hA.x, hA.y), -hA, Vec2(hA.x, -hA.y), hA};

  Vec2 vertsB[] = {localSpace + hB2, localSpace - hB1, localSpace - hB2, localSpace + hB1};

  real Ax1 = Dot(normalB, vertsA[0]);
  real Ax2 = Dot(normalB, vertsA[1]);
  real Bx1 = Dot(normalB, vertsB[3]);
  real Bx2 = Dot(normalB, vertsB[1]);

  real Ax = max(absf(Ax1), absf(Ax2));

  if ((-Ax + Bx1) * (Ax + Bx2) > 0) return;

  real Ay1 = Cross(normalB, vertsA[0]);
  real Ay2 = Cross(normalB, vertsA[1]);
  real By1 = Cross(normalB, vertsB[3]);
  real By2 = Cross(normalB, vertsB[1]);

  real Ay = max(absf(Ay1), absf(Ay2));

  if ((-Ay + By1) * (Ay + By2) > 0) return;

  Vec2 axes[] = {Vec2(-1, 0), Vec2(0, -1), normalB, Cross(normalB, -1)};

  real v1[] = {Bx + localSpace.x, By + localSpace.y, Ax - Bx1, Ay - By1};
  real v2[] = {Bx - localSpace.x, By - localSpace.y, Ax + Bx2, Ay + By2};

  int8_t axisIndex = 0, numContacts = 0;

  real penetration = min(v1[0], v2[0]);

  for (int8_t i = 1; i < 4; i++) {
    real minimum = min(v1[i], v2[i]);

    if (minimum <= penetration + 0.007f) {
      axisIndex = i;
      penetration = minimum;
    }
  }

  Vec2 ref[2], inc[2];
  int8_t refEdgeIdx, incEdgeIdx;

  int side = v1[axisIndex] < v2[axisIndex] ? 1 : -1;

  Vec2 normal = axes[axisIndex] * real(side); // Localspace normal

  if (axisIndex < 2) {
    m.A = b2->body;
    m.B = b1->body;
    if (side == -1) axisIndex += 2;
    ref[0] = vertsA[axisIndex];
    ref[1] = vertsA[(axisIndex + 1) % 4];
    refEdgeIdx = axisIndex;
    axisIndex = FindIncidentEdgeIndex(normal * axes[2], normal * axes[3]);
    incEdgeIdx = axisIndex;
    inc[0] = vertsB[axisIndex];
    inc[1] = vertsB[(axisIndex + 1) % 4];
    normal *= -1;
  } else {
    if (side == -1) axisIndex -= 2;
    ref[0] = vertsB[axisIndex];
    ref[1] = vertsB[(axisIndex + 1) % 4];
    refEdgeIdx = axisIndex;
    axisIndex = FindIncidentEdgeIndex(normal.x, normal.y);
    incEdgeIdx = axisIndex;
    inc[0] = vertsA[axisIndex];
    inc[1] = vertsA[(axisIndex + 1) % 4];
  }

  m.normal = Rot2.Transpose(normal); // Worldspace normal

  Vec2 ref0Point = Rot2.Transpose(ref[0]);

  for (int8_t i = 0; i < 2; i++) {
    Vec2 clipPoint = ClipSegment(inc[1 - i], inc[i], ref[i], normal);

    penetration = normal * (ref[0] - clipPoint);

    if (penetration <= 0) {
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
