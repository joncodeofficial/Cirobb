#ifndef RENDER_H
#define RENDER_H

#include "../cirobb/shape.h"
#include "GLFW/glfw3.h"

void DrawShape(const Shape* shape);

void DrawCircle(const Shape* shape);

void DrawObb(const Shape* shape);

void DrawPoint(const Vec2& p, const int& cases, const real& size);

void DrawLine(const Vec2& p1, const Vec2& p2);

#endif