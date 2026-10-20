#ifndef RENDER_H
#define RENDER_H

#include "imgui/imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#include "../cirobb/Shape.h"
#include "GLFW/glfw3.h"

void DrawText(const int& x, const int& y, char* string);

void DrawShape(const Shape* shape);

void DrawCircle(const Shape* shape);

void DrawObb(const Shape* shape);

void DrawPoint(const Vec2& p, const int& cases, const real& size);

void DrawLine(const Vec2& p1, const Vec2& p2);

#endif