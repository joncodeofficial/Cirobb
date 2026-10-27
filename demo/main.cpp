/*************************************************************************
 * Copyright (c) 2019-2025 Jonathan Peña
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Jonathan Peña makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 **************************************************************************/

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#else
#include "glad/glad.h"
#include "imgui/imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#endif
#include "../cirobb/scene.h"
#include "render.h"

namespace {
GLFWwindow *mainWindow = nullptr;

int width = 1200;
int height = 720;
real zoom = 10;
real viewScale = 0;

bool pause = false;
real timeStep = 1.0f / 60.0f;
Vec2 gravity(0.0f, -10.0f);

real mouseX = 0, mouseY = 0;

Scene scene(gravity, 10, 5);

// Web only: filters the duplicated mouse events that touch devices produce
bool canCreateBody = true;
double lastTouchTime = 0.0;
const double TOUCH_COOLDOWN = 0.2;
} // namespace

const char *PositionStrings[] = {"None", "Baumgarte Stabilization", "Non-Linear-Gauss-Seidel"};

#ifdef __EMSCRIPTEN__
// Called from the web page (web/main.js)
extern "C" {
bool setPause(bool value) {
  pause = value;
  return pause;
}

int setCorrectionType(int value) {
  Scene::CorrectionType = value;
  return Scene::CorrectionType;
}

bool setTouchDevice(bool value) {
  canCreateBody = value;
  return canCreateBody;
}
}
#endif

static void glfwErrorCallback(int error, const char *description) {
  printf("GLFW error %d: %s\n", error, description);
}

static void Start() {
  Scene::CorrectionType = NGS;

  scene.Clear();

  OBB ground(22.0f, 0.8f);
  RigidBody *b1 = new RigidBody(ground, Vec2(0, -8.5f), 0.0f);
  b1->Static();
  scene.Add(b1);

  OBB wall(16.5f, 0.8f);
  RigidBody *b2 = new RigidBody(wall, Vec2(12, -0.8f), 85 * RAD);
  b2->Static();
  scene.Add(b2);

  RigidBody *b3 = new RigidBody(wall, Vec2(-12, -0.8f), -85 * RAD);
  b3->Static();
  scene.Add(b3);
}

// Left button: box, right button: circle
static void CreateBody(int button) {
  RigidBody *b;

  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    OBB temp(Vec2(1.5f, 1.5f));
    b = new RigidBody(temp, Vec2(mouseX, mouseY), 0.0f);
  } else {
    Circle temp(0.75f);
    b = new RigidBody(temp, Vec2(mouseX, mouseY), 0.0f);
  }

  b->Dynamic(1.0f);
  scene.Add(b);
}

static void Keyboard(GLFWwindow *window, int key, int scancode, int action, int mods) {
  if (action != GLFW_PRESS) return;

  switch (key) {
  case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(mainWindow, GL_TRUE); break;

  case GLFW_KEY_P: pause = !pause; break;

  case GLFW_KEY_SPACE:
    Scene::CorrectionType = Scene::CorrectionType > 1 ? 0 : Scene::CorrectionType + 1;
    break;
  default: break;
  }
}

static void MouseMotion(GLFWwindow *window, double x, double y) {
  mouseX = ((real)x - static_cast<real>(width) * 0.5f) * viewScale;
  mouseY = ((real)y - static_cast<real>(height) * 0.5f) * -viewScale;
}

static void MouseButton(GLFWwindow *window, int button, int action, int mods) {
  if (button != GLFW_MOUSE_BUTTON_LEFT && button != GLFW_MOUSE_BUTTON_RIGHT) return;

#ifdef __EMSCRIPTEN__
  double currentTime = glfwGetTime();

  if (action == GLFW_PRESS) {
    if (currentTime - lastTouchTime < TOUCH_COOLDOWN) {
      canCreateBody = true;
      return;
    }

    double x, y;
    glfwGetCursorPos(window, &x, &y);
    MouseMotion(window, x, y);

    lastTouchTime = currentTime;
  } else if (action == GLFW_RELEASE && canCreateBody) {
    canCreateBody = false;
    CreateBody(button);
  }
#else
  if (ImGui::GetIO().WantCaptureMouse) return;

  if (action == GLFW_RELEASE) CreateBody(button);
#endif
}

static void Reshape(GLFWwindow *_, int w, int h) {
  width = w, height = h > 0 ? h : 1;

  int fbW, fbH;
  glfwGetFramebufferSize(mainWindow, &fbW, &fbH);
  glViewport(0, 0, fbW, fbH);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  real aspect = real(width) / real(height);

  if (width >= height) {
    viewScale = zoom / static_cast<real>(height * 0.5);
    glOrtho(-zoom * aspect, zoom * aspect, -zoom, zoom, -1.0, 1.0);
  } else {
    viewScale = zoom / static_cast<real>(width * 0.5);
    glOrtho(-zoom, zoom, -zoom / aspect, zoom / aspect, -1.0, 1.0);
  }
}

static void Frame() {
#ifdef __EMSCRIPTEN__
  glfwGetFramebufferSize(mainWindow, &width, &height);
#endif

  glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

#ifndef __EMSCRIPTEN__
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  ImGui::SetNextWindowPos(ImVec2(8.0f, 8.0f), ImGuiCond_Always);
  ImGui::Begin("Controls", nullptr,
               ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoTitleBar);

  ImGui::Combo("Position", &Scene::CorrectionType, PositionStrings, IM_ARRAYSIZE(PositionStrings));
  ImGui::Checkbox("(P)ause", &pause);
  ImGui::Text("Left click: box");
  ImGui::Text("Right click: circle");

  ImGui::End();
#endif

  if (!pause) scene.Step(timeStep);

  for (RigidBody *body : scene.bodies) DrawShape(body->shape);

#ifndef __EMSCRIPTEN__
  ImGui::Render();
  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
#endif

  glfwSwapBuffers(mainWindow);
  glfwPollEvents();
}

int main(int args, char **argv) {
  glfwSetErrorCallback(glfwErrorCallback);

  if (glfwInit() == 0) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return -1;
  }

  mainWindow = glfwCreateWindow(width, height, "Cirobb 1.1.12", NULL, NULL);

  if (mainWindow == NULL) {
    fprintf(stderr, "Failed to open GLFW mainWindow.\n");
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(mainWindow);

#ifndef __EMSCRIPTEN__
  // Load OpenGL functions using glad
  if (gladLoadGL() == 0) {
    fprintf(stderr, "Failed to load OpenGL.\n");
    glfwTerminate();
    return -1;
  }

  glfwSwapInterval(1);
#endif

  glfwSetWindowSizeCallback(mainWindow, Reshape);
  glfwSetKeyCallback(mainWindow, Keyboard);
  glfwSetCursorPosCallback(mainWindow, MouseMotion);
  glfwSetMouseButtonCallback(mainWindow, MouseButton);

#ifndef __EMSCRIPTEN__
  // Installed after our callbacks so ImGui chains them
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(mainWindow, true);
  ImGui_ImplOpenGL2_Init();
#endif

  Reshape(mainWindow, width, height);

  Start();

#ifdef __EMSCRIPTEN__
  emscripten_set_main_loop(Frame, 0, true);
#else
  while (glfwWindowShouldClose(mainWindow) == 0) Frame();

  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
#endif

  glfwTerminate();
  return 0;
}
