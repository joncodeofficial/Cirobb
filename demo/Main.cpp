/*************************************************************************
 * Copyright (c) 2019-2025 Jonathan Pe�a
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Jonathan Pe�a makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 **************************************************************************/

#include "../glad/glad.h"
#include "../cirobb/Scene.h"
#include "Render.h"
namespace
{
  GLFWwindow *mainWindow = NULL;

  int width = 900;
  int height = 700;
  real zoom = 10;

  int numScene = 0;
  int lastNumberScene = 0;
  real viewScale = 0;
  bool pause = false;
  real timeStep = 1.0f / 60.0f;
  Vec2 gravity(0.0f, -10.0f);

  real mouseX = 0, mouseY = 0;

  Scene scene(gravity, 10, 4);

  Circle cPrincipal(2);
  OBB bPrincipal(4, 4);

  RigidBody *sPrincipal = new RigidBody(cPrincipal, Vec2(0, 0), 0.0f);

  Circle cShape(2.0f);
  OBB bShape(9.0f, 6.0f);
  RigidBody *cLocal = new RigidBody(cShape, Vec2(-5, 0), 0.0f);
  RigidBody *bLocal = new RigidBody(bShape, Vec2(5, 0), -PI / 15.0f);

  OBB ww(1, 10);
  RigidBody *w1 = new RigidBody(ww, Vec2(5, 0), 0);
  RigidBody *w2 = new RigidBody(ww, Vec2(-5, 0), 0);
  RigidBody *w3 = new RigidBody(ww, Vec2(0, 5), PI * 0.5f);
  RigidBody *w4 = new RigidBody(ww, Vec2(0, -5), PI * 0.5f);
}

char *SceneStrings[] =
    {
        "Collision Detection",
        "Low Friction",
        "Stacking Boxes & Circles",
        "Pyramid",
        "Buildings",
        "Rotary Box",
        "Improvisational Mode"};

char *PositionStrings[] =
    {
        "None",
        "Baumgarte Stabilization",
        "Non-Linear-Gauss-Seidel"};

static void glfwErrorCallback(int error, const char *description)
{
  printf("GLFW error %d: %s\n", error, description);
}

static void DetectionCollision(RigidBody *p)
{
  DrawCircle(cLocal->shape);
  DrawObb(bLocal->shape);
  DrawShape(sPrincipal->shape);

  Manifold m1(p, cLocal);
  Manifold m2(p, bLocal);

  if (m1.numContacts == 1)
  {
    DrawPoint(m1.contacts[0].position, 2, 4);
    Vec2 value = m1.normal * m1.A->shape->radius;
    DrawLine(m1.A->position + value, m1.A->position + value + m1.normal * m1.contacts[0].penetration);
  }

  if (m2.numContacts > 0)
  {
    for (int i = 0; i < m2.numContacts; i++)
    {
      DrawPoint(m2.contacts[i].position, 2, 4);
      DrawLine(m2.contacts[i].position, m2.contacts[i].position - m2.normal * m2.contacts[i].penetration);
    }
  }
}

static void LowFriction(void)
{
  OBB w1(22.0f, 0.8f);
  RigidBody *b1 = new RigidBody(w1, Vec2(0.0f, -7.5f), 0.0f);
  b1->Static();

  OBB w2(10.0, 0.6f);

  RigidBody *b2 = new RigidBody(w2, Vec2(-5.0f, 3.5f), -15 * RAD);
  b2->Static();

  RigidBody *b3 = new RigidBody(w2, Vec2(5.0f, 0.0f), 15 * RAD);
  b3->Static();

  RigidBody *b4 = new RigidBody(w2, Vec2(-5.0f, -3.5f), -15 * RAD);
  b4->Static();

  Circle c1(0.5f);
  RigidBody *b5 = new RigidBody(c1, Vec2(8.72f, 5.32f), 0.0f);
  b5->Dynamic(1.0f);
  b5->angularDamping = 0.1f;

  OBB w3(1.0f, 1.0f);
  RigidBody *b6 = new RigidBody(w3, Vec2(-9.7f, 5.7f), -15 * RAD);
  b6->Dynamic(1.0f);
  b6->friction = 0.01f;

  scene.Add(b1);
  scene.Add(b2);
  scene.Add(b3);
  scene.Add(b4);
  scene.Add(b5);
  scene.Add(b6);
}

static void Stacking(void)
{
  OBB w1(22.0f, 0.8f);
  RigidBody *b1 = new RigidBody(w1, Vec2(0.0f, -7.5f), 0.0f);
  b1->Static();
  scene.Add(b1);

  for (int i = 0; i < 10; i++)
  {
    Circle cir(0.5f);
    OBB obb(1.0f, 1.0f);

    real x = -3.5f;
    real y = i * 1.0f - 6.5f;

    RigidBody *b2 = new RigidBody(obb, Vec2(x, y), 0.0f);
    b2->Dynamic(1.0f);

    RigidBody *b3 = new RigidBody(cir, Vec2(0, y), 0.0f);
    b3->Dynamic(1.0f);

    RigidBody *b4 = nullptr;

    if (i & 1)
    {
      b4 = new RigidBody(obb, Vec2(-x, y), 0.0f);
    }
    else
      b4 = new RigidBody(cir, Vec2(-x, y), 0.0f);

    b4->Dynamic(1.0f);

    scene.Add(b2);
    scene.Add(b3);
    scene.Add(b4);
  }
}

static void Pyramid(void)
{
  OBB w1(22.0f, 0.8f);
  RigidBody *b1 = new RigidBody(w1, Vec2(0, -7.5), 0.0f);
  b1->Static();
  scene.Add(b1);

  int lv = 10;
  real wh = 1.0f;

  for (int i = 0; i < lv; i++)
  {
    for (int j = 0; j < lv - i; j++)
    {
      real x = (j * 2 + i - lv) * wh * 0.5f;
      real y = (i + 0.5f) * wh - 6.5f;

      OBB obb(wh, wh);
      RigidBody *b2 = new RigidBody(obb, Vec2(x, y), 0);
      b2->Dynamic(1.0f);
      scene.Add(b2);
    }
  }
}

static void AngryBirds(void)
{
  OBB w1(22.0f, 0.8f);
  RigidBody *b1 = new RigidBody(w1, Vec2(0, -7.5), 0.0f);
  b1->Static();

  OBB w2(2.0f, 0.6f);
  OBB w3(4.3f, 0.4f);

  RigidBody *b2 = new RigidBody(w2, Vec2(-3.0f, -6.43f), 20 * RAD);
  b2->Static();

  RigidBody *b4 = new RigidBody(w2, Vec2(3.0f, -6.43f), -20 * RAD);
  b4->Static();

  RigidBody *b3 = new RigidBody(w3, Vec2(0, -6.0f), 0.0f);
  b3->Static();

  scene.Add(b1);
  scene.Add(b2);
  scene.Add(b3);
  scene.Add(b4);

  int row = 4, col = 3;

  for (int i = 0; i < col; i++) // Columns
  {
    for (int j = 0; j < row; j++) // Rows
    {
      OBB wo(0.2f, 1.2f);

      real x = 1.5f * i;
      real y = 1.4f * j - 5.2f;

      RigidBody *b5 = new RigidBody(wo, Vec2(x - 2.1f, y), 0.0f);
      b5->Dynamic(1.0f);
      scene.Add(b5);

      RigidBody *b6 = new RigidBody(wo, Vec2(x - 1.1f, y), 0.0f);
      b6->Dynamic(1.0);
      scene.Add(b6);

      RigidBody *b7 = new RigidBody(wo, Vec2(x - 1.6f, y + 0.7f), PI * 0.5f); // 90 degrees
      b7->Dynamic(1.0f);
      scene.Add(b7);
    }
  }
}

// this physics engine does not support rotation away from the center of mass.
// So this is a special scene that was built to roughly rotate 4 OBBs away from the local center of mass.
static void RotaryBox(void)
{
  real angularVelocity = RAD * 0.25f / timeStep; // Angular Velocity

  Mat2 rot1(w1->orientation);
  Mat2 rot2(w3->orientation);

  w1->angularVelocity = w2->angularVelocity = angularVelocity;
  w3->angularVelocity = w4->angularVelocity = angularVelocity;

  w1->velocity = rot1.Rotate(Cross(Vec2(5.0f, 0), -angularVelocity));
  w2->velocity = rot1.Rotate(Cross(Vec2(5.0f, 0), angularVelocity));
  w3->velocity = rot2.Rotate(Cross(Vec2(5.0f, 0), -angularVelocity));
  w4->velocity = rot2.Rotate(Cross(Vec2(5.0f, 0), angularVelocity));
}

static void FreeStyle(void)
{
  OBB w1(22.0f, 0.8f);
  RigidBody *b1 = new RigidBody(w1, Vec2(0, -7.5), 0);
  b1->Static();

  OBB w2(1.0f, 11.0f);

  RigidBody *b2 = new RigidBody(w2, Vec2(-12.0f, -2.5f), 10 * RAD);
  b2->Static();

  RigidBody *b3 = new RigidBody(w2, Vec2(12.0f, -2.5f), -10 * RAD);
  b3->Static();

  scene.Add(b1);
  scene.Add(b2);
  scene.Add(b3);
}

static void (*TestScene[])(void) = {LowFriction, Stacking, Pyramid, AngryBirds, RotaryBox, FreeStyle};

static void Start(void)
{
  if (numScene != lastNumberScene)
  {
    lastNumberScene = numScene;

    scene.Clear();

    sPrincipal->Dynamic(0.5f);

    scene.Add(sPrincipal);

    printf("%d", numScene);

    if (numScene == 5) // Special Scene 6 because we dont support distance constraint
    {
      w1->Static();
      w2->Static();
      w3->Static();
      w4->Static();

      scene.Add(w1);
      scene.Add(w2);
      scene.Add(w3);
      scene.Add(w4);
    }

    if (numScene > 0)
      TestScene[numScene - 1]();
  }
}

static void Keyboard(GLFWwindow *window, int key, int scancode, int action, int mods)
{

  if (action != GLFW_PRESS)
    return;

  switch (key)
  {
  case GLFW_KEY_ESCAPE:
    glfwSetWindowShouldClose(mainWindow, GL_TRUE);
    break;

  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
  case '6':
  case '7':
    numScene = (int(key) - 48) - 1;
    Start();
    break;

  case GLFW_KEY_Q:
    sPrincipal->orientation += PI / 72;
    break;

  case GLFW_KEY_E:
    sPrincipal->orientation -= PI / 72;
    break;

  case GLFW_KEY_B:
  {
    OBB temp(Vec2(1.0f, 1.0f));
    RigidBody *b = new RigidBody(temp, Vec2(mouseX, mouseY), 0);
    b->Dynamic(1.0f);
    scene.Add(b);
  }
  break;

  case GLFW_KEY_C:
  {
    Circle temp(0.5f);
    RigidBody *b = new RigidBody(temp, Vec2(mouseX, mouseY), 0.0f);
    b->Dynamic(1.0f);
    scene.Add(b);
  }
  break;

  case GLFW_KEY_P:
    pause = !pause;
    break;

  case GLFW_KEY_W:
    if (sPrincipal->shape->type == circle)
    {
      sPrincipal->shape = &bPrincipal;
    }
    else
      sPrincipal->shape = &cPrincipal;

    sPrincipal->shape->body = sPrincipal;
    sPrincipal->Dynamic(0.5f);
    break;

  case GLFW_KEY_SPACE:
    Scene::CorrectionType = Scene::CorrectionType > 1 ? 0 : Scene::CorrectionType + 1;
    break;
  }
}

static void MouseMotion(GLFWwindow *window, double x, double y)
{
  mouseX = ((real)x - width * 0.5) * viewScale;
  mouseY = ((real)y - height * 0.5) * -viewScale;

  if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
  {
    sPrincipal->position.Set(mouseX, mouseY);
    sPrincipal->velocity.SetZero();
    sPrincipal->angularVelocity = 0.0f;
  }
}

static void Reshape(GLFWwindow *, int w, int h)
{
  width = w, height = h > 0 ? h : 1;

  int fbW, fbH;
  glfwGetFramebufferSize(mainWindow, &fbW, &fbH);
  glViewport(0, 0, fbW, fbH);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  real aspect = real(width) / real(height);

  if (width >= height)
  {
    viewScale = zoom / (height * 0.5);
    glOrtho(-zoom * aspect, zoom * aspect, -zoom, zoom, -1.0, 1.0);
  }
  else
  {
    viewScale = zoom / (width * 0.5);
    glOrtho(-zoom, zoom, -zoom / aspect, zoom / aspect, -1.0, 1.0);
  }
}

int main(int args, char **argv)
{
  glfwSetErrorCallback(glfwErrorCallback);

  if (glfwInit() == 0)
  {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return -1;
  }

  mainWindow = glfwCreateWindow(width, height, "Cirobb 1.1.10", NULL, NULL);

  if (mainWindow == NULL)
  {
    fprintf(stderr, "Failed to open GLFW mainWindow.\n");
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(mainWindow);

  // Load OpenGL functions using glad
  int gladStatus = gladLoadGL();

  if (gladStatus == 0)
  {
    fprintf(stderr, "Failed to load OpenGL.\n");
    glfwTerminate();
    return -1;
  }

  glfwSwapInterval(1);
  glfwSetWindowSizeCallback(mainWindow, Reshape);
  glfwSetKeyCallback(mainWindow, Keyboard);
  glfwSetCursorPosCallback(mainWindow, MouseMotion);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(mainWindow, true);
  ImGui_ImplOpenGL2_Init();

  Reshape(mainWindow, width, height);

  Start(); // Start Scene

  while (!glfwWindowShouldClose(mainWindow))
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.2f, 0.2f, 0.2f, 0.2f));
    ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
    ImGui::PopStyleColor();
    ImGui::End();

    // Fijar posici�n en esquina superior izquierda
    ImGui::SetNextWindowPos(ImVec2(8.0f, 8.0f), ImGuiCond_Always);

    // Crear ventana de ImGui para los controles
    ImGui::Begin("Controls", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoMove |     // Previene que el usuario mueva la ventana
                     ImGuiWindowFlags_NoTitleBar); // Elimina la barra de t�tulo para un look m�s limpio

    ImGui::Combo("Scenes", &numScene, SceneStrings, IM_ARRAYSIZE(SceneStrings));
    ImGui::Combo("Position", &Scene::CorrectionType, PositionStrings, IM_ARRAYSIZE(PositionStrings));

    // Checkbox para pause
    ImGui::Checkbox("(P)ause", &pause);

    ImGui::End();

    if (numScene == 0)
    {
      DetectionCollision(sPrincipal);
    }

    if (numScene != lastNumberScene)
      Start();

    else
    {
      if (!pause)
        scene.Step(timeStep);

      if (numScene == 5)
        RotaryBox();

      for (RigidBody *temp : scene.bodies)
        DrawShape(temp->shape);

      for (auto temp = scene.manifolds.begin(); temp != scene.manifolds.end(); temp++)
      {
        for (int i = 0; i < temp->second.numContacts; i++)
        {
          DrawPoint(temp->second.contacts[i].position, 2, 3);
          DrawPoint(temp->second.contacts[i].warmPoint, 1, 6);
        }
      }
    }

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glfwPollEvents();
    glfwSwapBuffers(mainWindow);
  }

  glfwTerminate();
  return 0;
}
