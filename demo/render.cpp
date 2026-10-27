#include "render.h"


void DrawShape(const Shape* shape)
{
	switch (shape->type)
	{
		case circle: DrawCircle(shape); break;

		case obb: DrawObb(shape); break;
	}
}



void DrawCircle(const Shape* shape)
{
	const RigidBody* body = shape->body;

	const int  countVerts = 40; // Number of Vertices
	const real valueVerts = PI * 2 / countVerts;
	Vec2 verts[countVerts + 1];
 
	const real radius = shape->radius;

	for(int i = 0; i <= countVerts; i++) // RADIAN [0, 2PI] --> DEGREE [0, 360]
	{
	  verts[i].x = radius * cosf(valueVerts * i); 
	  verts[i].y = radius * sinf(valueVerts * i); 
	}
	
	glPushMatrix();
	glTranslatef(body->position.x, body->position.y, 0.0f);	
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_COLOR, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(0.4f, 0.4f, 0.4f, 0.4f);
	
	glBegin(GL_TRIANGLE_FAN);
	for(int i = 0; i <= countVerts; i++)
	{
	  glVertex2f(verts[i].x, verts[i].y);
	}
	glEnd();
	
	glDisable(GL_BLEND);
	
	glColor3f(0.65f, 0.65f, 0.65f);
	
	glBegin(GL_LINE_STRIP);
	for(int i = 0; i <= countVerts; i++)
	{
	  glVertex2f(verts[i].x, verts[i].y);
	}
	glEnd();
	
	glBegin(GL_LINES);
	glVertex2f(0, 0);
	glVertex2f(radius * cosf(body->orientation), radius * sinf(body->orientation));
	glEnd();
	
	glColor3f(1.0f, 1.0f, 1.0f);
	glPopMatrix();
}



void DrawObb(const Shape* shape)
{
	const RigidBody* body = shape->body;

	const Vec2 width = shape->width;

	glPushMatrix();
	glTranslatef(body->position.x, body->position.y, 0.0f);
	Mat2 Rot(body->orientation);
	Vec2 p1 = Rot * width * 0.5f;
	Vec2 p2 = Rot * Vec2(width.x, -width.y) * 0.5f;	
		
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_COLOR, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(0.4f, 0.4f, 0.4f, 0.4f);
	glBegin(GL_TRIANGLE_FAN);
	glVertex2f( p1.x, p1.y);
	glVertex2f( p2.x, p2.y);
	glVertex2f(-p1.x,-p1.y);
	glVertex2f(-p2.x,-p2.y);
	glEnd();
	glDisable(GL_BLEND);
		
	glColor3f(0.65f, 0.65f, 0.65f);
	glBegin(GL_LINE_LOOP);
	glVertex2f( p1.x, p1.y);
	glVertex2f( p2.x, p2.y);
	glVertex2f(-p1.x,-p1.y);
	glVertex2f(-p2.x,-p2.y);
	glEnd();
	glColor3f(1.0f, 1.0f, 1.0f);
	glPopMatrix();
}



void DrawPoint(const Vec2& p, const int& cases, const real& size)
{
  switch (cases)
  {
    case 0: glColor3f(1.0f, 0.0f, 0.0f); break;
    
		case 1: glColor3f(0.0f, 0.9f, 0.0f); break;
    
		case 2: glColor3f(0.0f, 0.5f, 1.0f); break;
  }
  glPointSize(size);	
  glBegin(GL_POINTS);
  glVertex2f(p.x, p.y);
  glEnd();
  glColor3f(1.0f, 1.0f, 1.0f);
}



void DrawLine(const Vec2& p1, const Vec2& p2)
{
  glLineWidth(1.0);
  glColor3f(1.0f, 0.0, 0.0f);
  glBegin(GL_LINES);
  glVertex2f(p1.x, p1.y);
  glVertex2f(p2.x, p2.y);
  glEnd();
  glColor3f(1.0f, 1.0f, 1.0f);
  glLineWidth(1);
}