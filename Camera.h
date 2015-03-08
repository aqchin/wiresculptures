#pragma once

#include "Vector3.h"
#include "Matrix4.h"
#include <GL/freeglut.h>

class Camera
{
public:
  Vector3 e; // Center of projection
  Vector3 d; // Look at
  Vector3 up; // Up-vector
  Matrix4d C; // Internal camera

	Camera();
	Camera(Vector3, Vector3, Vector3);
	~Camera();

	GLdouble* getGLMatrix();
  void update();
  void move(char c);
};