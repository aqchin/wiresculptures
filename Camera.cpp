#include "Camera.h"

Camera::Camera()
{
	this->e = (Vector3());
	this->d = (Vector3());
	this->up = (Vector3());

	this->C = Matrix4d();
	C.identity();
	C.setM4cm(Vector3(), Vector3(), Vector3(), e);
}

Camera::Camera(Vector3 e, Vector3 d, Vector3 up) {
	this->e = e;
	this->d = d;
	this->up = up;
	
	Vector3 z = e - d; // z = (e - d) / ||e - d||
	z.normalize();

	Vector3 x = x.cross(up, z); // z = (up X z) / ||up X z||
	x.normalize();

	Vector3 y = y.cross(z, x); // y = (z X x) / ||z X x||
	y.normalize();

	this->C = (Matrix4d());
	this->C.setM4cm(x, y, z, e);
}

Camera::~Camera()
{
}

GLdouble* Camera::getGLMatrix() {
	return (this->C.getPointer());
}

void Camera::update() {
  Vector3 z = e - d; // z = (e - d) / ||e - d||
  z.normalize();

  Vector3 x = x.cross(up, z); // z = (up X z) / ||up X z||
  x.normalize();

  Vector3 y = y.cross(z, x); // y = (z X x) / ||z X x||
  y.normalize();

  //this->C = (*new Matrix4());
  this->C.setM4cm(x, y, z, e);

}

void Camera::move(char c) {
  Matrix4d m;
  Vector3 dv, dv2;

  Vector4d v4e = Vector4d(e.x, e.y, e.z, 1.0);
  Vector4d upv = Vector4d(up.x, up.y, up.z, 1.0);

  switch(c) {
  case 'w':
    dv = d - e;
    dv2 = dv.cross(up, dv);
    dv2.normalize();
    m.makeRotate(1, dv2);
    v4e = m * v4e;
    upv = m * upv;
    e = Vector3(v4e.x, v4e.y, v4e.z);
    up = Vector3(upv.x, upv.y, upv.z);
    break;
  case 'a':
    m.makeRotate(-1, up);
    v4e = m * v4e;
    e = Vector3(v4e.x, v4e.y, v4e.z);
    break;
  case 's':
    dv = d - e;
    dv2 = dv.cross(up, dv);
    dv2.normalize();
    m.makeRotate(-1, dv2);
    v4e = m * v4e;
    upv = m * upv;
    e = Vector3(v4e.x, v4e.y, v4e.z);
    up = Vector3(upv.x, upv.y, upv.z);
    break;
  case 'd':
    m.makeRotate(1, up);
    v4e = m * v4e;
    e = Vector3(v4e.x, v4e.y, v4e.z);
    break;
  }
  update();
}