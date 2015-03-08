#pragma once

#include <GL/freeglut.h>

class Light {
private:
	void set(GLenum, GLfloat*);
	void set(GLenum, GLfloat);
public:
	GLenum light;

	Light();
	Light(int);

	void on();
	void off();

	void setAmbient(GLfloat*);
	void setDiffuse(GLfloat*);
	void setSpecular(GLfloat*);

	void setPosition(GLfloat*);
	void setSpotDirection(GLfloat*);
	void setSpotExponent(GLfloat); // 1
	void setSpotCutoff(GLfloat); // 1
};