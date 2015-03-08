#include "Light.h"

Light::Light() { this->light = GL_LIGHT0; }

Light::Light(int l) {
	switch (l) {
	case 0:
		light = GL_LIGHT0;
		break;
	case 1:
		light = GL_LIGHT1;
		break;
	case 2:
		light = GL_LIGHT2;
		break;
	case 3:
		light = GL_LIGHT3;
		break;
	case 4:
		light = GL_LIGHT4;
		break;
	case 5:
		light = GL_LIGHT5;
		break;
	case 6:
		light = GL_LIGHT6;
		break;
	case 7:
		light = GL_LIGHT7;
		break;
	}
}

void Light::on() { glEnable(light); }
void Light::off() { glDisable(light); }

void Light::set(GLenum g, GLfloat* f) { glLightfv(light, g, f); }
void Light::set(GLenum g, GLfloat f) { glLightf(light, g, f); }

void Light::setAmbient(GLfloat* f) { set(GL_AMBIENT, f); }
void Light::setDiffuse(GLfloat* f) { set(GL_DIFFUSE, f); }
void Light::setSpecular(GLfloat* f) { set(GL_SPECULAR, f); }

void Light::setPosition(GLfloat* f) { set(GL_POSITION, f); }
void Light::setSpotDirection(GLfloat* f) { set(GL_SPOT_DIRECTION, f); }

void Light::setSpotExponent(GLfloat f) { set(GL_SPOT_EXPONENT, f); }
void Light::setSpotCutoff(GLfloat f) { set(GL_SPOT_CUTOFF, f); }