#include "Vector4.h"

Vector4d::Vector4d() { this->x = this->y = this->z = this->w = 0; }

Vector4d::Vector4d(double x, double y, double z, double w) {
	this->x = x;
	this->y = y;
	this->z = z;
	this->w = w;
}

Vector4d::~Vector4d(){}

Vector4d Vector4d::operator+(const Vector4d& v) {
	Vector4d *v1 = new Vector4d(this->x + v.x, this->y + v.y, this->z + v.z, this->w + v.w);
	return *v1;
}

Vector4d Vector4d::operator-(const Vector4d& v) {
	Vector4d *v1 = new Vector4d(this->x - v.x, this->y - v.y, this->z - v.z, this->w - v.w);
	return *v1;
}

void Vector4d::dehomogenize() {
	x = x / w;
	y = y / w;
	z = z / w;
	w = 1.0;
}

void Vector4d::print(std::string s) {
	//printf("%s \n %d %d %d %d \n", s, x, y, z, w);
	std::cout << s << std::endl << x << " " << y << " " << z << " " << w << std::endl;
}