#pragma once
#include <string>
#include <math.h>
#include <iostream>

class Vector3 {
public:
  double x, y, z;

	Vector3();
	Vector3(double, double, double);
	~Vector3();

	Vector3 operator+(const Vector3&);
	Vector3 operator-(const Vector3&);

	void negate();
	void scale(double);
	double dot(const Vector3&, const Vector3&);
	Vector3 cross(const Vector3&, const Vector3&);
	double length();
	void normalize();
	void print(std::string);
};

