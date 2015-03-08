#pragma once

#include <iostream>
#include <string>

class Vector4d {
public:
  double x, y, z, w;
	Vector4d();
	Vector4d(double, double, double, double);
	Vector4d operator+(const Vector4d&);
	Vector4d operator-(const Vector4d&);
	~Vector4d();

	void dehomogenize();
	void print(std::string);
};

