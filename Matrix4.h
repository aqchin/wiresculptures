#ifndef _MATRIX4_H_
#define _MATRIX4_H_

#define M_PI 3.14159265358979323846
#include <iostream>
#include <string>
#include <math.h>
#include "Vector3.h"
#include "Vector4.h"

class Matrix4d {
  protected:
    double m[4][4];   // matrix elements; first index is for rows, second for columns (row-major)
    
  public:
    Matrix4d();
    Matrix4d& operator=(const Matrix4d&);
    double* getPointer(); 
    void identity(); 
    void transpose();
    void makeRotateY(double); 

	Matrix4d& operator*(const Matrix4d&);
	Vector4d& operator*(const Vector4d&);
	void makeRotateX(double);
	void makeRotateZ(double);
	void makeRotate(double, const Vector3&);
	void makeScale(double, double, double);
	void makeTranslate(double, double, double);
	void print(std::string);
	Vector3& getV3();
	void setM4cm(Vector3, Vector3, Vector3, Vector3); // manually set matrix column major V3
};

#endif