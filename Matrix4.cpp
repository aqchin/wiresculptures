#include "Matrix4.h"

Matrix4d::Matrix4d()
{
  for (int i=0; i<4; ++i)
  {
    for (int j=0; j<4; ++j)
    {
      m[i][j] = 0;
    }
  }
} 

Matrix4d& Matrix4d::operator=(const Matrix4d& m2)
{
  if (this != &m2)
  {
    for (int i=0; i<4; ++i)
    {
      for (int j=0; j<4; ++j)
      {
        m[i][j] = m2.m[i][j];
      }
    }
  }
  return *this;
}

// return pointer to matrix elements
double* Matrix4d::getPointer()
{
  return &m[0][0];
}

// set matrix to identity matrix
void Matrix4d::identity()
{
  for (int i=0; i<4; ++i)
  {
    for (int j=0; j<4; ++j)
    {
      if (i==j) 
        m[i][j] = 1.0; 
      else 
        m[i][j] = 0.0;
    }
  }
}

// transpose the matrix (mirror at diagonal)
void Matrix4d::transpose()
{
  Matrix4d temp;
  for (int i=0; i<4; ++i)
  {
    for (int j=0; j<4; ++j)
    {
      temp.m[j][i] = m[i][j];
    }
  }
  *this = temp;  // copy temporary values to this matrix
}

// Creates a rotation matrix which rotates about the y axis.
// angle is expected in degrees
void Matrix4d::makeRotateY(double angle)
{
  angle = angle / 180.0 * M_PI;  // convert from degrees to radians
  identity();
	m[0][0] = cos(angle);
	m[0][2] = sin(angle);
	m[2][0] = -sin(angle);
	m[2][2] = cos(angle);
}

static Matrix4d *m1 = new Matrix4d();
Matrix4d& Matrix4d::operator*(const Matrix4d& m) {
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			double sum = 0.0;
			for (int i = 0; i < 4; i++) {
				sum += this->m[i][c] * m.m[r][i];
			}
			m1->m[r][c] = sum;
		}
	}
	return *m1;
}

Vector4d& Matrix4d::operator*(const Vector4d& v) {
	Vector4d v1 = Vector4d(
		v.x*this->m[0][0] + v.y*this->m[0][1] + 
			v.z*this->m[0][2] + v.w*this->m[0][3],
		v.x*this->m[1][0] + v.y*this->m[1][1] +
			v.z*this->m[1][2] + v.w*this->m[1][3],
		v.x*this->m[2][0] + v.y*this->m[2][1] +
			v.z*this->m[2][2] + v.w*this->m[2][3],
		v.x*this->m[3][0] + v.y*this->m[3][1] +
			v.z*this->m[3][2] + v.w*this->m[3][3]);
	return v1;
}

void Matrix4d::makeRotateX(double angle) {
	angle = angle / 180.0 * M_PI;  // convert from degrees to radians
	identity();
	m[1][1] = cos(angle);
	m[1][2] = -sin(angle);
	m[2][1] = sin(angle);
	m[2][2] = cos(angle);
}

void Matrix4d::makeRotateZ(double angle) {
	angle = angle / 180.0 * M_PI;  // convert from degrees to radians
	identity();
	m[0][0] = cos(angle);
	m[0][1] = -sin(angle);
	m[1][0] = sin(angle);
	m[1][1] = cos(angle);
}

void Matrix4d::makeRotate(double a, const Vector3& v) {
	a = a / 180.0 * M_PI;
	double t = 1 - cos(a);
	double c = cos(a);
	double s = sin(a);
	double x = v.x;
	double y = v.y;
	double z = v.z;

	identity();
	m[0][0] = t * x * x + c;
	m[0][1] = t * x * y - s * z;
	m[0][2] = t * x * z + s * y;
	m[1][0] = t * x * y + s * z;
	m[1][1] = t * y * y + c;
	m[1][2] = t * y * z - s * x;
	m[2][0] = t * x * z - s * y;
	m[2][1] = t * y * z + s * x;
	m[2][2] = t * z * z + c;
}

void Matrix4d::makeScale(double sx, double sy, double sz) {
	identity();
	this->m[0][0] = sx;
	this->m[1][1] = sy;
	this->m[2][2] = sz;
}

void Matrix4d::makeTranslate(double tx, double ty, double tz) {
	identity();
	this->m[0][3] = tx;
	this->m[1][3] = ty;
	this->m[2][3] = tz;
}

void Matrix4d::print(std::string s) {
	std::cout << s << std::endl;
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			std::cout << this->m[r][c] << " ";
		}
		std::cout << std::endl;
	}
}

Vector3& Matrix4d::getV3() {
	Vector3 *v = new Vector3(m[0][3], m[1][3], m[2][3]);
	return *v;
}

void Matrix4d::setM4cm(Vector3 x, Vector3 y, Vector3 z, Vector3 w) {
	identity();
	this->m[0][0] = x.x;
	this->m[1][0] = y.x;
	this->m[2][0] = z.x;
	this->m[0][1] = x.y;
	this->m[1][1] = y.y;
	this->m[2][1] = z.y;
	this->m[0][2] = x.z;
	this->m[1][2] = y.z;
	this->m[2][2] = z.z;
	this->m[0][3] = -x.dot(x, w);
	this->m[1][3] = -y.dot(y, w);
	this->m[2][3] = -z.dot(z, w);
	this->transpose(); // hue
}