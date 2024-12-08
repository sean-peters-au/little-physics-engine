#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

double my_sqrt(double d);

class Vector;

class Position
{
	public:
	double x;
	double y;

  Position();
	Position(double x, double y);
  operator Vector() const;

  Position operator+(const Position& p);
  Position operator-(const Position& p);
  Position operator*(const double scalar);
  Position operator/(const double scalar);

  Position& operator+=(const Position& p);
  Position& operator-=(const Position& p);
  Position& operator*=(const double scalar);
  Position& operator/=(const double scalar);

	void add(Position& v);
	void subtract(Position& v);
	void multiply(Position& v);
	void divide(Position& v);

	double dist(Position& p);
};

class Vector
{
	public:
	double x;
	double y;

  Vector();
	Vector(double x, double y);
	Vector(Position& p);
  operator Position() const;

  Vector operator+(const Vector& v);
  Vector operator-(const Vector& v);
  Vector operator*(const double scalar);
  Vector operator/(const double scalar);

  Vector& operator+=(const Vector& v);
  Vector& operator-=(const Vector& v);
  Vector& operator*=(const double scalar);
  Vector& operator/=(const double scalar);

	void add(Vector& v);
	void subtract(Vector& v);
	void multiply(double scalar);
	void divide(double scalar);

	void rotate();
	void scale(double length);
	double length();
  double dotProduct(Vector& v);
};

#endif
