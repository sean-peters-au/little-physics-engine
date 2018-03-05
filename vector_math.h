#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

class Vector
{
	public:
	double x;
	double y;

  Vector();
	Vector(double x, double y);

	void add(Vector& v);
	void multiply(double scalar);
	void scale(double length);
	double length();
};

class Position
{
	public:
	double x;
	double y;

  Position();
	Position(double x, double y);

	void add(Vector& v);
	double dist(Position& p);
};

#endif
