#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

class Vector
{
	public:
	float x;
	float y;

  Vector();
	Vector(float x, float y);

	void add(Vector& v);
	void multiply(float scalar);
	void scale(float length);
	float length();
};

class Position
{
	public:
	float x;
	float y;

  Position();
	Position(float x, float y);

	void add(Vector& v);
	float dist(Position& p);
};

#endif
