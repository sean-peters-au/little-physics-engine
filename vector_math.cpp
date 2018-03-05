#include "vector_math.h"

#include <iostream>
#include <math.h>

Position::Position() : x(0), y(0) {}
Position::Position(double x, double y) : x(x), y(y) {}

void Position::add(Vector& v) {
	this->x += v.x;
	this->y += v.y;
}

double Position::dist(Position& p) {
	double dx = this->x - p.x;
	double dy = this->y - p.y;
	return sqrt(dx * dx + dy * dy);
}

Vector::Vector() : x(0), y(0) {}
Vector::Vector(double x, double y) : x(x), y(y) {}

void Vector::add(Vector& v) {
	this->x += v.x;
	this->y += v.y;
}

void Vector::multiply(double scalar) {
	this->x *= scalar;
	this->y *= scalar;
}

void Vector::scale(double length) {
	double vlen = this->length();
  if (vlen == 0) {
    std::cout << this->x << " " << this->y << std::endl;
    std::cout << "div by 0" << std::endl;
  }
	this->x = this->x * length / vlen;
	this->y = this->y * length / vlen;
}

double Vector::length() {
	return sqrt(this->x * this->x + this->y * this->y);
}
