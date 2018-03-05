#include "vector_math.h"

#include <iostream>
#include <math.h>

Position::Position() : x(0), y(0) {}
Position::Position(float x, float y) : x(x), y(y) {}

void Position::add(Vector& v) {
	this->x += v.x;
	this->y += v.y;
}

float Position::dist(Position& p) {
	float dx = this->x - p.x;
	float dy = this->y - p.y;
	return sqrt(dx * dx + dy * dy);
}

Vector::Vector() : x(0), y(0) {}
Vector::Vector(float x, float y) : x(x), y(y) {}

void Vector::add(Vector& v) {
	this->x += v.x;
	this->y += v.y;
}

void Vector::multiply(float scalar) {
	this->x *= scalar;
	this->y *= scalar;
}

void Vector::scale(float length) {
	float vlen = this->length();
  if (vlen == 0) {
    std::cout << this->x << " " << this->y << std::endl;
    std::cout << "div by 0" << std::endl;
  }
	this->x = this->x * length / vlen;
	this->y = this->y * length / vlen;
}

float Vector::length() {
	return sqrt(this->x * this->x + this->y * this->y);
}
