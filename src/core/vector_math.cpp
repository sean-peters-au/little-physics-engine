#include "nbody/core/vector_math.hpp"

#include <iostream>
#include <cmath>

double my_sqrt(double d) {
  if (d < 0) {
    std::cout << "-1 sqrt" << std::endl;
  }
  return sqrt(d);
}

// Position


Position::Position() : x(0), y(0) {}
Position::Position(double x, double y) : x(x), y(y) {}

// operators

Position::operator Vector() const { 
  return Vector(this->x, this->y); 
}

Position Position::operator+(const Position& b) {
  Position pos;
  pos.x = this->x + b.x;
  pos.y = this->y + b.y;
  return pos;
}

Position Position::operator-(const Position& b) {
  Position pos;
  pos.x = this->x - b.x;
  pos.y = this->y - b.y;
  return pos;
}

Position Position::operator*(const double scalar) {
  Position pos;
  pos.x = this->x * scalar;
  pos.y = this->y * scalar;
  return pos;
}

Position Position::operator/(const double scalar) {
  Position pos;
  pos.x = this->x / scalar;
  pos.y = this->y / scalar;
  return pos;
}

Position& Position::operator+=(const Position& b){
  this->x += b.x;
  this->y += b.y;
  return *this;
}

Position& Position::operator-=(const Position& b){
  this->x -= b.x;
  this->y -= b.y;
  return *this;
}

Position& Position::operator*=(const double scalar){
  this->x *= scalar;
  this->y *= scalar;
  return *this;
}

Position& Position::operator/=(const double scalar){
  this->x /= scalar;
  this->y /= scalar;
  return *this;
}

// avoids an unnecessary constructor call

void Position::add(Position& v) {
	this->x += v.x;
	this->y += v.y;
}

void Position::subtract(Position& v) {
	this->x -= v.x;
	this->y -= v.y;
}

void Position::multiply(Position& v) {
	this->x *= v.x;
	this->y *= v.y;
}

void Position::divide(Position& v) {
	this->x /= v.x;
	this->y /= v.y;
}

double Position::dist(Position& p) {
	double dx = this->x - p.x;
	double dy = this->y - p.y;
	return my_sqrt(dx * dx + dy * dy);
}


// Vector


Vector::Vector() : x(0), y(0) {}
Vector::Vector(double x, double y) : x(x), y(y) {}
Vector::Vector(Position& p) : x(p.x), y(p.y) {}


// operators

Vector::operator Position() const { 
  return Position(this->x, this->y);
}

Vector Vector::operator+(const Vector& b) {
  Vector a;
  a.x = this->x + b.x;
  a.y = this->y + b.y;
  return a;
}

Vector Vector::operator-(const Vector& b) {
  Vector a;
  a.x = this->x - b.x;
  a.y = this->y - b.y;
  return a;
}

Vector Vector::operator*(const double scalar) {
  Vector a;
  a.x = this->x * scalar;
  a.y = this->y * scalar;
  return a;
}

Vector Vector::operator/(const double scalar) {
  Vector a;
  a.x = this->x / scalar;
  a.y = this->y / scalar;
  return a;
}

Vector& Vector::operator+=(const Vector& b){
  this->x += b.x;
  this->y += b.y;
  return *this;
}

Vector& Vector::operator-=(const Vector& b){
  this->x -= b.x;
  this->y -= b.y;
  return *this;
}

Vector& Vector::operator*=(const double scalar){
  this->x *= scalar;
  this->y *= scalar;
  return *this;
}

Vector& Vector::operator/=(const double scalar){
  this->x /= scalar;
  this->y /= scalar;
  return *this;
}

// avoids an unnecessary constructor call

void Vector::add(Vector& v) {
	this->x += v.x;
	this->y += v.y;
}

void Vector::subtract(Vector& v) {
	this->x -= v.x;
	this->y -= v.y;
}

void Vector::multiply(double scalar) {
	this->x *= scalar;
	this->y *= scalar;
}

void Vector::divide(double scalar) {
	this->x /= scalar;
	this->y /= scalar;
}

// vector math

void Vector::rotate() { // clockwise 90 degrees
  double temp = this->x;
  this->x = -this->y;
  this->y = temp;
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
	return my_sqrt(this->x * this->x + this->y * this->y);
}

double Vector::dotProduct(Vector& v) {
	return this->x * v.x + this->y * v.y;
}
