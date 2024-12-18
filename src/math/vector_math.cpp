#include "nbody/math/vector_math.hpp"

#include <iostream>
#include <cmath>

double my_sqrt(double d) {
  if (d < 0) {
    std::cout << "-1 sqrt" << std::endl;
  }
  return sqrt(d);
}

bool nearlyEqual(double a, double b, double epsilon) {
  return std::fabs(a-b) < epsilon;
}

// Position

Position::Position() : x(0), y(0) {}
Position::Position(double x, double y) : x(x), y(y) {}

Position::operator Vector() const { 
  return Vector(this->x, this->y); 
}

Position Position::operator+(const Position& b) const {
  return Position(this->x + b.x, this->y + b.y);
}

Position Position::operator-(const Position& b) const {
  return Position(this->x - b.x, this->y - b.y);
}

Position Position::operator*(double scalar) const {
  return Position(this->x * scalar, this->y * scalar);
}

Position Position::operator/(double scalar) const {
  return Position(this->x / scalar, this->y / scalar);
}

double Position::dist(const Position& p) const {
	double dx = this->x - p.x;
	double dy = this->y - p.y;
	return my_sqrt(dx * dx + dy * dy);
}

// Vector

Vector::Vector() : x(0), y(0) {}
Vector::Vector(double x, double y) : x(x), y(y) {}
Vector::Vector(const Position& p) : x(p.x), y(p.y) {}

Vector::operator Position() const { 
  return Position(this->x, this->y);
}

Vector Vector::operator-() const {
    return Vector(-this->x, -this->y);
}

Vector Vector::operator+(const Vector& b) const {
  return Vector(this->x + b.x, this->y + b.y);
}

Vector Vector::operator-(const Vector& b) const {
  return Vector(this->x - b.x, this->y - b.y);
}

Vector Vector::operator*(double scalar) const {
  return Vector(this->x * scalar, this->y * scalar);
}

Vector Vector::operator/(double scalar) const {
  return Vector(this->x / scalar, this->y / scalar);
}

double Vector::length() const {
	return my_sqrt(this->x * this->x + this->y * this->y);
}

double Vector::dotProduct(const Vector& v) const {
	return this->x * v.x + this->y * v.y;
}

double Vector::cross(const Vector &other) const {
  return this->x * other.y - this->y * other.x;
}

Vector Vector::perp() const {
  return Vector(-this->y, this->x); 
}

Vector Vector::normalized() const {
  double len = this->length();
  if (len > 1e-9) {
    return Vector(this->x / len, this->y / len);
  } else {
    // default direction if zero-length vector
    return Vector(1.0, 0.0);
  }
}

double Vector::projectLength(const Vector &onto) const {
  return (this->x * onto.x + this->y * onto.y) / onto.length();
}

Vector Vector::projectOnto(const Vector &onto) const {
  double denom = onto.dotProduct(onto);
  if (denom < 1e-9) return Vector(0,0);
  double scalar = (this->x * onto.x + this->y * onto.y) / denom;
  return Vector(onto.x * scalar, onto.y * scalar);
}

Vector Vector::rotateByAngle(double angle) const {
  double c = std::cos(angle);
  double s = std::sin(angle);
  return Vector(this->x*c - this->y*s, this->x*s + this->y*c);
}

double Vector::angleBetween(const Vector &other) const {
  double lenProduct = this->length()*other.length();
  if (lenProduct < 1e-9) {
    return 0.0; // undefined, but return 0 for no angle
  }
  double dotVal = (this->x * other.x + this->y * other.y) / lenProduct;
  dotVal = std::max(-1.0,std::min(1.0,dotVal));
  return std::acos(dotVal);
}

Vector Vector::rotate() const {
  // Rotate 90 degrees clockwise creates a new vector
  return Vector(this->y, -this->x);
}

Vector Vector::scale(double length) const {
	double vlen = this->length();
  if (vlen > 1e-9) {
    double factor = length / vlen;
    return Vector(this->x * factor, this->y * factor);
  } else {
    std::cout << "Warning: scaling zero-length vector." << std::endl;
    return *this;
  }
}

Vector closestPointOnLine(const Vector &a, const Vector &b, const Vector &p) {
  Vector ab = b - a;
  double denom = ab.dotProduct(ab);
  if (denom < 1e-9) return a;
  double t = ((p.x - a.x)*ab.x + (p.y - a.y)*ab.y) / denom;
  t = std::max(0.0,std::min(1.0,t));
  return Vector(a.x + ab.x*t, a.y + ab.y*t);
}