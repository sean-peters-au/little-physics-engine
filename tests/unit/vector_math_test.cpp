#include <gtest/gtest.h>
#include "nbody/core/vector_math.hpp"

TEST(VectorMathTest, VectorConstruction) {
    Vector v1;  // Default constructor
    EXPECT_DOUBLE_EQ(v1.x, 0.0);
    EXPECT_DOUBLE_EQ(v1.y, 0.0);

    Vector v2(3.0, 4.0);  // Parameterized constructor
    EXPECT_DOUBLE_EQ(v2.x, 3.0);
    EXPECT_DOUBLE_EQ(v2.y, 4.0);
}

TEST(VectorMathTest, VectorAddition) {
    Vector v1(1.0, 2.0);
    Vector v2(3.0, 4.0);
    
    // Test operator+
    Vector result = v1 + v2;
    EXPECT_DOUBLE_EQ(result.x, 4.0);
    EXPECT_DOUBLE_EQ(result.y, 6.0);

    // Test operator+=
    v1 += v2;
    EXPECT_DOUBLE_EQ(v1.x, 4.0);
    EXPECT_DOUBLE_EQ(v1.y, 6.0);

    // Test add method
    Vector v3(1.0, 2.0);
    Vector v4(3.0, 4.0);
    v3.add(v4);
    EXPECT_DOUBLE_EQ(v3.x, 4.0);
    EXPECT_DOUBLE_EQ(v3.y, 6.0);
}

TEST(VectorMathTest, VectorScalarOperations) {
    Vector v(2.0, 3.0);
    double scalar = 2.0;
    
    // Test multiplication
    Vector mult_result = v * scalar;
    EXPECT_DOUBLE_EQ(mult_result.x, 4.0);
    EXPECT_DOUBLE_EQ(mult_result.y, 6.0);

    // Test division
    Vector div_result = v / 0.5;
    EXPECT_DOUBLE_EQ(div_result.x, 4.0);
    EXPECT_DOUBLE_EQ(div_result.y, 6.0);
}

TEST(VectorMathTest, VectorMethods) {
    Vector v(3.0, 4.0);
    
    // Test length
    EXPECT_DOUBLE_EQ(v.length(), 5.0);

    // Test scale
    Vector v2(3.0, 4.0);
    v2.scale(10.0);
    EXPECT_DOUBLE_EQ(v2.length(), 10.0);

    // Test rotate (90 degrees clockwise)
    Vector v3(1.0, 1.0);
    v3.rotate();
    EXPECT_DOUBLE_EQ(v3.x, -1.0);
    EXPECT_DOUBLE_EQ(v3.y, 1.0);
}

TEST(VectorMathTest, PositionOperations) {
    Position p1(1.0, 2.0);
    Position p2(3.0, 4.0);

    // Test addition
    Position p3 = p1 + p2;
    EXPECT_DOUBLE_EQ(p3.x, 4.0);
    EXPECT_DOUBLE_EQ(p3.y, 6.0);

    // Test distance
    EXPECT_DOUBLE_EQ(p1.dist(p2), 2.8284271247461903);  // sqrt(8)
}

TEST(VectorMathTest, VectorPositionConversion) {
    Position p(1.0, 2.0);
    Vector v = static_cast<Vector>(p);
    EXPECT_DOUBLE_EQ(v.x, 1.0);
    EXPECT_DOUBLE_EQ(v.y, 2.0);

    Vector v2(3.0, 4.0);
    Position p2 = static_cast<Position>(v2);
    EXPECT_DOUBLE_EQ(p2.x, 3.0);
    EXPECT_DOUBLE_EQ(p2.y, 4.0);
}

TEST(VectorMathTest, DotProduct) {
    Vector v1(1.0, 2.0);
    Vector v2(3.0, 4.0);
    EXPECT_DOUBLE_EQ(v1.dotProduct(v2), 11.0);  // 1*3 + 2*4
} 