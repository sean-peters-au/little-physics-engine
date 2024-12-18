#include <gtest/gtest.h>
#include "nbody/math/vector_math.hpp"

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

    // Test normalize
    Vector v4(3.0, 4.0);
    v4.normalize();
    EXPECT_DOUBLE_EQ(v4.length(), 1.0);

    // Test cross product
    Vector v5(1.0, 0.0);
    Vector v6(0.0, 1.0);
    EXPECT_DOUBLE_EQ(v5.cross(v6), 1.0);

    // Test perpendicular vector
    Vector v7(1.0, 0.0);
    Vector perp_v7 = v7.perp();
    EXPECT_DOUBLE_EQ(perp_v7.x, 0.0);
    EXPECT_DOUBLE_EQ(perp_v7.y, 1.0);

    // Test angle between vectors
    Vector v8(1.0, 0.0);
    Vector v9(0.0, 1.0);
    EXPECT_DOUBLE_EQ(v8.angleBetween(v9), M_PI / 2);

    // Test rotate by angle
    Vector v10(1.0, 0.0);
    Vector rotated_v10 = v10.rotateByAngle(M_PI / 2);
    EXPECT_NEAR(rotated_v10.x, 0.0, EPSILON);
    EXPECT_NEAR(rotated_v10.y, 1.0, EPSILON);

    // Test projection length
    Vector v11(3.0, 4.0);
    Vector v12(1.0, 0.0);
    EXPECT_DOUBLE_EQ(v11.projectLength(v12), 3.0);

    // Test projection onto another vector
    Vector projected_v11 = v11.projectOnto(v12);
    EXPECT_DOUBLE_EQ(projected_v11.x, 3.0);
    EXPECT_DOUBLE_EQ(projected_v11.y, 0.0);
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