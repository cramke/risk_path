#include <gtest/gtest.h>
#include "Coordinates.hpp"

TEST(Coordinates, TestConstructorWithIntegers)
{
    std::array<double, 6> transform = {
        5.8676388888888891, 0.00027777777777799998, 0.0,
        55.056805555555556, 0.0, -0.00027777777777799998};
    Coordinates coordinates(0, 0, transform);
    EXPECT_NEAR(55.056805555555556, coordinates.getLat(), 0.0001);
    EXPECT_NEAR(5.8676388888888891, coordinates.getLon(), 0.0001);
    EXPECT_EQ(transform, coordinates.getTransform());
}

TEST(Coordinates, TestConstructorWithDoubles)
{
    std::array<double, 6> transform = {
        5.8676388888888891, 0.00027777777777799998, 0.0,
        55.056805555555556, 0.0, -0.00027777777777799998};
    double lon = 5.8676388888888891;
    double lat = 55.056805555555556;
    auto coordinates = Coordinates(lat, lon, transform);
    EXPECT_EQ(0, coordinates.getX());
    EXPECT_EQ(0, coordinates.getY());
    EXPECT_EQ(transform, coordinates.getTransform());
}

TEST(Coordinates, TestTransformSetter)
{
    std::array<double, 6> transform = {
        5.8676388888888891, 0.00027777777777799998, 0.0,
        55.056805555555556, 0.0, -0.00027777777777799998};
    Coordinates coordinates(10, 20, transform);
    EXPECT_EQ(transform, coordinates.getTransform());
}

TEST(Coordinates, TestIndexToSpatialCoordinates)
{
    std::array<double, 6> transform = {
        5.8676388888888891, 0.00027777777777799998, 0.0,
        55.056805555555556, 0.0, -0.00027777777777799998};
    double delta = 0.0001;
    Coordinates coordinates(5023, 3774, transform);
    EXPECT_NEAR(54.008472222221386, coordinates.getLat(), delta);
    EXPECT_NEAR(7.2629166666677829, coordinates.getLon(), delta);
}

TEST(Coordinates, TestSpatialToIndexCoordinates)
{
    std::array<double, 6> transform = {
        5.8676388888888891, 0.00027777777777799998, 0.0,
        55.056805555555556, 0.0, -0.00027777777777799998};
    double lat = 49.0;
    double lon = 9.5;
    auto point = Coordinates(lat, lon, transform);
    EXPECT_EQ(13076, point.getX());
    EXPECT_EQ(21804, point.getY());
}

// Add more test cases as needed