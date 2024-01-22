#include <gtest/gtest.h>
#include "risk_path.h"

TEST(UnitTest_CoordinateTransformations, TestsEverything)
{
    auto map = std::make_shared<PopulationMap>();
    EXPECT_TRUE(map->has_file_loaded);
}

TEST(PopulationMap, TestMethod_RandomPointReverse)
{
    auto map = std::make_shared<PopulationMap>();
    auto point = Coordinates(52.4693, 13.3804, map->transform);
    int expected_x = 27046;
    int expected_y = 9315;
    EXPECT_EQ(expected_x, point.getX());
    EXPECT_EQ(expected_y, point.getY());
}

TEST(PopulationMap, InBounds)
{
    auto map = std::make_shared<PopulationMap>();
    auto point = Coordinates(52.3, 8.9, map->transform);
    EXPECT_TRUE(map->check_map_bounds(point.getLat(), point.getLon()));
}

TEST(PopulationMap, OutOfBounds)
{
    auto map = std::make_shared<PopulationMap>();
    auto point = Coordinates(23.453, 45.036, map->transform);
    EXPECT_FALSE(map->check_map_bounds(point.getLat(), point.getLon()));
}

TEST(PopulationMap, ReadPopulation)
{
    auto map = std::make_shared<PopulationMap>();
    double delta = 0.0001;
    auto point = Coordinates(52.368830, 9.75593335, map->transform);
    double expected_population = 5.97513;
    double population = map->read_population_from_indexes(point.getX(), point.getY());
    ASSERT_NEAR(expected_population, population, delta);
}

TEST(PopulationMap, RandomPoint)
{
    auto map = std::make_shared<PopulationMap>();
    double delta = 0.0001;
    auto point = Coordinates(27046, 9315, map->transform);
    double expected_lat = 52.4693;
    double expected_lon = 13.3804;
    ASSERT_NEAR(expected_lat, point.getLat(), delta);
    ASSERT_NEAR(expected_lon, point.getLon(), delta);
}
