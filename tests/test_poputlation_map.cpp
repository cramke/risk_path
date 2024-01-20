#include <gtest/gtest.h>
#include "src/planning/risk_path.h"

TEST(UnitTest_CoordinateTransformations, TestsEverything)
{
    auto map = std::make_shared<PopulationMap>();
    EXPECT_TRUE(map->has_file_loaded);
}

TEST(Coordinates, TransformFunction)
{
    auto map = std::make_shared<PopulationMap>();
    EXPECT_EQ(0.0, map->transform[2]);
    EXPECT_EQ(0.0, map->transform[4]);
}

TEST(Coordinates, TransformMatrix)
{
    auto map = std::make_shared<PopulationMap>();
    double delta = 0.0001;
    EXPECT_EQ(0.00027777777777799998, map->transform[1]);
    EXPECT_EQ(-0.00027777777777799998, map->transform[5]);
    ASSERT_NEAR(5.8676388888888891, map->transform[0], delta);
    ASSERT_NEAR(55.056805555555556, map->transform[3], delta);
}

TEST(Coordinates, RandomPoint)
{
    auto map = std::make_shared<PopulationMap>();
    double delta = 0.0001;
    auto point = Coordinates(0, 0, map);
    double expected_lat = 55.056805555555556;
    double expected_lon = 5.8676388888888891;
    ASSERT_NEAR(expected_lat, point.lat, delta);
    ASSERT_NEAR(expected_lon, point.lon, delta);
}

TEST(Coordinates, ExpectZero)
{
    auto map = std::make_shared<PopulationMap>();
    double lat = 55.056805555555556;
    double lon = 5.8676388888888891;
    auto point = Coordinates(lat, lon, map);
    int expected_x = 0;
    int expected_y = 0;
    EXPECT_EQ(expected_x, point.x);
    EXPECT_EQ(expected_y, point.y);
}

TEST(PopulationMap, TestMethod_RandomPointReverse)
{
    auto map = std::make_shared<PopulationMap>();
    auto point = Coordinates(52.4693, 13.3804, map);
    int expected_x = 27046;
    int expected_y = 9315;
    EXPECT_EQ(expected_x, point.x);
    EXPECT_EQ(expected_y, point.y);
}

TEST(PopulationMap, InBounds)
{
    auto map = std::make_shared<PopulationMap>();
    auto point = Coordinates(52.3, 8.9, map);
    EXPECT_TRUE(map->check_map_bounds(point.lat, point.lon));
}

TEST(PopulationMap, OutOfBounds)
{
    auto map = std::make_shared<PopulationMap>();
    auto point = Coordinates(23.453, 45.036, map);
    EXPECT_FALSE(map->check_map_bounds(point.lat, point.lon));
}

TEST(PopulationMap, ReadPopulation) 
{
    auto map = std::make_shared<PopulationMap>();
    double delta = 0.0001;
    auto point = Coordinates(52.368830, 9.75593335, map);
    double expected_population = 5.97513;
    double population = map->read_population_from_indexes(point.x, point.y);
    ASSERT_NEAR(expected_population, population, delta);
}

TEST(PopulationMap, RandomPoint) 
{
    auto map = std::make_shared<PopulationMap>();
    double delta = 0.0001;
    auto point = Coordinates(27046, 9315, map);
    double expected_lat = 52.4693;
    double expected_lon = 13.3804;
    ASSERT_NEAR(expected_lat, point.lat, delta);
    ASSERT_NEAR(expected_lon, point.lon, delta);
}


