#pragma once
#include <iostream>
#include <array>
#include "Population.h"

// Forward declaration because of circular dependency of Coordinates.hpp and Population.h
class PopulationMap;

class Coordinates {
public:
    double lat;
    double lon;
    int x;
    int y;
    std::array<double, 6> transform;

    Coordinates(int x , int y, std::shared_ptr<PopulationMap> map_given);
    Coordinates(double lat_given, double lon_given, std::shared_ptr<PopulationMap> map_given);

    void index_to_spatial_coordinates();
    void spatial_to_index_coordinates();
};
