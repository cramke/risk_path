#pragma once
#include <iostream>
#include <array>
#include <cmath>

class Coordinates
{
public:
    Coordinates(int x, int y, std::array<double, 6> &transform);
    Coordinates(double lat_given, double lon_given, std::array<double, 6> &transform);

    void index_to_spatial_coordinates();
    void spatial_to_index_coordinates();

    // Getters and Setters
    double getLat() const { return lat; }
    void setLat(double newLat) { lat = newLat; }

    double getLon() const { return lon; }
    void setLon(double newLon) { lon = newLon; }

    int getX() const { return x; }
    void setX(int newX) { x = newX; }

    int getY() const { return y; }
    void setY(int newY) { this->y = newY; }

    std::array<double, 6> getTransform() const { return transform; }
    void setTransform(const std::array<double, 6> &newTransform) { transform = newTransform; }

private:
    double lat;
    double lon;
    int x;
    int y;
    std::array<double, 6> transform;
};
