#pragma once
#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include <iostream>
#include <array>

class PopulationMap {
private:
    const char* filename;
    GDALDataset* dataset;
    GDALRasterBand* band;
    float* scanline;
    int nXSize, nYSize;
public:
    double transformer[6];
    bool has_file_loaded;
    std::array<double, 6> transform;

    PopulationMap();

    void transform_array();
    void close();
    bool check_transform();
    double read_population_from_indexes(int x, int y) const;
    // double read_propulation_from_point(Coordinates point);
    std::tuple<double, double, double, double> get_spatial_bounds() const;
    bool check_map_bounds(double lat, double lon) const;
};

class Coordinates {
public:
    std::array<double, 6> transform;
    double lat;
    double lon;
    int x;
    int y;

    Coordinates(int x , int y, std::shared_ptr<PopulationMap> map_given);
    Coordinates(double lat_given, double lon_given, std::shared_ptr<PopulationMap> map_given);

    std::tuple<double, double> index_to_spatial_coordinates(int, int);
    std::tuple<int, int> spatial_to_index_coordinates(double, double);
};