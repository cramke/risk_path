#pragma once
#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include <iostream>
#include <array>

class PopulationMap {
private:
    const char* filename = "/home/samtal/risk_path/ressources/pop_deu.tif";
    GDALDataset* dataset;
    GDALRasterBand* band;
    float* scanline;
    int nXSize;
    int nYSize;
public:
    bool has_file_loaded;
    std::array<double, 6> transform;

    PopulationMap();
    void close();
    bool check_transform();
    double read_population_from_indexes(int x, int y) const;
    // TODO: Implement read from Coordinates
    std::tuple<double, double, double, double> get_spatial_bounds() const;
    bool check_map_bounds(double lat, double lon) const;
};

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