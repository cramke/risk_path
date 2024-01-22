#pragma once
#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include <iostream>
#include <array>
#include "Coordinates.hpp"

// Forward declaration because of circular dependency of Coordinates.hpp and Population.h
class Coordinates;

class PopulationMap
{
private:
    const char *filename = "/home/samtal/risk_path/ressources/pop_deu.tif";
    GDALDataset *dataset;
    GDALRasterBand *band;
    float *scanline;
    int nXSize;
    int nYSize;

public:
    bool has_file_loaded;
    std::array<double, 6> transform;

    PopulationMap();
    void close();
    bool check_transform();
    double read_population_from_indexes(int x, int y) const;
    double read_population_from_coooordinates(const Coordinates &coords) const;
    std::tuple<double, double, double, double> get_spatial_bounds() const;
    bool check_map_bounds(double lat, double lon) const;
};
