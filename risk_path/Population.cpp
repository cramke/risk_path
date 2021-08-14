#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include <iostream>
#include <array>
#include <cassert>

class PopulationMap {
private:
    const char* filename = "C:/Users/carst/OneDrive/Projekte/risk-path/risk_path/maps/pop_deu.tif";
    GDALDataset* dataset;
    GDALRasterBand* band;
    float* scanline = (float*)CPLMalloc(sizeof(float) * 1);
    int             nXSize, nYSize;
public:
    double transformer[6];
    bool has_file_loaded = false;
    std::array<double, 6> transform;

    PopulationMap() {
        GDALAllRegister();
        dataset = (GDALDataset*)GDALOpen(filename, GA_ReadOnly);
        if (dataset != NULL)
        {
            dataset->GetGeoTransform(transformer);
            transform_array();
            band = dataset->GetRasterBand(1);
            nXSize = band->GetXSize();
            nYSize = band->GetYSize();
            has_file_loaded = true;
        }
        else
        {
            printf("The dataset failed to open. Maybe check the filename?");
            has_file_loaded = false;
        }
    }

    void transform_array()
    {
        for (int i = 0; i < 6; i++) {
            transform[i] = transformer[i];

        }
        check_transform();
    }

    double read_population_from_indexes(int x, int y) const
    {
        band->RasterIO(GF_Read, x, y, 1, 1, scanline, 1, 1, GDT_Float32, 0, 0);
        return (double)scanline[0];
    }

    void close()
    {
        GDALClose(dataset);
    }

    bool check_transform()
    {
        if (transform[2] == 0 && transform[4] == 0)
        {
            return true;
        }
        else {
            return false;
        }
    }

    std::tuple<double, double, double, double> get_spatial_bounds() const {
        double upper_lat = transform[3];
        double lower_lat = upper_lat + nXSize * transform[5]; // transform[5] is negativ
        double lower_lon = transform[0];
        double upper_lon = lower_lon + nYSize * transform[1];
        return std::make_tuple(lower_lat, upper_lat, lower_lon, upper_lon);
    }

};

class Coordinates {
public:
    PopulationMap map;
    double lat;
    double lon;
    int x;
    int y;

    Coordinates(int x_given, int y_given, const PopulationMap& map_given) {
        map = map_given;
        x = x_given;
        y = y_given;
        std::tuple<double, double> spatial = index_to_spatial_coordinates(x, y);
        lat = std::get<0>(spatial);
        lon = std::get<1>(spatial);
        if (!check_map_bounds()) {
            std::cout << "Coordinate is out of Map boundaries" << std::endl;
        }
    }

    Coordinates(double lat_given, double lon_given, const PopulationMap& map_given) {
        map = map_given;
        lat = lat_given;
        lon = lon_given;
        std::tuple<int, int> index = spatial_to_index_coordinates(lat, lon);
        x = std::get<0>(index);
        y = std::get<1>(index);
        if (!check_map_bounds()) {
            std::cout << "Coordinate is out of Map boundaries" << std::endl;
        }
    }

    std::tuple<double, double>  index_to_spatial_coordinates(int xd, int yd) {
        // https://gdal.org/tutorials/geotransforms_tut.html
        double lon = map.transform[0] + map.transform[1] * xd + map.transform[2] * yd;
        double lat = map.transform[3] + map.transform[4] * xd + map.transform[5] * yd;
        return std::make_tuple(lat, lon);
    }

    std::tuple<int, int> spatial_to_index_coordinates(double lat, double lon) {
        double x = ((lon - map.transform[0]) - (map.transform[2] * 0)) / map.transform[1];
        double y = ((lat - map.transform[3]) - (map.transform[4] * 0)) / map.transform[5];
        int x_index = int(std::round(x));
        int y_index = int(std::round(y));
        return std::make_tuple(x_index, y_index);
    }

    bool check_map_bounds() {
        auto bounds = map.get_spatial_bounds();
        if (lat > std::get<0>(bounds) &&
            lat < std::get<1>(bounds) &&
            lon > std::get<2>(bounds) &&
            lon < std::get<3>(bounds)) {
            return true;
        }
        else {
            return false;
        }
    }

    double get_population() {
        return map.read_population_from_indexes(x, y);
    }
};


