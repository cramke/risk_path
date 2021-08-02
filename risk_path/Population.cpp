#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include <iostream>
#include <array>
#include <cassert>


class Coordinates {
private:
    std::array<double, 6> transform;
public:
    double lat;
    double lon;
    int x;
    int y;

    Coordinates(int x, int y, std::array<double, 6> &given) {
        transform = given;
        x = x;
        y = y;
        std::tuple<double, double> spatial = index_to_spatial_coordinates(x, y);
        lat = std::get<0>(spatial);
        lon = std::get<1>(spatial); 
    }

    Coordinates(double lat, double lon, std::array<double, 6> &given) {
        transform = given;
        lat = lat;
        lon = lon;
        std::tuple<int, int> index = spatial_to_index_coordinates(lat, lon);
        x = std::get<0>(index);
        y = std::get<1>(index);
    }

private:
    std::tuple<double, double>  index_to_spatial_coordinates(int xd, int yd) {
        // https://gdal.org/tutorials/geotransforms_tut.html
        double lon = transform[0] + transform[1] * xd + transform[2] * yd;
        double lat = transform[3] + transform[4] * xd + transform[5] * yd;
        return std::make_tuple(lat, lon);
    }

    std::tuple<int, int> spatial_to_index_coordinates(double lat, double lon) {
        // assert that transform[2]&[4] == 0 otherwise logic error and transform wont work.
        assert(transform[2] == 0);
        assert(transform[4] == 0);
        double x = ((lon - transform[0]) - (transform[2] * 0)) / transform[1];
        double y = ((lat - transform[3]) - (transform[4] * 0)) / transform[5];
        int x_index = int(std::round(x));
        int y_index = int(std::round(y));
        return std::make_tuple(x_index, y_index);
    }
};


class PopulationMap {
    private:
        const char *filename = "C:/Users/carst/source/repos/risk_path/risk_path/maps/pop_deu.tif";
        GDALDataset* dataset;
        GDALRasterBand* band;
        float* scanline = (float*)CPLMalloc(sizeof(float) * 1);
        int             nXSize, nYSize;
    public:
        double transformer[6];
        bool has_file_loaded = false;
        std::array<double, 6> transform;

        PopulationMap()
        {
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
            for (int i=0; i<6; i++)
            {
                transform[i] = transformer[i];
            }
        }

        double read_population_from_coordinates(Coordinates coords) const 
        {
            // 27046 9315
            band->RasterIO(GF_Read, coords.x, coords.y, 1, 1, scanline, 1, 1, GDT_Float32, 0, 0);
            return (double) scanline[0];
        }

        void close()
        {
            GDALClose(dataset);
        }

        void print_band_size()
        {       
            std::cout << "XSize: " << nXSize << std::endl;
            std::cout << "YSize: " << nYSize << std::endl;
        }

};