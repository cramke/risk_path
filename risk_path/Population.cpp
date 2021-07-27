#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include <iostream>

class Coordinates {
private:
    double transform[6];
public:
    double lat;
    double lon;
    int x;
    int y;
    
    Coordinates(int x, int y, double transform[6]) {
        transform = transform;
        x = x;
        y = y;
        std::tuple<double, double> spatial = index_to_spatial(x, y);
        lat = std::get<0>(spatial);
        lon = std::get<1>(spatial);
    }

    Coordinates(double lat, double lon, double transform[6]) {
        transform = transform;
        lat = lat;
        lon = lon;
        std::tuple<int, int> spatial = spatial_to_index(lat, lon);
        x = std::get<0>(spatial);
        x = std::get<1>(spatial);
    }

    std::tuple<int, int>  index_to_spatial(int xd, int yd) {
        // https://gdal.org/tutorials/geotransforms_tut.html
        double x = (xd - transform[0] - yd * transform[2]) / transform[1];
        double y = (yd - transform[3] - xd * transform[4]) / transform[5];
        int x_index = int(std::round(x));
        int y_index = int(std::round(y));
        return std::make_tuple(x_index, y_index);
    }

    std::tuple<double, double> spatial_to_index(double lat, double lon) {
        double x_spatial = transform[0] + transform[1] * x + transform[2] * y;
        double y_spatial = transform[3] + transform[4] * x + transform[5] * y;
        return std::make_tuple(x_spatial, y_spatial);
    }
};


class PopulationMap {
    private:
        const char *filename = "maps/pop_deu.tif";
        GDALDataset* dataset;
        GDALRasterBand* band;
        float* scanline = (float*)CPLMalloc(sizeof(float) * 1);
        int             nXSize, nYSize;
    public:
        double transform[6];

        PopulationMap()
        {
            GDALAllRegister();
            dataset = (GDALDataset*)GDALOpen(filename, GA_ReadOnly);
            if (dataset != NULL) 
            {
                std::cout << "Dataset successfully loaded." << std::endl;
                dataset->GetGeoTransform(transform);
                band = dataset->GetRasterBand(1);
                nXSize = band->GetXSize();
                nYSize = band->GetYSize();
            }
            else
            {
                printf("The dataset failed to open. Maybe check the filename?");
            }
        }

        
        std::tuple<int, int> coordinates_spatial_to_index(double xd, double yd) const
        {
            // https://gdal.org/tutorials/geotransforms_tut.html
            double x = (xd - transform[0] - yd * transform[2]) / transform[1];
            double y = (yd - transform[3] - xd * transform[4]) / transform[5];
            int x_index = int(std::round(x));
            int y_index = int(std::round(y));
            return std::make_tuple(x_index, y_index);
        }

        std::tuple<double, double> index_to_spatial(int x, int y) const
        {
            double x_spatial = transform[0] + transform[1] * x + transform[2] * y;
            double y_spatial = transform[3] + transform[4] * x + transform[5] * y;
            return std::make_tuple(x_spatial, y_spatial);
        }

        float read_population_from_index(int x, int y) const 
        {
            // 27046 9315
            band->RasterIO(GF_Read, x, y, 1, 1, scanline, 1, 1, GDT_Float32, 0, 0);
            return scanline[0];
        }

        float read_population_from_spatial(double x_geo, double y_geo) const
        // TODO: x_geo / y_geo to lat lon. But which is which?
        {
            auto [x_index, y_index] = coordinates_spatial_to_index(x_geo, y_geo);
            return read_population_from_index(x_index, y_index); 
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

/*
void test()
{
    PopMap ds = PopMap();
    double pop1 = ds.getPopIndex(27046, 9315);
    ds.IndexAsSpatial(27046, 9315);
    std::cout << "Population: " << pop1 << std::endl;
    double pop2 = ds.getPopGeo(13.3804, 52.4693);
    std::cout << "Population: " << pop2 << std::endl;
}
*/