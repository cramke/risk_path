#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include <iostream>


class PopMap {
    private:
        const char *filename = "maps/pop_deu.tif";
        GDALDataset* dataset;
        GDALRasterBand* band;
        float* scanline = (float*)CPLMalloc(sizeof(float) * 1);
        int             nXSize, nYSize;
    public:
        double transform[6];

        PopMap() {
            GDALAllRegister();
            dataset = (GDALDataset*)GDALOpen(filename, GA_ReadOnly);
            if (dataset != NULL) 
            {
                std::cout << "Dataset successfully loaded." << std::endl;
                dataset->GetGeoTransform(transform);
                band = dataset->GetRasterBand(1);
            }
            else
            {
                printf("The dataset failed to open. Maybe check the filename?");
            }
        }

        
        std::tuple<int, int> SpatialAsIndex(double xd, double yd)
        {
            // https://gdal.org/tutorials/geotransforms_tut.html
            double x = (xd - transform[0] - yd * transform[2]) / transform[1];
            double y = (yd - transform[3] - xd * transform[4]) / transform[5];
            int x_index = int(std::round(x));
            int y_index = int(std::round(y));
            return std::make_tuple(x_index, y_index);
        }

        std::tuple<int, int> IndexAsSpatial(int x, int y)
        {
            double x_spatial = transform[0] + transform[1] * x + transform[2] * y;
            double y_spatial = transform[3] + transform[4] * x + transform[5] * y;
            std::cout << "X': " << x_spatial << " Y': " << y_spatial << std::endl;
            return std::make_tuple(x_spatial, y_spatial);
        }

        float getPopIndex(int x, int y)
        {
            // 27046 9315
            band->RasterIO(GF_Read, x, y, 1, 1, scanline, 1, 1, GDT_Float32, 0, 0);
            return scanline[0];
        }

        float getPopGeo(double x_geo, double y_geo)
        {
            auto [x_index, y_index] = SpatialAsIndex(x_geo, y_geo);
            return getPopIndex(x_index, y_index); 
        }

        void close()
        {
            GDALClose(dataset);
        }

        void printBandSize()
        {
            nXSize = band->GetXSize();
            std::cout << "XSize: " << nXSize << std::endl;
            nYSize = band->GetYSize();
            std::cout << "YSize: " << nYSize << std::endl;
        }

};

int test()
{
    PopMap ds = PopMap();
    double pop1 = ds.getPopIndex(27046, 9315);
    ds.IndexAsSpatial(27046, 9315);
    std::cout << "Population: " << pop1 << std::endl;
    double pop2 = ds.getPopGeo(13.3804, 52.4693);
    std::cout << "Population: " << pop2 << std::endl;
    return 1;
}

