#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include <iostream>

void IndexAsSpatial(int x, int y)
{
    // if getGeoTransform is:
    // 5.86764    0.000277778    0
    //55.0568    0 - 0.000277778
    double xd = 5.86764 + 0.000277778 * x + 0 * y;
    double yd = 55.0568 + 0 * x + (-1) * 0.000277778 * y;
    std::cout << "X': " << xd << " Y': " << yd << std::endl;
}

void SpatialAsIndex(double xd, double yd)
{
    // if getGeoTransform is:
    // 5.86764    0.000277778    0
    //55.0568    0 - 0.000277778
    double x = (xd - 5.86764 - yd * 0) / 0.000277778;
    double y = (yd - 55.0568 - xd * 0) / 0.000277778 * (-1);
    x = std::round(x);
    y = std::round(y);
    int x_index = int(x);
    int y_index = int(y);
    std::cout << "X: " << x_index << " Y: " << y_index << std::endl;
}

int main()
{
    GDALDataset* poDataset;
    GDALAllRegister();
    poDataset = (GDALDataset*)GDALOpen("maps/pop_deu.tif", GA_ReadOnly);
    int rasterCount = poDataset->GetRasterCount();
    if (poDataset != NULL)
    {
        GDALRasterBand* poBand;
        int             nBlockXSize, nBlockYSize;
        
        poBand = poDataset->GetRasterBand(1);
        poBand->GetBlockSize(&nBlockXSize, &nBlockYSize);
        printf("Block=%dx%d Type=%s, ColorInterp=%s\n",
            nBlockXSize, nBlockYSize,
            GDALGetDataTypeName(poBand->GetRasterDataType()),
            GDALGetColorInterpretationName(
                poBand->GetColorInterpretation()));

        float* pafScanline;
        int   nXSize = poBand->GetXSize();
        std::cout << "XSize: " << nXSize << std::endl;
        int   nYSize = poBand->GetYSize();
        std::cout << "YSize: " << nYSize << std::endl;

        pafScanline = (float*)CPLMalloc(sizeof(float) * 1);
        CPLErr success;
        success = poBand->RasterIO(GF_Read, 27046, 9315, 1, 1,
            pafScanline, 1, 1, GDT_Float32,
            0, 0);


        std::cout << "People in raster: " << pafScanline[0] << std::endl;

        double transform[6];
        poDataset->GetGeoTransform(transform);
        std::cout << transform[0] << "    " << transform[1] << "    " << transform[2] << std::endl;
        std::cout << transform[3] << "    " << transform[4] << "    " << transform[5] << std::endl;

        SpatialAsIndex(13.380331649, 52.469187773);

        GDALClose(poDataset);
    }
    else
    {
        printf("The dataset failed to open. Maybe check the filename?");
    }
}