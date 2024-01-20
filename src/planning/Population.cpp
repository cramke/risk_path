#include "Population.h"


PopulationMap::PopulationMap()
{
    GDALAllRegister();
    dataset = (GDALDataset*)GDALOpen(filename, GA_ReadOnly);
    if (dataset != NULL) {
        dataset->GetGeoTransform(transform.data());
        check_transform();
        band = dataset->GetRasterBand(1);
        nXSize = band->GetXSize();
        nYSize = band->GetYSize();
        scanline = (float*)CPLMalloc(sizeof(float) * nXSize);
        has_file_loaded = true;
    } else {
        printf("The dataset failed to open. Maybe check the filename?");
        throw std::ios_base::failure("File not found or file is empty");
    }
}

double PopulationMap::read_population_from_indexes(int x, int y) const 
{
    auto result = band->RasterIO(GF_Read, x, y, 1, 1, scanline, 1, 1, GDT_Float32, 0, 0);
    return (double)scanline[0];
}

double PopulationMap::read_population_from_coooordinates(const Coordinates & coords) const
{
    return read_population_from_indexes(coords.x, coords.y);
}

void PopulationMap::close() 
{
    GDALClose(dataset);
}

bool PopulationMap::check_transform() 
{
    if (transform[2] == 0 && transform[4] == 0) 
    {
        return true;
    }
    else 
    {
        return false;
    }
}

std::tuple<double, double, double, double> PopulationMap::get_spatial_bounds() const 
{
    double upper_lat = transform[3];
    double lower_lat = upper_lat + nXSize * transform[5]; // transform[5] is negativ
    double lower_lon = transform[0];
    double upper_lon = lower_lon + nYSize * transform[1];
    return std::make_tuple(lower_lat, upper_lat, lower_lon, upper_lon);
}

bool PopulationMap::check_map_bounds(double lat, double lon) const 
{
    auto [min_lat, max_lat, min_lon, max_lon] = PopulationMap::get_spatial_bounds();
    if (lat < min_lat) {return false;}
    if (lat > max_lat) {return false;}
    if (lon < min_lon) {return false;}
    if (lon > max_lon) {return false;}
        return true;
}
