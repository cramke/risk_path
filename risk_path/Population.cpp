#include "Population.h"


PopulationMap::PopulationMap() {
    filename = "C:/Users/carst/OneDrive/Projekte/risk-path/risk_path/maps/pop_deu.tif";
    GDALAllRegister();
    dataset = (GDALDataset*)GDALOpen(filename, GA_ReadOnly);
    if (dataset != NULL)
    {
        dataset->GetGeoTransform(transformer);
        transform_array();
        band = dataset->GetRasterBand(1);
        nXSize = band->GetXSize();
        nYSize = band->GetYSize();
        scanline = (float*)CPLMalloc(sizeof(float) * nXSize);
        has_file_loaded = true;
    }
    else
    {
        printf("The dataset failed to open. Maybe check the filename?");
        throw std::runtime_error("File not found or file is empty");
    }
}

void PopulationMap::transform_array()
{
    for (int i = 0; i < 6; i++) {
        transform[i] = transformer[i];
    }
    check_transform();
}

double PopulationMap::read_population_from_indexes(int x, int y) const
{
    band->RasterIO(GF_Read, x, y, 1, 1, scanline, 1, 1, GDT_Float32, 0, 0);
    return (double)scanline[0];
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
    else {
        return false;
    }
}

std::tuple<double, double, double, double> PopulationMap::get_spatial_bounds() const {
    double upper_lat = transform[3];
    double lower_lat = upper_lat + nXSize * transform[5]; // transform[5] is negativ
    double lower_lon = transform[0];
    double upper_lon = lower_lon + nYSize * transform[1];
    return std::make_tuple(lower_lat, upper_lat, lower_lon, upper_lon);
}

bool PopulationMap::check_map_bounds(double lat, double lon) const {
    auto bounds = PopulationMap::get_spatial_bounds();
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







Coordinates::Coordinates(int x_given, int y_given, std::shared_ptr<PopulationMap> map_given)
{
    transform = map_given->transform;
    x = x_given;
    y = y_given;
    std::tuple<double, double> spatial = index_to_spatial_coordinates(x, y);
    lat = std::get<0>(spatial);
    lon = std::get<1>(spatial);
}

Coordinates::Coordinates(double lat_given, double lon_given, std::shared_ptr<PopulationMap> map_given) {
    transform = map_given->transform;
    lat = lat_given;
    lon = lon_given;
    std::tuple<int, int> index = spatial_to_index_coordinates(lat, lon);
    x = std::get<0>(index);
    y = std::get<1>(index);
}

std::tuple<double, double>  Coordinates::index_to_spatial_coordinates(int xd, int yd) {
    // https://gdal.org/tutorials/geotransforms_tut.html
    double lon = transform[0] + transform[1] * xd + transform[2] * yd;
    double lat = transform[3] + transform[4] * xd + transform[5] * yd;
    return std::make_tuple(lat, lon);
}

std::tuple<int, int> Coordinates::spatial_to_index_coordinates(double lat, double lon) {
    double x = ((lon - transform[0]) - (transform[2] * 0)) / transform[1];
    double y = ((lat - transform[3]) - (transform[4] * 0)) / transform[5];
    int x_index = int(std::round(x));
    int y_index = int(std::round(y));
    return std::make_tuple(x_index, y_index);
}