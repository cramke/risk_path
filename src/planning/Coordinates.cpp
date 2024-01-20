#include "Coordinates.hpp"

Coordinates::Coordinates(int x_given, int y_given, std::shared_ptr<PopulationMap> map_given) :
    x(x_given), y(y_given), transform(map_given->transform)
{
    index_to_spatial_coordinates();
}

Coordinates::Coordinates(double lat_given, double lon_given, std::shared_ptr<PopulationMap> map_given) :
    lat(lat_given), lon(lon_given), transform(map_given->transform)
{
    spatial_to_index_coordinates();
}

void  Coordinates::index_to_spatial_coordinates() 
{
    // https://gdal.org/tutorials/geotransforms_tut.html
    lon = transform[0] + transform[1] * x + transform[2] * y;
    lat = transform[3] + transform[4] * x + transform[5] * y;
}

void Coordinates::spatial_to_index_coordinates() 
{
    double sub_x = ((lon - transform[0]) - (transform[2] * 0)) / transform[1];
    double sub_y = ((lat - transform[3]) - (transform[4] * 0)) / transform[5];
    x = int(std::round(sub_x));
    y = int(std::round(sub_y));
}