#include "Coordinates.hpp"

Coordinates::Coordinates(int x_given, int y_given, std::array<double, 6> transform) : x(x_given), y(y_given), transform(transform)
{
    index_to_spatial_coordinates();
}

Coordinates::Coordinates(double lat_given, double lon_given, std::array<double, 6> transform) : lat(lat_given), lon(lon_given), transform(transform)
{
    spatial_to_index_coordinates();
}

/**
 * @brief Converts the index coordinates to spatial coordinates
 *
 * GT(0) x-coordinate of the upper-left corner of the upper-left pixel.
 * GT(1) w-e pixel resolution / pixel width.
 * GT(2) row rotation (typically zero).
 * GT(3) y-coordinate of the upper-left corner of the upper-left pixel.
 * GT(4) column rotation (typically zero).
 * GT(5) n-s pixel resolution / pixel height (negative value for a north-up image).
 */
void Coordinates::index_to_spatial_coordinates()
{
    // https://gdal.org/tutorials/geotransforms_tut.html
    lon = transform[0] + transform[1] * x + transform[2] * y;
    lat = transform[3] + transform[4] * x + transform[5] * y;
}

/**
 * @brief Converts the spatial coordinates to index coordinates
 *
 * Inverse of index_to_spatial_coordinates()
 */
void Coordinates::spatial_to_index_coordinates()
{
    double sub_x = ((lon - transform[0]) - (transform[2] * 0)) / transform[1];
    double sub_y = ((lat - transform[3]) - (transform[4] * 0)) / transform[5];
    x = int(std::round(sub_x));
    y = int(std::round(sub_y));
}
