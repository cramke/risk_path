#include "Vector.h"

Vector::Vector(double ax, double ay, double bx, double by)
{
	const double coor[][2] = { {ax, ay}, {ax, by}, {bx, by}, {bx, ay}, {ax, ay} };
	bg::assign_points(poly, coor);
}

bool Vector::within(double lat, double lon)
{
	point p(lat, lon);
	return bg::within(p, poly);
}


GeoJsonReader::GeoJsonReader()
{
	path = "C:/Users/carst/OneDrive/Projekte/risk-path/risk_path/maps/test.geojson";
	boost::property_tree::ptree root;
	boost::property_tree::read_json(path, root);


}
