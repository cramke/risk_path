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

RTree::RTree(std::vector<polygon> polygons)
{
	int id = 0;
	for (polygon poly : polygons)
	{
		box b = bg::return_envelope<box>(poly);
		auto pair = std::make_pair(b, id);
		rtree.insert(pair);
		id++;
	}
	
};


GeoJsonReader::GeoJsonReader()
{
	path = "C:/Users/carst/OneDrive/Projekte/risk-path/risk_path/maps/test.geojson";
	boost::property_tree::read_json(path, root);
}

std::vector<polygon> GeoJsonReader::get_polygons()
{
	std::vector<polygon> polygons;
	for (auto& feature : root.get_child("features"))
	{
		polygon poly;
		std::vector<point> polygon_vector;
		for (auto& fourth : feature.second.get_child("geometry.coordinates"))
		{
			for (auto& third : fourth.second)
			{
				for (auto& row : third.second)
				{
					std::vector<double> point_vector;
					for (auto& cell : row.second)
					{
						point_vector.push_back(std::stod(cell.second.data()));
					}
					point p1(point_vector[0], point_vector[1]);
					polygon_vector.push_back(p1);
				}
			}
		}
		bg::assign_points(poly, polygon_vector);
		polygons.push_back(poly);
	}
	return polygons;
}
