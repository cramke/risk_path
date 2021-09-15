#include "Vector.h"

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
}

bool RTree::check_point(double lat, double lon)
{
	std::vector<value> result;
	point p(lon, lat);
	rtree.query(bg::index::contains(p), std::back_inserter(result));
	if (result.empty())
	{
		return true;
	}
	else
	{
		return false;
	}
}

GeoJsonReader::GeoJsonReader(std::string path)
{
	boost::property_tree::read_json(path, root);
	std::cout << "Completed Loading Vector File" << std::endl;
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
			
					std::vector<double> point_vector;
					for (auto& cell : third.second)
					{
						point_vector.push_back(std::stod(cell.second.data()));
					}
					point p1(point_vector[0], point_vector[1]);
					polygon_vector.push_back(p1);
				
			}
		}
		bg::assign_points(poly, polygon_vector);
		polygons.push_back(poly);
	}
	return polygons;
}
