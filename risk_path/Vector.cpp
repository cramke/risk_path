#include "Vector.h"
// https://stackoverflow.com/questions/55608080/how-to-store-some-information-at-internal-nodes-of-r-tree

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

RTree::RTree(std::vector<population_point> points) 
{
	for (population_point point : points)
	{
		box b = bg::return_envelope<box>(point);
		auto pair = std::make_pair(b, point.population);
		rtree.insert(pair);
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

GeoJsonReader::GeoJsonReader(const char* path)
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

std::vector<population_point> GeoJsonReader::get_points()
{
	std::vector<population_point> points;
	for (auto& feature : root.get_child("features"))
	{
		population_point point;

		point.population = std::stod(feature.second.get_child("properties.POP_1").data());
		std::vector<double> coordinates;
		for (auto& geo_coords : feature.second.get_child("geometry.coordinates"))
		{
			double single_coordinate = std::stod(geo_coords.second.data());
			coordinates.push_back(single_coordinate);
		}
		point.lon = coordinates[0];
		point.lat = coordinates[1];
		points.push_back(point);
	}
	return points;
}
