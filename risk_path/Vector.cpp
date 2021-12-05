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
		rtree_double.insert(point);
	}
}

bool RTree::check_point(double lat, double lon)
{
	std::vector<value> result;
	point p(lon, lat);
	rtree.query(bg::index::contains(p), std::back_inserter(result));
	if (result.empty()) return true;
	else return false;
}

double RTree::nearest_point_cost(double lat, double lon)
{
	point p(lon, lat); 
	std::vector<population_point> result;
	rtree_double.query(bg::index::nearest(p, 1), std::back_inserter(result));
	return result[0].population;
}

double RTree::buffered_line_cost(const double* pos1, const double* pos2)
{
	point p1(pos1[1], pos1[0]);
	point p2(pos2[1], pos1[0]);
	bg::model::linestring<point> line{p1, p2};

	const double buffer_distance = 0.005;
	const int points_per_circle = 36;
	boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance);
	boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
	boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
	boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
	boost::geometry::strategy::buffer::side_straight side_strategy;

	bg::model::multi_polygon<polygon> buffered_line;
	bg::buffer(line, buffered_line, distance_strategy, side_strategy,
		join_strategy, end_strategy, circle_strategy);

	std::vector<std::pair<box, double>> result;

	rtree.query(bg::index::intersects(buffered_line), std::back_inserter(result));
	if (result.empty()) return 0.0;
	else
	{
		double cost = 0;
		for (std::pair<box, double> pair : result)
		{
			cost += pair.second;
		}
		return cost;
	};
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