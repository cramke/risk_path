#include "Vector.h"
// https://stackoverflow.com/questions/55608080/how-to-store-some-information-at-internal-nodes-of-r-tree

RTreeBox::RTreeBox(std::vector<polygon> polygons)
{
	int id = 0;
	for (polygon poly : polygons)
	{
		box b = bg::return_envelope<box>(poly);
		value pair = std::make_pair(b, id);
		rtree.insert(pair);
		id++;
	}	
}

bool RTreeBox::check_point(double lat, double lon)
{
	std::vector<value> result;
	point p(lon, lat);
	rtree.query(bg::index::contains(p), std::back_inserter(result));
	if (result.empty()) return true;
	else return false;
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

std::vector<point_with_double> GeoJsonReader::get_points()
{
	std::vector<point_with_double> points;
	for (auto& feature : root.get_child("features"))
	{
		point_with_double point;
		point.population = std::stod(feature.second.get_child("properties.POP_1").data());
		int i = 0;
		for (auto& geo_coords : feature.second.get_child("geometry.coordinates"))
		{
			if (i == 0) point.lon = std::stod(geo_coords.second.data());
			else if (i == 1) point.lat = std::stod(geo_coords.second.data());
			i++;
		}
		points.push_back(point);
	}
	return points;
}

RTreePoint::RTreePoint(std::vector<point_with_double> points)
{
	for (const point_with_double& point : points)
	{
		rtree.insert(point);
	}
}

double RTreePoint::nearest_point_cost(double lat, double lon)
{
	point p(lon, lat);
	std::vector<point_with_double> result;
	rtree.query(bg::index::nearest(p, 1), std::back_inserter(result));
	return result[0].population;
}

double RTreePoint::buffered_line_cost(const double* pos1, const double* pos2)
{
	point p1(pos1[1], pos1[0]);
	point p2(pos2[1], pos1[0]);
	bg::model::linestring<point> line{ p1, p2 };

	const double buffer_distance = 0.005;
	const int points_per_circle = 8;
	boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance);
	boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
	boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
	boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
	boost::geometry::strategy::buffer::side_straight side_strategy;

	bg::model::multi_polygon<polygon> buffered_line;
	bg::buffer(line, buffered_line, distance_strategy, side_strategy,
		join_strategy, end_strategy, circle_strategy);

	std::vector<point_with_double> result;

	rtree.query(bg::index::intersects(buffered_line), std::back_inserter(result));
	double cost = std::accumulate(result.begin(),
		result.end(),
		0.0,
		[](double sum, const point_with_double& curr) {return sum + curr.population; });
	return cost;
}