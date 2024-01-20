#include "Vector.h"
// https://stackoverflow.com/questions/55608080/how-to-store-some-information-at-internal-nodes-of-r-tree

RTreeBox::RTreeBox(const std::vector<polygon> & polygons)
{
	const int DUMMY_ID = 0;
	for (polygon poly : polygons)
	{
		box b = bg::return_envelope<box>(poly);
		value box_id_pair = std::make_pair(b, DUMMY_ID);
		rtree.insert(box_id_pair);
	}	
}

bool RTreeBox::check_point(double lon, double lat) const
{
	std::vector<value> result;
	point p(lon, lat);
	rtree.query(bg::index::contains(p), std::back_inserter(result));
	if (result.empty()) 
		return true;
	else 
		return false;
}

GeoJsonReader::GeoJsonReader(const char* path)
{
	boost::property_tree::read_json(path, root);
}

std::vector<polygon> GeoJsonReader::get_polygons()
{
	std::vector<polygon> polygons;
	for (const auto& [key, value] : root.get_child("features"))
	{
		std::vector<point> polygon_points;
		for (const auto& [_, coordinates] : value.get_child("geometry.coordinates"))
		{
			for (const auto& [key3, value3] : coordinates)
			{
				assert(key3.empty()); // array elements have no names
				auto size = std::distance(value3.begin(), value3.end());
				assert(size == 2);
				double x = std::stod(value3.front().second.data());
				double y = std::stod(value3.back().second.data());
				point p1(x, y);
				polygon_points.push_back(p1);
			}
		}
		polygon poly;
		bg::assign_points(poly, polygon_points);
		polygons.push_back(poly);
	}
	return polygons;
}

std::vector<point_with_double> GeoJsonReader::get_points()
{
	std::vector<point_with_double> points;
	for (const auto& [_, feature] : root.get_child("features"))
	{
		point_with_double point;
		point.population = std::stod(feature.get_child("properties.POP_1").data());
		int i = 0;
		for (const auto& [_, geo_coords] : feature.get_child("geometry.coordinates"))
		{
			if (i == 0) 
				point.lon = std::stod(geo_coords.data());
			else if (i == 1) 
				point.lat = std::stod(geo_coords.data());
			i++;
		}
		points.push_back(point);
	}
	return points;
}

RTreePoint::RTreePoint(const std::vector<point_with_double> & points)
{
	for (const point_with_double &point : points)
	{
		rtree.insert(point);
	}
}

double RTreePoint::nearest_point_cost(double lon, double lat) const
{
	point p(lon, lat);
	std::vector<point_with_double> result;
	rtree.query(bg::index::nearest(p, 1), std::back_inserter(result));
	return result[0].population;
}

double RTreePoint::buffered_point_cost(const double* pos) const
{
	bg::model::multi_polygon<polygon> buffer;
	point p(pos[0], pos[1]);
	buffer = buffer_point(p);	

	std::vector<point_with_double> result;
	rtree.query(bg::index::intersects(buffer), std::back_inserter(result));
	double cost = std::accumulate(	result.begin(),
									result.end(),
									0.0,
									[](double sum, const point_with_double& curr) {return sum + curr.population; });
	return cost;
}

bg::model::multi_polygon<polygon> RTreePoint::buffer_point(const point &p) const
{
	const double buffer_distance = 0.005;
	const int points_per_circle = 8;
	boost::geometry::strategy::buffer::distance_symmetric distance_strategy(buffer_distance);
	boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
	boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
	boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
	boost::geometry::strategy::buffer::side_straight side_strategy;

	bg::model::multi_polygon<polygon> buffer;
	bg::buffer(p, buffer, distance_strategy, side_strategy,
		join_strategy, end_strategy, circle_strategy);
	return buffer;
}

bg::model::multi_polygon<polygon> RTreePoint::buffer_line(const bg::model::linestring<point> &line) const
{
	const double buffer_distance = 0.005;
	const int points_per_circle = 8;
	boost::geometry::strategy::buffer::distance_symmetric distance_strategy(buffer_distance);
	boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
	boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
	boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
	boost::geometry::strategy::buffer::side_straight side_strategy;

	bg::model::multi_polygon<polygon> buffered_line;
	bg::buffer(line, buffered_line, distance_strategy, side_strategy,
		join_strategy, end_strategy, circle_strategy);
	return bg::model::multi_polygon<polygon>();
}

double RTreePoint::buffered_line_cost(const double* pos1, const double* pos2) const
{
	point p1(pos1[0], pos1[1]);
	point p2(pos2[0], pos1[1]);
	bg::model::linestring<point> line{ p1, p2 };
	bg::model::multi_polygon<polygon> buffered_line = buffer_line(line);	

	std::vector<point_with_double> result;
	rtree.query(bg::index::intersects(buffered_line), std::back_inserter(result));
	double cost = std::accumulate(	result.begin(),
									result.end(),
									0.0,
									[](double sum, const point_with_double& curr) {return sum + curr.population; });
	return cost;
}