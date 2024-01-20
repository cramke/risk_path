#pragma once
#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/multi_array.hpp>

namespace bg = boost::geometry;

BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(cs::cartesian)

using point = bg::model::d2::point_xy<double>;
using polygon = bg::model::polygon<point>;
using box = bg::model::box<point>;
using value = std::pair<box, unsigned>; 

struct point_with_double
{
	double lon;
	double lat;
	double population;
};

BOOST_GEOMETRY_REGISTER_POINT_2D(point_with_double, double, cs::cartesian, lon, lat)

class RTreeBox
{
public:
	bg::index::rtree<value, bg::index::rstar<16, 4>> rtree;
	explicit RTreeBox(const std::vector<polygon> & polygons);
	bool check_point(double lat, double lon) const;
};

class RTreePoint
{
public:
	bg::index::rtree<point_with_double, bg::index::rstar<16, 4>> rtree;
	explicit RTreePoint(const std::vector<point_with_double> & points);
	double nearest_point_cost(double lat, double lon) const;
	double buffered_point_cost(const double* pos) const;
	bg::model::multi_polygon<polygon> buffer_point(const point &p) const;
	bg::model::multi_polygon<polygon> buffer_line(const bg::model::linestring<point> &line) const;
	double buffered_line_cost(const double* pos1, const double* pos2) const;
};

class GeoJsonReader
{
	std::string path;
	boost::property_tree::ptree root;
public:
	explicit GeoJsonReader(const char*);
	std::vector<polygon> get_polygons();
	std::vector<point_with_double> get_points();
};