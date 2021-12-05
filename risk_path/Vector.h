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

typedef bg::model::d2::point_xy<double> point;
typedef bg::model::polygon<point> polygon;
typedef bg::model::box<point> box;
typedef std::pair<box, unsigned> value; 

struct point_with_double
{
	double lon, lat;
	double population;
};

BOOST_GEOMETRY_REGISTER_POINT_2D(point_with_double, double, cs::cartesian, lon, lat)

class RTreeBox
{
public:
	bg::index::rtree<value, bg::index::rstar<16, 4>> rtree;
	bg::index::rtree<point_with_double, bg::index::rstar<16, 4>> rtree_double;
	RTreeBox(std::vector<polygon> polygons);
	RTreeBox(std::vector<point_with_double> points);
	bool check_point(double lat, double lon);
	double nearest_point_cost(double lat, double lon);
	double buffered_line_cost(const double* pos1, const double* pos2);
};

class RTreePoint
{
public:
	bg::index::rtree<point_with_double, bg::index::rstar<16, 4>> rtree;
	RTreePoint(std::vector<point_with_double> points);
	double nearest_point_cost(double lat, double lon);
	double buffered_line_cost(const double* pos1, const double* pos2);
};

class GeoJsonReader
{
	std::string path;
	boost::property_tree::ptree root;
public:
	GeoJsonReader(std::string);
	GeoJsonReader(const char*);
	std::vector<polygon> get_polygons();
	std::vector<point_with_double> get_points();
};