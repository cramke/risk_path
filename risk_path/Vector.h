#pragma once
#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/multi_array.hpp>

BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(cs::cartesian)

namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> point;
typedef bg::model::polygon<point> polygon;

class Vector
{
	polygon poly;
public:
	Vector(double ax, double ay, double bx, double by);
	bool within(double lat, double lon);
};

class RTree
{
public:
	RTree();
};

class GeoJsonReader
{
	std::string path;
	boost::property_tree::ptree root;
public:
	GeoJsonReader();
	std::vector<polygon> get_polygons();
};