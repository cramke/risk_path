#include "Vector.h"

Vector::Vector(double ax, double ay, double bx, double by)
{
	polygon poly;
	const double coor[][2] = { {ax, ay}, {ax, by}, {bx, by}, {bx, ay}, {ax, ay} };
	bg::assign_points(poly, coor);
}

