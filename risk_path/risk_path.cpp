#include <iostream>
#include "Population.cpp"

int main() {
	PopulationMap map = PopulationMap();
	Coordinates point = Coordinates(52.4693, 13.3804, map.transform);
	double expected_lat = 55.056805555555556;
	double expected_lon = 5.8676388888888891;
	std::cout << "Lat: " << point.x << std::endl;
	std::cout << "Lon: " << point.y << std::endl;
};