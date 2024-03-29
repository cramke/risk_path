# risk_path

risk_path is a library which automatically plans a 2D path between a start and a goal, while minimizing a risk function.

## Purpose

This repository is primarily a private project just for fun. It is currently not intended to be used with other projects or in real life. I am also in the process of learning C++, where I choose this project over a course. 

## Background

The idea is to find optimal paths for drones to fly over mostly uninhabited areas. If the drone should crash, a minimal amount of people would be at risk. The path planning should happen automatically with no human input, besides defining a start and a goal.

## Thanks

This repos makes heavy usage of the following libraries. The list is probably not complete. 
- [OMPL](https://ompl.kavrakilab.org/): The Open Motion Planning Library
- [GDAL](https://gdal.org/): Geospatial Data Abstraction Library
- Boost
   - [Boost.Geometry](https://www.boost.org/doc/libs/1_77_0/libs/geometry/doc/html/geometry/introduction.html)
