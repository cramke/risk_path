# risk_path

risk_path is a library which automatically plans a 2D path between a start and a goal, while minimizing a risk function.

## Purpose

This repository is primarily a private project just for fun. It is currently not intended to be used with other projects or in real life. I am also in the process of learning C++, where I choose this project over a course. 

## Background

The idea is to find optimal paths for drones to fly over mostly uninhabited areas. If the drone should crash, a minimal amount of people would be at risk. The path planning should happen automatically with no human input, besides defining a start and a goal.

## Thanks

This repos makes heavy usage of the following libraries. The list is probably not complete. 
- [OMPL](https://ompl.kavrakilab.org/): The Open Motion Planning LibraryOMPL
- [Gdal](https://gdal.org/): Geospatial Data Abstraction Library
- Boost
   - [Boost.Geometry](https://www.boost.org/doc/libs/1_73_0/libs/geometry/doc/html/geometry/introduction.html#:~:text=Boost.Geometry%20%28aka%20Generic%20Geometry%20Library%2C%20GGL%29%2C%20part%20of,kernel%2C%20based%20on%20concepts%2C%20meta-functions%20and%20tag%20dispatching.)
