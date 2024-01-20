find_package(ompl REQUIRED)
find_package(GDAL REQUIRED)
find_package(Boost 1.74 REQUIRED )

include_directories(
    ${OMPL_INCLUDE_DIRS} 
    ${GDAL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

file(GLOB SRCS "*.cpp")
add_library(planning 
    ${SRCS}
)
target_link_libraries(planning
    ${OMPL_LIBRARIES}
    ${GDAL_LIBRARIES}
    ${Boost_LIBRARIES}
)   
