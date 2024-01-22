cmake_minimum_required(VERSION 3.22.0)
project(risk_path VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(ompl REQUIRED)
find_package(GDAL REQUIRED)
find_package(Boost 1.74 REQUIRED )

set(PLANNING_SRCS
    "src/planning/Coordinates.cpp"
    "src/planning/CustomOptimizationObjective.cpp"
    "src/planning/Population.cpp"
    "src/planning/risk_path.cpp"
    "src/planning/Vector.cpp"
)
set(PLANNING_HEADERS
    "src/planning/Coordinates.hpp"
    "src/planning/Population.h"
    "src/planning/risk_path.h"
    "src/planning/Vector.h"
)

include_directories(${OMPL_INCLUDE_DIRS} 
    ${GDAL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${Boost_INCLUDE_DIRS}
    src/planning
)

add_library(planning SHARED
    ${PLANNING_SRCS}
    ${PLANNING_HEADERS}
)

target_link_libraries(planning
    ${OMPL_LIBRARIES}
    ${GDAL_LIBRARIES}
    ${Boost_LIBRARIES}
)  

add_executable(risk_path src/main.cpp)
target_link_libraries(risk_path ${OMPL_LIBRARIES} ${GDAL_LIBRARIES} planning)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

## UnitTests
enable_testing() #to  discover tests in test explorer 

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

file(GLOB TESTS "tests/*.cpp")
add_executable(
    test_risk_path
    ${TESTS}
)

target_link_libraries(
    test_risk_path
    GTest::gtest_main
    ${OMPL_LIBRARIES}
    ${GDAL_LIBRARIES}
    planning
)

include(GoogleTest)
gtest_discover_tests(test_risk_path)
