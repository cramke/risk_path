cmake_minimum_required(VERSION 3.0.0)
project(risk_path VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(ompl REQUIRED)
find_package(GDAL REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS} 
    ${GDAL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB SOURCES "src/*.cpp")

include(CTest)
enable_testing()

add_executable(risk_path src/main.cpp ${SOURCES})
target_include_directories(risk_path PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(risk_path ${OMPL_LIBRARIES} ${GDAL_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
