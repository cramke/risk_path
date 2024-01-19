cmake_minimum_required(VERSION 3.0.0)
project(risk_path VERSION 0.1.0 LANGUAGES C CXX)

find_package(ompl REQUIRED)
find_package(gdal REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS} 
    ${GDAL_INCLUDE_DIRS}
)

include(CTest)
enable_testing()

add_executable(risk_path src/main.cpp)
target_link_libraries(risk_path ${OMPL_LIBRARIES} ${GDAL_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
