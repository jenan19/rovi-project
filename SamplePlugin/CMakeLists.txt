# ##################################################################################################
# Template for building RobWork dependent projects - the src should contain code for putting into
# libraries - the plugins contain code for multiple RobWorkStudio plugins - the test for testing -
# the example dir for illustrativ examples of your project
#
# Use config.cmake to change default settings and edit this file for adding additional dependencies
# to external projects
# ##################################################################################################

#
# Test CMake version
#
cmake_minimum_required(VERSION 3.10)

# The name of the project.
project(RoViPlugin)

# Set build type to release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
message("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
find_package(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)

find_package(Eigen3 REQUIRED)

link_directories(${Eigen_INCLUDE_DIRS})

# Use OpenCV 4
find_package(OpenCV 4 REQUIRED)


find_package(Qt6 COMPONENTS Core Widgets Charts QUIET)
# If we want to use ui files add them here
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)


# Need to add the current binary dir to the include directory because the generated source files are
# placed here
include_directories(${CMAKE_CURRENT_BINARY_DIR})

include_directories(${CMAKE_SOURCE_DIR}/src/)



add_executable(hsvfilter src/hsvfilter.cpp )
target_link_libraries(hsvfilter ${OpenCV_LIBS} ${HEADER_FILES} ${Eigen_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES}) 

add_executable(planning src/planning.cpp )
target_link_libraries(planning ${OpenCV_LIBS} ${HEADER_FILES} ${Eigen_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${Qt6Core_LIBRARIES} ${Qt6Widgets_LIBRARIES} ${Qt6Charts_LIBRARIES}) 

# The shared library to build:
add_library(${PROJECT_NAME} MODULE src/SamplePlugin.cpp src/resources.qrc)
target_link_libraries(
    ${PROJECT_NAME} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${HEADER_FILES} ${OpenCV_LIBS}
)
