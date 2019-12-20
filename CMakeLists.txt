cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(prm VERSION 1.0)
configure_file(prmConfig.h.in prmConfig.h)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find and include OpenCV library
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# bring the headers into the project
include_directories(include)

# add class library
include_directories(class)
link_directories(class)
add_subdirectory(class)

# add the executable
set(SOURCE_FILES prm.cpp)
add_executable( ${PROJECT_NAME} ${SOURCE_FILES} )

target_include_directories(prm PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           )
                           
target_link_libraries( ${PROJECT_NAME} class)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

