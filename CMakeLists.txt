# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(t265_mavconn)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

## System dependencies are found with CMake's conventions
find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(libmavconn_DIR /usr/local/share/libmavconn/cmake)
find_package(libmavconn REQUIRED)

message(STATUS "libmavconn library status:")
message(STATUS "    config: ${libmavconn_DIR}")
message(STATUS "    version: ${libmavconn_VERSION}")
message(STATUS "    libraries: ${libmavconn_LIBRARIES}")
message(STATUS "    include path: ${libmavconn_INCLUDE_DIRS}")

find_package(iir)

# add package modules path, not needed in dependend packages
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
include(EnableCXX11)
include(MavrosMavlink)

###########
## Build ##
###########

set(OBJS
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/obj.o
)

# Declare the executable target built from your sources
add_executable(t265_mavconn t265_mavconn.cpp mavconn_msg.cpp)

# Link your application with OpenCV libraries
target_link_libraries(t265_mavconn LINK_PRIVATE realsense2 ${OpenCV_LIBS} ${libmavconn_LIBRARIES} iir::iir)
