# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(t265_mavconn)

set(APRILTAG_INC /usr/local/include/apriltag)
set(APRILTAG_LIB /usr/local/lib/libapriltag.so)

find_path(APRILTAG_INC NAMES apriltag.h PATH_SUFFIXES apriltag)
find_library(APRILTAG_LIB apriltag)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(apriltag DEFAULT_MSG APRILTAG_INC APRILTAG_LIB)

add_library(apriltag INTERFACE)
target_include_directories(apriltag INTERFACE ${APRILTAG_INC})
target_link_libraries(apriltag INTERFACE ${APRILTAG_LIB})

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
add_executable(t265_mav_tag t265_mav_tag.cpp mavconn_msg.cpp pose_apriltag.cpp)

# Link your application with OpenCV libraries
target_link_libraries(t265_mavconn LINK_PRIVATE realsense2 ${OpenCV_LIBS} ${libmavconn_LIBRARIES})
target_link_libraries(t265_mav_tag LINK_PRIVATE realsense2 ${OpenCV_LIBS} ${libmavconn_LIBRARIES} apriltag)
#target_link_libraries(t265_mav_tag LINK_PRIVATE realsense2 ${OpenCV_LIBS} ${libmavconn_LIBRARIES} apriltag -lpthread)
