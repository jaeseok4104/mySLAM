CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( SLAM ) 

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###########################dependencies##########################

find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIR})

find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

FIND_PACKAGE(PCL REQUIRED COMPONENTS common io visualization filters)
include_directories(${PCL_INCLUDE_DIRS})
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED)
# CSparse
FIND_PACKAGE( CSparse )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )
#eigen

include_directories( "/usr/include/eigen3/" )


set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${realsense2_LIBRARY}
    ${PCL_LIBRARIES}
    ${Sophus_LIBRARIES}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY}
)

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)
add_subdirectory( src )
add_subdirectory( test )
