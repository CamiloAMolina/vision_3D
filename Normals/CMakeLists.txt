cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(normales_test1)

find_package(PCL 1.8.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (normals_test3 normals_test3.cpp)
target_link_libraries (normals_test3 ${PCL_LIBRARIES})
#add_executable (normales_test2 normales_test2.cpp)
#target_link_libraries (normales_test2 ${PCL_LIBRARIES})

#add_executable (normales_test1 normales_test1.cpp)
#target_link_libraries (normales_test1 ${PCL_LIBRARIES})
