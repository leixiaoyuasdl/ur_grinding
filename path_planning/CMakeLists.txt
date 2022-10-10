cmake_minimum_required(VERSION 3.2)
project(fanuc_grinding_path_planning)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
             bezier
             genmsg
             geometry_msgs
             moveit_ros_planning_interface
             roscpp
             rviz_visual_tools
             std_msgs
             tf_conversions
             visualization_msgs
             eigen_conversions)
find_package(KDL)
set(KDL /usr/local)

find_package(PCL 1.8.0 REQUIRED)
find_package(VTK 7.1.0 QUIET)
if (NOT VTK_FOUND)
  find_package(VTK 8.0.0 REQUIRED)
endif()
add_service_files(FILES PathPlanningService.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS
               bezier
               rviz_visual_tools
               geometry_msgs
               message_runtime
               std_msgs)

include_directories(${VTK_USE_FILE} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_executable(plytest src/test.cpp)
target_link_libraries(plytest ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

add_executable(${PROJECT_NAME} src/path_planning.cpp)
if(VTK_LIBRARIES) # May never happen because we expect VTK 7.0 minimum
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} bezier)
else()
  target_link_libraries (${PROJECT_NAME} ${catkin_LIBRARIES} vtkHybrid vtkWidgets ${PCL_LIBRARIES} bezier)
endif()

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
