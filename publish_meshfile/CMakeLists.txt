cmake_minimum_required(VERSION 3.2)
project(fanuc_grinding_publish_meshfile)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
             genmsg
          pcl_ros
             roscpp
             std_msgs
             visualization_msgs)

find_package(PCL 1.8.0 REQUIRED)
message(leixiaoyu ${pcl_ros_INCLUDE_DIRS})
add_service_files(FILES PublishMeshfileService.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
               message_runtime
               std_msgs)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/publish_meshfile.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

