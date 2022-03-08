cmake_minimum_required(VERSION 3.2)
project(fanuc_grinding_comparison)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
             genmsg
             roscpp
             std_msgs)

add_service_files(FILES ComparisonService.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
               message_runtime
               std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/comparison.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

