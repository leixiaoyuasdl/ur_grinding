cmake_minimum_required(VERSION 3.2)
project(ur_grinding_rviz_plugin)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
                    class_loader
                    fanuc_grinding_alignment
                    fanuc_grinding_comparison
                    fanuc_grinding_path_planning
                    fanuc_post_processor
                    fanuc_grinding_scanning
                    rviz)


## Find Qt5 modules
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

qt5_wrap_cpp(
  MOCS_FILES
  src/alignment_widget.h
  src/comparison_widget.h
  src/${PROJECT_NAME}.h
  src/path_planning_widget.h
  src/post_processor_widget.h
  src/scanning_widget.h
)

add_library(
  ${PROJECT_NAME}
  src/alignment_widget.cpp
  src/comparison_widget.cpp
  src/${PROJECT_NAME}.cpp
  src/path_planning_widget.cpp
  src/post_processor_widget.cpp
  src/scanning_widget.cpp
  ${MOCS_FILES}
)
target_link_libraries(
  ${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Gui
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}
  fanuc_grinding_scanning_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

class_loader_hide_library_symbols(
  ${PROJECT_NAME}
)

## Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

