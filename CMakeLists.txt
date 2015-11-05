cmake_minimum_required(VERSION 2.8)
project(velma_robrex)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros rtt_rosclock std_msgs interactive_markers tf eigen_conversions cmake_modules collision_convex_model kin_dyn_model planer_utils octomap_msgs barrett_hand_controller_msgs
  )

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(OMPL REQUIRED)
find_package(octomap REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

add_executable(robrex_get_jar src/get_jar.cpp src/velma_dyn_model.cpp src/output_ikfast_velma_right.cpp src/experiments_utilities.cpp src/ompl_utilities.cpp src/velma_robrex_utilities.cpp)
add_executable(robrex_get_jar_hw src/get_jar_hw.cpp src/velma_dyn_model.cpp src/output_ikfast_velma_right.cpp
                src/experiments_utilities.cpp src/ompl_utilities.cpp src/velma_robrex_utilities.cpp src/barrett_hand_interface.cpp
                src/velma_interface.cpp)

target_link_libraries(robrex_get_jar ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_link_libraries(robrex_get_jar_hw ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${OCTOMAP_LIBRARIES})

catkin_package(
    DEPENDS ompl eigen
    CATKIN_DEPENDS collision_convex_model kin_dyn_model planer_utils
)

