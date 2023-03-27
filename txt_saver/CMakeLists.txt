cmake_minimum_required(VERSION 3.0.2)
project(txt_saver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  rospy
  serial
  std_msgs
)

#add_message_files(FILES control2serial.msg fromERP.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime roscpp rospy std_msgs)

catkin_install_python(PROGRAMS 
	src/txt_saver_for_kcity.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
 )

 catkin_install_python(PROGRAMS 
	scripts/map_recorder.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
 )

 catkin_install_python(PROGRAMS 
	scripts/imu_node.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
 )

 catkin_install_python(PROGRAMS 
 scripts/ebimu_driver.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

catkin_install_python(PROGRAMS 
scripts/erp_driver.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

catkin_install_python(PROGRAMS 
scripts/erp_dummy_state_publisher.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

catkin_install_python(PROGRAMS 
scripts/odometry_manager.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

catkin_install_python(PROGRAMS 
scripts/pub_example.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

catkin_install_python(PROGRAMS 
scripts/sub_example.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

catkin_install_python(PROGRAMS 
scripts/erp_commander.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

catkin_install_python(PROGRAMS 
scripts/erp_serial_publisher.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)


install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(${catkin_INCLUDE_DIRS})

# add_executable(kbub_localization src/kbub_localization.cpp)
#add_dependencies(txt_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
#target_link_libraries(txt_saver ${catkin_LIBRARIES}   ${Eigen_LIBRARIES}
 # )
