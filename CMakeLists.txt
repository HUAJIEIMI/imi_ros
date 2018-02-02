cmake_minimum_required(VERSION 2.8.3)
project(imi_camera)

find_package(catkin REQUIRED dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_generation)

find_package(Boost REQUIRED COMPONENTS system thread)

SET(IMI_SDK_DIR imiLinuxSDK)

include(ExternalProject)

ExternalProject_Add(imi_sdk
  PREFIX imi_sdk
  GIT_REPOSITORY https://github.com/HUAJIEIMI/imi_linux_sdk.git
  CONFIGURE_COMMAND echo "no need to configure"
  BUILD_IN_SOURCE 0
  BUILD_COMMAND echo "no need to build"
  INSTALL_DIR ${IMI_SDK_DIR}
  INSTALL_COMMAND tar -zxvf <SOURCE_DIR>/imiLinuxSDK_noSkeleton.tar.gz -C <INSTALL_DIR> --strip 1
)

generate_dynamic_reconfigure_options(
   cfg/Imi.cfg
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imi_wrapper
  CATKIN_DEPENDS dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_runtime
)


SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

link_directories(${CMAKE_CURRENT_BINARY_DIR}/${IMI_SDK_DIR}/lib)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_BINARY_DIR}/${IMI_SDK_DIR}/include
                    )


add_library(imi_wrapper
   src/imi_Device.cpp
   src/imi_Driver.cpp
)

add_dependencies(imi_wrapper imi_sdk)
add_dependencies(imi_wrapper imi_camera_gencfg)

target_link_libraries(imi_wrapper ${catkin_LIBRARIES} -liminect -lImiCamera
  ${Boost_LIBRARIES} )

add_executable(ros_imi_node
   ros/ros_imi_node.cpp
)

target_link_libraries(ros_imi_node ${catkin_LIBRARIES} -liminect imi_wrapper -lImiCamera
  ${Boost_LIBRARIES} )

install(TARGETS imi_wrapper ros_imi_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add xml file
install(FILES imi_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
