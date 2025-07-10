cmake_minimum_required(VERSION 3.0.2)
project(adrd2121_imu)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
)

#################################
## Getting User Configurations ##
#################################

option(ADRD2121_IMU_DEBUG "Show ROS Debug Stream of adrd2121_imu" OFF)
# If built without any ADRD2121 and IMU, assumes no HW is connected to Host, purely debugging of flow only
option(ADRD2121_IMU_WITH_HW "Build adrd2121_imu with Hardware (i.e. ADRD2121 <-> IMU)" ON)
option(ADRD2121_IMU_TIMING_DEBUG "Show Timing debug" OFF)

MESSAGE(STATUS "ADRD2121_IMU_DEBUG=${ADRD2121_IMU_DEBUG}")
MESSAGE(STATUS "ADRD2121_IMU_WITH_HW=${ADRD2121_IMU_WITH_HW}")
MESSAGE(STATUS "ADRD2121_IMU_TIMING_DEBUG=${ADRD2121_IMU_TIMING_DEBUG}")

IF(ADRD2121_IMU_DEBUG)
add_definitions("-DADRD2121_IMU_DEBUG")
ENDIF(ADRD2121_IMU_DEBUG)
IF(ADRD2121_IMU_WITH_HW)
add_definitions("-DADRD2121_IMU_WITH_HW")
ENDIF(ADRD2121_IMU_WITH_HW)
IF(ADRD2121_IMU_TIMING_DEBUG)
add_definitions("-DADRD2121_IMU_TIMING_DEBUG")
ENDIF(ADRD2121_IMU_TIMING_DEBUG)

message(STATUS "> Current Source Dir ${CMAKE_CURRENT_SOURCE_DIR}")

###################################
## External Project Dependencies ##
###################################

message(STATUS "> Download, build, and install driver dependencies...")

include(ExternalProject)
ExternalProject_Add(adi_imu_driver
  PREFIX              ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/adi_imu_driver
  GIT_REPOSITORY      https://github.com/analogdevicesinc/adi-imu-driver.git
  GIT_TAG             compact_imu
  GIT_SHALLOW         true
  CONFIGURE_COMMAND   bash -c "mkdir -p ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/adi_imu_driver"
  BUILD_COMMAND       cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_C_FLAGS_RELEASE=-fPIC -DCMAKE_CXX_FLAGS_RELEASE=-fPIC <SOURCE_DIR>
  INSTALL_COMMAND     make -j2
  TEST_COMMAND        echo ">>> adi_imu_driver dirs: " COMMAND echo <SOURCE_DIR> COMMAND echo <BINARY_DIR>
  LOG_DOWNLOAD        1
  LOG_BUILD           1
  LOG_INSTALL         1
)

ExternalProject_Get_Property(adi_imu_driver SOURCE_DIR)
message(STATUS "  adi_imu_driver's source_dir is: " ${SOURCE_DIR})

ExternalProject_Get_Property(adi_imu_driver BINARY_DIR)
message(STATUS "  adi_imu_driver's binary_dir is: " ${BINARY_DIR})

# Set library location of adi_imu_driver
set(ADI_IMU_DRIVER_IMU_LIB ${BINARY_DIR}/lib/imu/libadimu.so)
set(ADI_IMU_DRIVER_BUF_LIB ${BINARY_DIR}/lib/imu_buf/libadimubuf.so)

# Set include directories of adi_imu_driver
set(ADI_IMU_DRIVER_INCLUDE ${SOURCE_DIR}/include)
set(ADI_IMU_DRIVER_IMU_LIB_INCLUDE ${SOURCE_DIR}/lib/imu)
set(ADI_IMU_DRIVER_BUF_LIB_INCLUDE ${SOURCE_DIR}/lib/imu_buf)

# Create target for the ROS Driver and add dependency to the libraries built before this
add_library (adi_imu_driver_imu_target SHARED IMPORTED)
set_target_properties (adi_imu_driver_imu_target PROPERTIES IMPORTED_LOCATION ${ADI_IMU_DRIVER_IMU_LIB})
add_dependencies(adi_imu_driver_imu_target adi_imu_driver)

add_library (adi_imu_driver_imubuf_target SHARED IMPORTED)
set_target_properties (adi_imu_driver_imubuf_target PROPERTIES IMPORTED_LOCATION ${ADI_IMU_DRIVER_BUF_LIB})
add_dependencies(adi_imu_driver_imubuf_target adi_imu_driver)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AdiImu.msg
  ImuState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ImuGlobCmd.srv
  BiasEstimateCmd.srv
  BufStatus.srv
)

## Generate added messages with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adrd2121_imu_common adi_imu_buf adi_imu imu_buf adi_imu_driver_imu_target adi_imu_driver_imubuf_target
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs std_srvs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  include/adrd2121_imu
  ${catkin_INCLUDE_DIRS}
  ${ADI_IMU_DRIVER_INCLUDE}
  ${ADI_IMU_DRIVER_IMU_LIB_INCLUDE}
  ${ADI_IMU_DRIVER_BUF_LIB_INCLUDE}
)

## Declare a C++ library
add_library(adi_imu_buf src/adi_imu_buf.cpp)
add_library(adi_imu src/adi_imu.cpp)
add_library(imu_buf src/imu_buf.cpp)
add_library(adrd2121_imu_common src/adrd2121_imu_common.cpp)

add_dependencies(adi_imu_buf adi_imu_driver_imu_target adrd2121_imu_generate_messages_cpp)
add_dependencies(adi_imu adi_imu_driver_imu_target)
add_dependencies(imu_buf adi_imu_driver_imu_target adi_imu_driver_imubuf_target adrd2121_imu_generate_messages_cpp)
add_dependencies(adrd2121_imu_common adi_imu_driver_imu_target adi_imu_driver_imubuf_target)

## Declare a C++ executable
add_executable(adrd2121_imu_node src/adrd2121_imu_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(adrd2121_imu_node ${adrd2121_imu_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(adi_imu adrd2121_imu_common ${ADI_IMU_DRIVER_IMU_LIB})

target_link_libraries(imu_buf adrd2121_imu_common ${ADI_IMU_DRIVER_IMU_LIB} ${ADI_IMU_DRIVER_BUF_LIB})

target_link_libraries(adi_imu_buf adi_imu imu_buf ${ADI_IMU_DRIVER_IMU_LIB})

target_link_libraries(adrd2121_imu_node adi_imu_buf ${catkin_LIBRARIES})

## Bias Estimate Node

 add_library(imu_state_checker src/imu_state_checker.cpp)
 add_dependencies(imu_state_checker adrd2121_imu_common adrd2121_imu_generate_messages_cpp)

 add_executable(bias_estimate_node src/bias_estimate_node.cpp)
 add_dependencies(bias_estimate_node ${adrd2121_imu_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(bias_estimate_node imu_state_checker adrd2121_imu_common ${catkin_LIBRARIES})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

 find_package(rostest REQUIRED)

 catkin_add_gtest(test_data test/test_imu_data_raw.cpp)
 target_link_libraries(test_data ${catkin_LIBRARIES} ${rostest_LIBRARIES})

 catkin_add_gtest(test_param test/test_param.cpp)
 target_link_libraries(test_param ${catkin_LIBRARIES} ${rostest_LIBRARIES})

endif()

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS
  adrd2121_imu_node
  bias_estimate_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libaries for installation
install(TARGETS
  adi_imu_buf
  adi_imu
  imu_buf
  adrd2121_imu_common
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY
  config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(FILES
  LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
