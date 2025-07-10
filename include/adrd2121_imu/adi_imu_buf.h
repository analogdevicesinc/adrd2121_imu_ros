/*
 * Copyright (c) 2022, 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 */

#ifndef ADI_IMU_BUF_H
#define ADI_IMU_BUF_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdint.h>
#include <ros/console.h>

// Include adrd2121_imu headers
#include "adi_imu.h"
#include "imu_buf.h"

// Include adi_imu_driver headers
#include "adi_imu_driver.h"
#include "imu_spi_buffer.h"

// Include headers for custom msg and srv
#include "adrd2121_imu/AdiImu.h"
#include "adrd2121_imu/ImuGlobCmd.h"

//! \class AdiImuBuf
//! \brief Class for ADI IMU + ADRD2121
class AdiImuBuf
{
private:

  //! \brief ROS Publisher for IMU data
  ros::Publisher imu_pub_;

  //! \brief Nodehandle for ROS Publishers and ROS Service Server
  ros::NodeHandle nh_;

  //! \brief Local nodehandle for ROS Parameters
  ros::NodeHandle nh_local_;

  //! \brief ROS topic name for imu_pub_
  std::string topic_name_;

  //! \brief ROS frame name for imu_pub_
  std::string frame_name_;

  //! \brief ROS Service Service to send commands to GLOB_CMD register of ADI IMU
  ros::ServiceServer imu_glob_cmd_service_;

  //! \brief ROS Message type of imu_pub_
  int msg_type_;

  //! \brief ROS loop rate in Hz
  int ros_loop_rate_hz_;

  //! \brief Pointer to an ImuBuf object
  ImuBuf* p_imu_buf_;

  //! \brief Pointer to an AdiImu object
  AdiImu* p_adi_imu_;

  //! \brief Main struct for device
  adi_imu_Device_t imu_dev_;

  //! \brief Mode of Operation of Node
  mode_of_operation_e mode_of_operation_;

  //! \brief Boolean if Hardware is initialized
  bool b_hw_initialized_;

  //! \brief Timer for publishing IMU Data
  ros::Timer imu_pub_timer_callback_;

  //! \brief Boolean if reading and publishing data is successful
  bool b_read_pub_data_success_;

public:
  //! \brief Constructor for AdiImuBuf class
  //!
  //! \param[in] nodehandle   Pointer to nodehandle for nh_
  AdiImuBuf(ros::NodeHandle* nodehandle);

  //! \brief Destructor for AdiImuBuf class
  ~AdiImuBuf();

  //! \brief Loads Parameters of ROS-related parameters, ADI IMU (AdiImu), and  ADRD2121 (ImuBuf)
  void loadParams(void);

  //! \brief Initializes the hardware
  //! Initializes the (1) communication with the board (i.e. via USB), (2) ADRD2121, (3) ADI IMU, and
  //! (4) advertises ROS Publishers and Service Servers
  //!
  //! \return Boolean if successful (true) or not (false)
  bool init(void);

  //! \brief Conifgures the hardware based on the loaded parameters
  //! Configure the (1) ADRD2121, (2) ADI IMU
  //! Also, sets the appropriate ROS Loop rate
  //!
  //! \return Boolean if successful (true) or not (false)
  bool config(void);

  //! \brief Reads and Publishes IMU Data
  void readPubData(const ros::TimerEvent& event);

  //! \brief Getter of b_hw_initialized_
  //!
  //! \return Boolean based on b_hw_initialized_
  bool getHwInitialized(void);

  //! \brief Service Server callback for imu_glob_cmd_service_
  //!
  //! \param req,res  Request and Response to the Service
  //!
  //! \return Boolean if successful (true) or not (false)
  bool triggerImuGlobCmd(adrd2121_imu::ImuGlobCmd::Request &req, adrd2121_imu::ImuGlobCmd::Response &res);

  //! \brief Getter of mode_of_operation_
  //!
  //! \return Mode of Operation
  mode_of_operation_e getModeOfOperation();

  //! \brief Getter of b_read_pub_data_success_
  //!
  //! \return Boolean based on b_read_pub_data_success_
  bool getReadPubDataSuccess();
};

#endif //ADI_IMU_BUF_H
