/*
 * Copyright (c) 2022, 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 */

#include "adi_imu_buf.h"
#include <signal.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

void shutdown(void);
void gracefulShutdownHandler(int signal);

AdiImuBuf* p_adi_imu_buf = nullptr;
bool g_shutdown_signal = false;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "adrd2121_imu_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  signal(SIGINT, gracefulShutdownHandler);
  bool b_success = false;
  mode_of_operation_e mode_of_operation=STREAMING;

#if defined(ADRD2121_IMU_DEBUG)
  // To print ros debug messages
  ROSCONSOLE_AUTOINIT;

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
#endif

  // Create AdiImuBuf object
  p_adi_imu_buf = new AdiImuBuf(&nh);

  p_adi_imu_buf->loadParams();

  b_success=p_adi_imu_buf->init();

  // If initialization failed, check if HW is initialized
  // If yes, will still be allowed to run in RECOVERY mode
  if(!b_success)
  {
    if(!p_adi_imu_buf->getHwInitialized())
    {
      shutdown();
    }
  }

  mode_of_operation=p_adi_imu_buf->getModeOfOperation();
  if(STREAMING==mode_of_operation)
  {
    b_success=p_adi_imu_buf->config();
    if(!b_success)
    {
      shutdown();
    }

    while(ros::ok() && p_adi_imu_buf->getReadPubDataSuccess() && !g_shutdown_signal)
    {
      ROS_INFO_STREAM_ONCE("Node running at STREAMING mode...");
      ros::spinOnce();
    }
  }
  else if(RECOVERY==mode_of_operation)
  {
    while(ros::ok() && !g_shutdown_signal)
    {
      ROS_WARN_STREAM_ONCE("Node running at RECOVERY mode...");
      ros::spinOnce();
    }
  }

  // If goes out of while loop, shutdown
  shutdown();
}

void shutdown(void)
{
  ROS_INFO("===== SHUTDOWN CALLED ======");
  delete p_adi_imu_buf;
  p_adi_imu_buf=nullptr;
  ros::shutdown();
}

void gracefulShutdownHandler(int signal)
{
  ROS_INFO_STREAM("===== SIGINT RECEIVED ======");
  g_shutdown_signal=true;
}
