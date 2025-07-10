/*
 * Copyright (c) 2022, 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 */

#ifndef IMU_STATE_CHECKER_H
#define IMU_STATE_CHECKER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <deque>

// Include headers for custom msg
#include <adrd2121_imu/ImuState.h>
#include "adrd2121_imu/AdiImu.h"

// Include adrd2121_imu headers
#include "adrd2121_imu_common.h"

//! \enum e_imu_state
//! State of IMU
typedef enum e_imu_state
{
  STANDSTILL = 0,
  MOVING = 1
} e_imu_state;

//! \class ImuStateChecker
//! \brief Checks state of IMU (STANDSTILL or MOVING)
class ImuStateChecker
{
private:
  //! \brief Node Handle
  ros::NodeHandle nh_;

  //! \brief Local Node Handle
  ros::NodeHandle nh_local_;

  //! \brief Subscriber to IMU topic
  ros::Subscriber imu_sub_;

  //! \brief Publisher of IMU state topic
  ros::Publisher imu_state_pub_;

  //! \brief Current state of IMU
  e_imu_state state_;

  //! \brief Previous state of IMU
  e_imu_state prev_state_;

  //! \brief Time start when IMU is STANDSTILL
  ros::Time standstill_begin_;

  //! \brief Contains the recent 10 magnitude of Gyroscopre data
  std::deque<float> gyro_mag_q_;

  //! \brief Contains the recent 10 magnitude of Accelerometer data
  std::deque<float> accl_mag_q_;

  //! \brief Gyroscope Standard Deviation threshold
  double  gyro_std_thresh_;

  //! \brief Accelerometer Standard Deviation threshold
  double  accl_std_thresh_;

  //! \brief Imu message type
  int  imu_msg_type_;

  //! \brief Imu topic name
  std::string imu_topic_name_;

  //! \brief Delay before subscribing to IMU Topic
  double imu_topic_delay_;
public:
  //! \brief Constructor for StateChecker
  ImuStateChecker(void);

  //! \brief Destructor for StateChecker
  ~ImuStateChecker(void);

  //! \brief Evaluates the state of the IMU based on the standard deviation of the magnitude
  //!
  //! \param[in] imu_msg  The IMU data that will be evaluated
  //!
  //! \return state of the IMU
  e_imu_state evaluateState(sensor_msgs::Imu imu_msg);

  //! \brief Callback when IMU topic is received
  //!
  //! \param[in] msg  The IMU data
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  //! \brief Callback when IMU topic is received
  //!
  //! \param[in] msg  The IMU data
  void imuCallback(const adrd2121_imu::AdiImu::ConstPtr& msg);

  //! \brief Calculates the standard deviation
  //!
  //! \param[in] data  The deque of data
  //!
  //! \return The computed standard deviation
  float getStandardDev(std::deque<float> data);

  //! \brief Gets the Time Start when IMU is STANDSTILL
  void evaluateStandstillBegin(void);

  //! \brief Returns the Time Start when IMU is STANDSTILL
  //!
  //! \return value of standstill_begin_
  ros::Time getStandstillBegin(void);

  //! \brief Returns the current state
  //!
  //! \return value of standstill_begin_
  e_imu_state getState(void);

  //! \brief Loads parameters for the IMU state checker
  //!
  //! \return Boolean if successful (true) or not (false)
  bool loadParams(void);

  //! \brief Initialize
  //!
  //! \return Boolean if successful (true) or not (false)
  bool initialize(void);

};

#endif //IMU_STATE_CHECKER_H
