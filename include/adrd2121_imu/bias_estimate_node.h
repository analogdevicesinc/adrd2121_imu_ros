/*
 * Copyright (c) 2022, 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 */

#ifndef BIAS_ESTIMATE_NODE_H
#define BIAS_ESTIMATE_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>

// Include headers for custom msg and srv
#include "adrd2121_imu/ImuGlobCmd.h"
#include "adrd2121_imu/BiasEstimateCmd.h"

// Include adrd2121_imu headers
#include "adrd2121_imu/imu_state_checker.h"

//! \class BiasEstimate
//! \brief Class for Bias Estimate Service
class BiasEstimate
{
private:
  //! \brief Node Handle
  ros::NodeHandle nh_;

  //! \brief Service Server for the /bias_estimate
  ros::ServiceServer bias_estimate_service_;

  //! \brief Service client that subscribes to /trigger_glob_cmd service
  ros::ServiceClient trigger_imu_glob_cmd_;

  //! \brief Object for ImuStateChecker
  ImuStateChecker imu_state_checker_;

  //! \brief Time start when IMU is STANDSTILL
  ros::Time standstill_duration_;

public:
  //! \brief Constructor for BiasEstimate
  BiasEstimate(void);

  //! \brief Destructor for BiasEstimate
  ~BiasEstimate(void);

  //! \brief Initialize
  //!
  //! \return Boolean if successful (true) or not (false)
  bool initialize(void);

  //! \brief Callback when /bias_estimate is called
  //!
  //! \param[in] req Request
  //! \param[in]  res Result
  //!
  //! \return Boolean if successful (true) or not (false)
  bool biasEstimateCB(adrd2121_imu::BiasEstimateCmd::Request &req, \
    adrd2121_imu::BiasEstimateCmd::Response &res);

  //! \brief Check if Standstill for a specific time period
  //!
  //! \param[in] standstill_begin   The start time of standstill
  //!
  //! \return Boolean if successful (true) or not (false)
  bool checkStandstillDuration(ros::Time standstill_begin, double &duration_in_secs);
};

#endif //BIAS_ESTIMATE_NODE_H
