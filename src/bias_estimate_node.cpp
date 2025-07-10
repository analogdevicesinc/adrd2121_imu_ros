/*
 * Copyright (c) 2022, 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 */

#include "adrd2121_imu/bias_estimate_node.h"

#include <ros/console.h>
#include <cmath>
#include <string>
#include <sstream>

BiasEstimate::BiasEstimate() :
  imu_state_checker_()
{
  ROS_INFO("[Bias Estimate] Constructed");
}

BiasEstimate::~BiasEstimate()
{
    ROS_INFO("[Bias Estimate] Destructed");
}

bool BiasEstimate::initialize(void)
{
  bool b_success=false;
  b_success=imu_state_checker_.initialize();
  if(b_success)
  {
    trigger_imu_glob_cmd_ = nh_.serviceClient<adrd2121_imu::ImuGlobCmd>("trigger_imu_glob_cmd");
    ROS_INFO_STREAM("[Bias Estimate] Connected to "<<trigger_imu_glob_cmd_.getService() << "service.");
    bias_estimate_service_ = nh_.advertiseService("bias_estimate", &BiasEstimate::biasEstimateCB,this);
  }
  return b_success;
}

bool BiasEstimate::biasEstimateCB(adrd2121_imu::BiasEstimateCmd::Request &req, \
    adrd2121_imu::BiasEstimateCmd::Response &res)
{
  ros::Time standstill_begin;
  bool b_standstill_duration=false;
  double duration_in_secs = 0;
  std::ostringstream ss;

  if(imu_state_checker_.getState() == STANDSTILL)
  {
    standstill_begin=imu_state_checker_.getStandstillBegin();
    b_standstill_duration=this->checkStandstillDuration(standstill_begin, duration_in_secs);
    if(b_standstill_duration)
    {
      ROS_INFO("[Bias Estimate] Trigger IMU bias correction update (trigger_imu_glob_cmd_)");
      adrd2121_imu::ImuGlobCmd data;
      data.request.bit = 0;
      trigger_imu_glob_cmd_.call(data);

      res.success=data.response.success;
      if(res.success)
      {
        res.message="Bias Correction Update successfully triggered.";
      }
      else
      {
        res.message="Bias Correction Update unsuccessful.";
      }
    }
    else
    {
      ss << "[Bias Estimate] Make sure IMU is standstill for at least 40s. Standstill time: "<<duration_in_secs<<"s";
      ROS_WARN_STREAM(ss.str());
      res.success=false;
      res.message=ss.str();
    }
    res.state = "STANDSTILL";
    res.stand_still_secs = duration_in_secs;
  }
  else
  {
    ss << " [Bias Estimate] Make sure IMU is standstill for at least 40s. IMU is detected as MOVING";
    ROS_WARN_STREAM(ss.str());
    res.success=false;
    res.message=ss.str();
    res.state = "MOVING";
    res.stand_still_secs = duration_in_secs;
  }


  return true;
}

bool BiasEstimate::checkStandstillDuration(ros::Time standstill_begin, double &duration_in_secs)
{
  bool b_success=false;
  ros::Time now=ros::Time::now();

  duration_in_secs = now.toSec() - standstill_begin.toSec();
  ROS_INFO("[Bias Estimate] Standstill duration: %f s", duration_in_secs);

  // Set that the IMU should be Standstill for at least 40 s
  if(duration_in_secs>=40)
  {
    b_success=true;
  }

  return b_success;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"bias_estimate_node");

  BiasEstimate bias_estimate;
  if(bias_estimate.initialize())
  {
    ros::spin();
  }

  return 0;
}
