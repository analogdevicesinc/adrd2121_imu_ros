/*
 * Copyright (c) 2022, 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 */

#include "adrd2121_imu/imu_state_checker.h"

#include <ros/console.h>
#include <cmath>
#include <string>
#include <sstream>

ImuStateChecker::ImuStateChecker():
    nh_local_("~")
{
  ROS_INFO("[IMU State] Constructed");
}

ImuStateChecker::~ImuStateChecker()
{
  ROS_INFO("[IMU State] Destructed");
}

bool ImuStateChecker::initialize(void)
{
  bool b_success=false;
  b_success = this->loadParams();
  // Load Parameters
  if(b_success)
  {
    if(SENSOR_MSGS_IMU==imu_msg_type_)
    {
      imu_sub_ = nh_.subscribe<adrd2121_imu::AdiImu>(imu_topic_name_,1000,&ImuStateChecker::imuCallback, this);
      ROS_INFO_STREAM("[IMU State] Subscribed to "<< imu_topic_name_ << " with msg type adrd2121_imu/AdiImu");
    }
    else if(ADI_IMU_MSG==imu_msg_type_)
    {
      imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(imu_topic_name_,1000,&ImuStateChecker::imuCallback, this);
      ROS_INFO_STREAM("[IMU State] Subscribed to "<< imu_topic_name_ << " with msg type sensor_msgs/Imu");
    }
    imu_state_pub_ = nh_.advertise<adrd2121_imu::ImuState>("imu_state", 1000);
    prev_state_=MOVING;
  }

  return b_success;
}

void ImuStateChecker::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  state_=this->evaluateState(*msg);
  this->evaluateStandstillBegin();
}

void ImuStateChecker::imuCallback(const adrd2121_imu::AdiImu::ConstPtr& msg)
{
  sensor_msgs::Imu imu;
  imu.angular_velocity = msg->angular_velocity;
  imu.linear_acceleration = msg->linear_acceleration;
  state_=this->evaluateState(imu);
  this->evaluateStandstillBegin();
}

e_imu_state ImuStateChecker::evaluateState(sensor_msgs::Imu imu_msg)
{
  bool gyro_standstill=false;
  bool accl_standstill=false;
  e_imu_state state;
  float gyro_std=0.0;
  float accl_std=0.0;
  // Get gyro Magnitude
  float gyro_mag=sqrt((imu_msg.angular_velocity.x*imu_msg.angular_velocity.x)+\
    (imu_msg.angular_velocity.y*imu_msg.angular_velocity.y)+(imu_msg.angular_velocity.z*imu_msg.angular_velocity.z));

  // Get accl Magnitude
  float accl_mag=sqrt((imu_msg.linear_acceleration.x*imu_msg.linear_acceleration.x)+\
    (imu_msg.linear_acceleration.y*imu_msg.linear_acceleration.y)+\
    (imu_msg.linear_acceleration.z*imu_msg.linear_acceleration.z));

  // Make sure only the last 10 Magnitude are stored in the deque
  if(gyro_mag_q_.size()>9)
    gyro_mag_q_.pop_front();

  gyro_mag_q_.push_back(gyro_mag);

  if(accl_mag_q_.size()>9)
    accl_mag_q_.pop_front();

  accl_mag_q_.push_back(accl_mag);

  // Calculate standard deviation of gyro and acc magnitude
  gyro_std=this->getStandardDev(gyro_mag_q_);
  accl_std=this->getStandardDev(accl_mag_q_);

  // Note: If the Standard deviation are less than threshold, then assume standstill.
  // These threshold are manually tested;
  // the lower the value, the more sensitive (i.e. slight IMU movement may be considered as MOVING already)
  if(gyro_std < gyro_std_thresh_)
    gyro_standstill=true;

  if(accl_std < accl_std_thresh_)
    accl_standstill=true;

  if(accl_standstill && gyro_standstill)
  {
    state=STANDSTILL;
  }
  else
  {
    state=MOVING;
  }
  return state;
}

float ImuStateChecker::getStandardDev(std::deque<float> data)
{
  int size_data=data.size();
  float sum = 0.0;
  float mean = 0.0;
  float var = 0.0;
  float std = 0.0;

  // get Sum of data
  for(int i=0; i<size_data; i++)
  {
    sum+=data[i];
  }
  // get Mean of data
  mean=sum/size_data;

  // Get variance
  for(int i=0; i<size_data; i++)
  {
    var+=pow(data[i] - mean,2);
  }
  // get standard deviation
  std = sqrt(var/size_data);
  return std;
}

void ImuStateChecker::evaluateStandstillBegin(void)
{
  adrd2121_imu::ImuState imu_state;
  std::ostringstream ss;
  ros::Time now=ros::Time::now();
  std::string s_prev_state, s_state;

  if(prev_state_==MOVING && state_==STANDSTILL)
  {
    ROS_DEBUG("IMU State: moving to standstill");
    standstill_begin_=ros::Time::now();
  }

  s_prev_state = (prev_state_==MOVING)? "MOVING":"STANDSTILL";
  s_state = (state_==MOVING)? "MOVING":"STANDSTILL";
  ss << "Previous State: "<< s_prev_state << " to Current State: "<< s_state;

  // Set current state as previous state
  prev_state_=state_;

  // Update and publish topic
  imu_state.header.stamp = ros::Time::now();
  imu_state.message=ss.str();
  imu_state.state= s_state;
  imu_state.stand_still_secs = now.toSec() - standstill_begin_.toSec();
  imu_state_pub_.publish(imu_state);
}

ros::Time ImuStateChecker::getStandstillBegin(void)
{
  return standstill_begin_;
}

e_imu_state ImuStateChecker::getState(void)
{
  return state_;
}

bool ImuStateChecker::loadParams(void)
{
  bool b_default = false;
  bool b_success = false;
  ROS_INFO("Loading IMU State Checker Parameters");

  // Default values were based on manual testing.
  b_default = nh_local_.param("gyro_std_thresh", gyro_std_thresh_, 0.02);
  ROS_DEBUG_STREAM_COND(b_default, "[IMU State] gyro_std_thresh: " << gyro_std_thresh_);
  ROS_WARN_STREAM_COND(!b_default, "[IMU State] Using default gyro_std_thresh:" << gyro_std_thresh_);

  b_default = nh_local_.param("accl_std_thresh", accl_std_thresh_, 0.08);
  ROS_DEBUG_STREAM_COND(b_default, "[IMU State] accl_std_thresh: " << accl_std_thresh_);
  ROS_WARN_STREAM_COND(!b_default, "[IMU State] Using default accl_std_thresh:" << accl_std_thresh_);

  b_default = nh_local_.param<std::string>("imu_topic_name", imu_topic_name_, "/imu/data_raw");
  ROS_DEBUG_STREAM_COND(b_default, "[IMU State] Subscribed to IMU Topic: " << imu_topic_name_);
  ROS_WARN_STREAM_COND(!b_default, "[IMU State] Subscribed to default IMU Topic: " << imu_topic_name_);

  b_default = nh_local_.param("imu_topic_delay", imu_topic_delay_, 20.0);
  ROS_DEBUG_STREAM_COND(b_default, "[IMU State] imu_topic_delay: " << imu_topic_delay_ << "seconds");
  ROS_WARN_STREAM_COND(!b_default, "[IMU State] Using default imu_topic_delay: " << imu_topic_delay_);

  // Get Topic of imu_topic
  ROS_INFO_STREAM("[IMU State] Wait for "<<imu_topic_delay_<<"... Waiting for IMU topic to be published...");
  ros::Duration(imu_topic_delay_).sleep(); // Delay before checking the ROS topics being published
  ROS_INFO_STREAM("[IMU State] Checking all topics published ...");
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
  {
    const ros::master::TopicInfo& info = *it;
    ROS_DEBUG_STREAM("Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << \
      info.datatype << std::endl);
    if(imu_topic_name_.compare(info.name) == 0)
    {
      if(info.datatype.compare("adrd2121_imu/AdiImu"))
      {
        imu_msg_type_ = ADI_IMU_MSG;
      }
      else if(info.datatype.compare("sensor_msgs/Imu"))
      {
        imu_msg_type_ = SENSOR_MSGS_IMU;
      }
      b_success = true;
    }
  }

  ROS_ERROR_STREAM_COND(!b_success,"[IMU State] Cannot subscribe to " << imu_topic_name_ << "; Not published.");

  return b_success;
}
