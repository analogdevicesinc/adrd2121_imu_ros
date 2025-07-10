#include <gtest/gtest.h>
#include <iostream>
#include <log4cxx/logger.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdint.h>
#include <ros/console.h>
#include <vector>
#include <cmath>

class ImuDataRaw : public ::testing::Test{
  protected:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    std::vector<sensor_msgs::Imu> imu_data_;
    bool HasFatalFailure;

    void SetUp() override
    {
      sensor_msgs::Imu imu_msg_init;
      imu_msg_init.angular_velocity.x = 0.0;
      imu_msg_init.angular_velocity.y = 0.0;
      imu_msg_init.angular_velocity.z = 0.0;
      imu_msg_init.linear_acceleration.x = 0.0;
      imu_msg_init.linear_acceleration.y = 0.0;
      imu_msg_init.linear_acceleration.z = 0.0;
      imu_data_.push_back(imu_msg_init);
      imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data_raw",1000,&ImuDataRaw::imuCallback,this);
      ROS_INFO_STREAM("Wait 10 seconds...");
      ros::Duration(10).sleep();

      ROS_INFO_STREAM("[TEST] Subscribed to " << imu_sub_.getTopic());

      if(imu_sub_.getNumPublishers() != 1u)
      {
        HasFatalFailure = true;
        ROS_ERROR_STREAM("Subscriber not connected to any publisher.");
      }
      else
      {
        HasFatalFailure = false;
      }

    }

    void TearDown() override
    {
      imu_sub_.shutdown();
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
      ROS_DEBUG_STREAM("[TEST] imuCallback called");
      imu_data_.push_back(*msg);
    }

};


TEST_F(ImuDataRaw,countDataReceived)
{
  if (HasFatalFailure)
  {
    FAIL();
    return;
  }

  ros::Rate loopRate(50);
  for(size_t i = 0; i < 50; ++i)
  {
    ros::spinOnce();
    loopRate.sleep();
  }

  int imu_data_size = imu_data_.size();
  ROS_DEBUG_STREAM("[TEST] (countDataReceived) IMU Data Received " << imu_data_size);

  // Should be greater that 10
  // Vector would initially have 1 data with values of zero
  EXPECT_GT(imu_data_size,10);
}

TEST_F(ImuDataRaw,validateData)
{
  if (HasFatalFailure)
  {
    FAIL();
    return;
  }

  ROS_WARN_STREAM("[TEST] (validateData) This test assumes gravity 9.81 m/s and IMU is static)");
  ros::Rate loopRate(50);
  for(size_t i = 0; i < 50; ++i)
  {
    ros::spinOnce();
    loopRate.sleep();
  }

  sensor_msgs::Imu last_imu_data = imu_data_.back();

  ROS_DEBUG_STREAM("[TEST] Linear_acceleration.x " << last_imu_data.linear_acceleration.x);
  ROS_DEBUG_STREAM("[TEST] Linear_acceleration.y " << last_imu_data.linear_acceleration.y);
  ROS_DEBUG_STREAM("[TEST] Linear_acceleration.z " << last_imu_data.linear_acceleration.z);

  ROS_DEBUG_STREAM("[TEST] Angular_velocity.x " << last_imu_data.angular_velocity.x);
  ROS_DEBUG_STREAM("[TEST] Angular_velocity.y " << last_imu_data.angular_velocity.y);
  ROS_DEBUG_STREAM("[TEST] Angular_velocity.z " << last_imu_data.angular_velocity.z);

  EXPECT_NEAR(0.0, std::abs(last_imu_data.linear_acceleration.x), 0.5);
  EXPECT_NEAR(0.0, std::abs(last_imu_data.linear_acceleration.y), 0.5);
  EXPECT_NEAR(9.81, std::abs(last_imu_data.linear_acceleration.z), 0.5);

  EXPECT_NEAR(0.0, std::abs(last_imu_data.angular_velocity.x), 0.5);
  EXPECT_NEAR(0.0, std::abs(last_imu_data.angular_velocity.y), 0.5);
  EXPECT_NEAR(0.0, std::abs(last_imu_data.angular_velocity.z), 0.5);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "test_imu_data_raw");
  ros::NodeHandle nh;

  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  ROS_DEBUG_STREAM("[TEST] Execute tests.");

  testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();

  return res;
}
