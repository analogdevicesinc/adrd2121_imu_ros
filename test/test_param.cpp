#include <gtest/gtest.h>
#include <iostream>
#include <log4cxx/logger.h>
#include <ros/ros.h>
#include <stdint.h>
#include <ros/console.h>
#include <vector>
#include "adi_imu_buf.h"

// void validateStrParam(std::string rosparam_name, std::string expected_value);

class AdiImuParams : public ::testing::Test{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  std::string nh_namespace_;
  bool HasFatalFailure;
  int prod_under_test_;

  AdiImuParams(): nh_local_("~")
  {
    nh_namespace_ = nh_.getNamespace();
    nh_local_.getParam("prod_under_test", prod_under_test_);
  }

  void validateParam(std::string rosparam_name, std::string expected_value)
  {
    std::string input_value;

    bool has_param = nh_.hasParam(nh_namespace_ + rosparam_name);
    EXPECT_TRUE(has_param);

    if(has_param)
    {
      nh_.getParam(nh_namespace_ + rosparam_name, input_value);
      EXPECT_STREQ(input_value.c_str(), expected_value.c_str());
    }
  }

  void validateParam(std::string rosparam_name, bool expected_value)
  {
    bool input_value;

    bool has_param = nh_.hasParam(nh_namespace_ + rosparam_name);
    EXPECT_TRUE(has_param);

    if(has_param)
    {
      nh_.getParam(nh_namespace_ + rosparam_name, input_value);
      EXPECT_EQ(input_value, expected_value);
    }
  }

  void validateParam(std::string rosparam_name, int expected_value)
  {
    int input_value;

    bool has_param = nh_.hasParam(nh_namespace_ + rosparam_name);
    EXPECT_TRUE(has_param);

    if(has_param)
    {
      nh_.getParam(nh_namespace_ + rosparam_name, input_value);
      EXPECT_EQ(input_value, expected_value);
    }
  }

  void validateParam(std::string rosparam_name, double expected_value)
  {
    double input_value;

    bool has_param = nh_.hasParam(nh_namespace_ + rosparam_name);
    EXPECT_TRUE(has_param);

    if(has_param)
    {
      nh_.getParam(nh_namespace_ + rosparam_name, input_value);
      EXPECT_EQ(input_value, expected_value);
    }
  }

  void validateParam(std::string rosparam_name, std::vector<int> expected_value)
  {
    std::vector<int> input_value;

    bool has_param = nh_.hasParam(nh_namespace_ + rosparam_name);
    EXPECT_TRUE(has_param);

    if(has_param)
    {
      nh_.getParam(nh_namespace_ + rosparam_name, input_value);
      EXPECT_EQ(input_value[0], expected_value[0]);
      EXPECT_EQ(input_value[1], expected_value[1]);
      EXPECT_EQ(input_value[2], expected_value[2]);
    }
  }

  void getExpectedParamVal(std::string rosparam_name, std::string& expected_value)
  {
    bool b_success;
    b_success = nh_local_.getParam("expected" + rosparam_name, expected_value);
    if(!b_success)
    {
      HasFatalFailure = true;
      ROS_ERROR_STREAM("Please provide expected" << rosparam_name << " value");
      FAIL();
    }

  }

  void getExpectedParamVal(std::string rosparam_name, bool& expected_value)
  {
    bool b_success;
    b_success = nh_local_.getParam("expected" + rosparam_name, expected_value);
    if(!b_success)
    {
      HasFatalFailure = true;
      ROS_ERROR_STREAM("Please provide expected" << rosparam_name << " value");
      FAIL();
    }
  }

  void getExpectedParamVal(std::string rosparam_name, int& expected_value)
  {
    bool b_success;
    b_success = nh_local_.getParam("expected" + rosparam_name, expected_value);
    if(!b_success)
    {
      HasFatalFailure = true;
      ROS_ERROR_STREAM("Please provide expected" << rosparam_name << " value");
      FAIL();
    }
  }

  void getExpectedParamVal(std::string rosparam_name, double& expected_value)
  {
    bool b_success;
    b_success = nh_local_.getParam("expected" + rosparam_name, expected_value);
    if(!b_success)
    {
      HasFatalFailure = true;
      ROS_ERROR_STREAM("Please provide expected" << rosparam_name << " value");
      FAIL();
    }
  }

  void getExpectedParamVal(std::string rosparam_name, std::vector<int>& expected_value)
  {
    bool b_success;
    b_success = nh_local_.getParam("expected" + rosparam_name, expected_value);
    if(!b_success)
    {
      HasFatalFailure = true;
      ROS_ERROR_STREAM("Please provide expected" << rosparam_name << " value");
      FAIL();
    }

    ROS_DEBUG_STREAM_COND(b_success,"expected" << rosparam_name << " value = \
      ["<< expected_value[0] << "," << expected_value[1] << "," << \
      expected_value[2] << "]");
  }

};

TEST_F(AdiImuParams,validateTopicName)
{
  std::string expected_value;
  getExpectedParamVal("/topic_name", expected_value);
  if(!HasFatalFailure)
    validateParam("/topic_name", expected_value);
}

TEST_F(AdiImuParams,validateFrameId)
{
  std::string expected_value;
  getExpectedParamVal("/frame_name", expected_value);
  if(!HasFatalFailure)
    validateParam("/frame_name", expected_value);
}

TEST_F(AdiImuParams,validateMsgType)
{
  int expected_value;
  getExpectedParamVal("/msg_type", expected_value);
  if(!HasFatalFailure)
    validateParam("/msg_type", expected_value);
}

TEST_F(AdiImuParams,validateModeOfOperation)
{
  int expected_value;
  getExpectedParamVal("/mode_of_operation", expected_value);
  if(!HasFatalFailure)
    validateParam("/mode_of_operation", expected_value);
}

TEST_F(AdiImuParams,validateUsbDev)
{
  std::string expected_value;
  getExpectedParamVal("/usb_dev", expected_value);
  if(!HasFatalFailure)
    validateParam("/usb_dev", expected_value);
}

TEST_F(AdiImuParams,validateUsbBaud)
{
  int expected_value;
  getExpectedParamVal("/usb_baud", expected_value);
  if(!HasFatalFailure)
    validateParam("/usb_baud", expected_value);
}

TEST_F(AdiImuParams,validateEnableInitRecovery)
{
  bool expected_value;
  getExpectedParamVal("/enable_init_recovery", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_init_recovery", expected_value);
}

TEST_F(AdiImuParams,validateBufBurstCount)
{
  int expected_value;
  getExpectedParamVal("/buf_burst_count", expected_value);
  if(!HasFatalFailure)
    validateParam("/buf_burst_count", expected_value);
}

TEST_F(AdiImuParams,validateEnableImuBurst)
{
  bool expected_value;
  getExpectedParamVal("/enable_imu_burst", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_imu_burst", expected_value);
}

TEST_F(AdiImuParams,validateBufOverflow)
{
  int expected_value;
  getExpectedParamVal("/buf_overflow", expected_value);
  if(!HasFatalFailure)
    validateParam("/buf_overflow", expected_value);
}

TEST_F(AdiImuParams,validateEnableBufPps)
{
  bool expected_value;
  getExpectedParamVal("/enable_buf_pps", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_buf_pps", expected_value);
}

TEST_F(AdiImuParams,validateBufDataReadySel)
{
  int expected_value;
  getExpectedParamVal("/buf_overflow", expected_value);
  if(!HasFatalFailure)
    validateParam("/buf_overflow", expected_value);
}

TEST_F(AdiImuParams,validateBufDataReadyPol)
{
  int expected_value;
  getExpectedParamVal("/buf_data_rdy_pol", expected_value);
  if(!HasFatalFailure)
    validateParam("/buf_data_rdy_pol", expected_value);
}

TEST_F(AdiImuParams,validateBufPpsSel)
{
  int expected_value;
  getExpectedParamVal("/buf_pps_sel", expected_value);
  if(!HasFatalFailure)
    validateParam("/buf_pps_sel", expected_value);
}

TEST_F(AdiImuParams,validateBufPpsPol)
{
  int expected_value;
  getExpectedParamVal("/buf_pps_pol", expected_value);
  if(!HasFatalFailure)
    validateParam("/buf_pps_pol", expected_value);
}

TEST_F(AdiImuParams,validateBufPpsFreq)
{
  int expected_value;
  getExpectedParamVal("/buf_pps_freq", expected_value);
  if(!HasFatalFailure)
    validateParam("/buf_pps_freq", expected_value);
}

TEST_F(AdiImuParams,validateBufPinPass)
{
  int expected_value;
  getExpectedParamVal("/buf_pin_pass", expected_value);
  if(!HasFatalFailure)
    validateParam("/buf_pin_pass", expected_value);
}

TEST_F(AdiImuParams,validateBufWatermarkInt)
{
  int expected_value;
  getExpectedParamVal("/buf_watermark_int", expected_value);
  if(!HasFatalFailure)
    validateParam("/buf_watermark_int", expected_value);
}

TEST_F(AdiImuParams,validateBufPOverflowInt)
{
  int expected_value;
  getExpectedParamVal("/buf_overflow_int", expected_value);
  if(!HasFatalFailure)
    validateParam("/buf_overflow_int", expected_value);
}

TEST_F(AdiImuParams,validateBufErrorInt)
{
  int expected_value;
  getExpectedParamVal("/buf_error_int", expected_value);
  if(!HasFatalFailure)
    validateParam("/buf_error_int", expected_value);
}

TEST_F(AdiImuParams,validateClearBufferTimeout)
{
  int expected_value;
  getExpectedParamVal("/clear_buffer_timeout", expected_value);
  if(!HasFatalFailure)
    validateParam("/clear_buffer_timeout", expected_value);
}

TEST_F(AdiImuParams,validateImuProdId)
{
  int expected_value;
  getExpectedParamVal("/imu_prod_id", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_prod_id", expected_value);
}

TEST_F(AdiImuParams,validateGravity)
{
  double expected_value;
  getExpectedParamVal("/gravity", expected_value);
  if(!HasFatalFailure)
    validateParam("/gravity", expected_value);
}

TEST_F(AdiImuParams,validateImuDataFormat)
{
  int expected_value;
  getExpectedParamVal("/imu_data_format", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_data_format", expected_value);
}

TEST_F(AdiImuParams,validateImuDataRate)
{
  int expected_value;
  getExpectedParamVal("/imu_data_rate", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_data_rate", expected_value);
}

TEST_F(AdiImuParams,validateImuDataRdyLine)
{
  int expected_value;

  if(prod_under_test_ == 16470 || prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16470");
    GTEST_SKIP();
  }

  getExpectedParamVal("/imu_data_rdy_line", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_data_rdy_line", expected_value);
}

TEST_F(AdiImuParams,validateImuDataRdyPol)
{
  int expected_value;
  getExpectedParamVal("/imu_data_rdy_pol", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_data_rdy_pol", expected_value);
}

TEST_F(AdiImuParams,validateEnableImuSyncClk)
{
  bool expected_value;
  getExpectedParamVal("/enable_imu_sync_clk", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_imu_sync_clk", expected_value);
}

TEST_F(AdiImuParams,validateImuSyncClkMode)
{
  int expected_value;
  getExpectedParamVal("/imu_sync_clk_mode", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_sync_clk_mode", expected_value);
}

TEST_F(AdiImuParams,validateImuSyncClkLine)
{
  int expected_value;

  if(prod_under_test_ == 16470 || prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16470");
    GTEST_SKIP();
  }

  getExpectedParamVal("/imu_sync_clk_line", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_sync_clk_line", expected_value);
}

TEST_F(AdiImuParams,validateImuSyncClkPol)
{
  int expected_value;
  getExpectedParamVal("/imu_sync_clk_pol", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_sync_clk_pol", expected_value);
}

TEST_F(AdiImuParams,validateEnableImuLinGComp)
{
  bool expected_value;
  getExpectedParamVal("/enable_imu_lin_g_comp", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_imu_lin_g_comp", expected_value);
}

TEST_F(AdiImuParams,validateEnableImuPpAlign)
{
  bool expected_value;
  getExpectedParamVal("/enable_imu_pp_align", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_imu_pp_align", expected_value);
}

TEST_F(AdiImuParams,validateUpdateImuBiasCorr)
{
  bool expected_value;

  if(prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/update_imu_bias_corr", expected_value);
  if(!HasFatalFailure)
    validateParam("/update_imu_bias_corr", expected_value);
}

TEST_F(AdiImuParams,validateImuTimeBaseControl)
{
  int expected_value;

  if(prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/imu_time_base_control", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_time_base_control", expected_value);
}

TEST_F(AdiImuParams,validateEnableImuAcclZBiasNull)
{
  int expected_value;

  if(prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/enable_imu_accl_z_bias_null", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_imu_accl_z_bias_null", expected_value);
}

TEST_F(AdiImuParams,validateEnableImuAcclYBiasNull)
{
  int expected_value;

  if(prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/enable_imu_accl_y_bias_null", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_imu_accl_y_bias_null", expected_value);
}

TEST_F(AdiImuParams,validateEnableImuAcclXBiasNull)
{
  int expected_value;

  if(prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/enable_imu_accl_x_bias_null", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_imu_accl_x_bias_null", expected_value);
}

TEST_F(AdiImuParams,validateEnableImuGyroZBiasNull)
{
  int expected_value;

  if(prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/enable_imu_gyro_z_bias_null", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_imu_gyro_z_bias_null", expected_value);
}

TEST_F(AdiImuParams,validateEnableImuGyroYBiasNull)
{
  int expected_value;

  if(prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/enable_imu_gyro_y_bias_null", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_imu_gyro_y_bias_null", expected_value);
}

TEST_F(AdiImuParams,validateEnableImuGyroXBiasNull)
{
  int expected_value;

  if(prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/enable_imu_gyro_x_bias_null", expected_value);
  if(!HasFatalFailure)
    validateParam("/enable_imu_gyro_x_bias_null", expected_value);
}

TEST_F(AdiImuParams,validateUpdateImuAcclBias)
{
  bool expected_value;
  getExpectedParamVal("/update_imu_accl_bias", expected_value);
  if(!HasFatalFailure)
    validateParam("/update_imu_accl_bias", expected_value);
}

TEST_F(AdiImuParams,validateImuAcclBias)
{
  std::vector<int> expected_value;
  getExpectedParamVal("/imu_accl_bias", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_accl_bias", expected_value);
}

TEST_F(AdiImuParams,validateUpdateImuGyroBias)
{
  bool expected_value;
  getExpectedParamVal("/update_imu_gyro_bias", expected_value);
  if(!HasFatalFailure)
    validateParam("/update_imu_gyro_bias", expected_value);
}

TEST_F(AdiImuParams,validateImuGyroBias)
{
  std::vector<int> expected_value;
  getExpectedParamVal("/imu_gyro_bias", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_gyro_bias", expected_value);
}

TEST_F(AdiImuParams,validateUpdateImuAcclScale)
{
  bool expected_value;

  if(prod_under_test_ == 16470 || prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16470 / ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/update_imu_accl_scale", expected_value);
  if(!HasFatalFailure)
    validateParam("/update_imu_accl_scale", expected_value);
}

TEST_F(AdiImuParams,validateImuAcclScale)
{
  std::vector<int> expected_value;

  if(prod_under_test_ == 16470 || prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16470 / ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/imu_accl_scale", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_accl_scale", expected_value);
}

TEST_F(AdiImuParams,validateUpdateImuGyroScale)
{
  bool expected_value;

  if(prod_under_test_ == 16470 || prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16470 / ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/update_imu_gyro_scale", expected_value);
  if(!HasFatalFailure)
    validateParam("/update_imu_gyro_scale", expected_value);
}

TEST_F(AdiImuParams,validateImuGyroScale)
{
  std::vector<int> expected_value;

  if(prod_under_test_ == 16470 || prod_under_test_ == 16500)
  {
    ROS_WARN_STREAM("Unused in ADIS16470 / ADIS16500");
    GTEST_SKIP();
  }

  getExpectedParamVal("/imu_gyro_scale", expected_value);
  if(!HasFatalFailure)
    validateParam("/imu_gyro_scale", expected_value);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "test_param");
  ros::NodeHandle nh;

  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  ROS_INFO_STREAM("Wait 10 seconds...");
  ros::Duration(10).sleep();

  ros::Rate loopRate(50);
  for(size_t i = 0; i < 200; ++i)
  {
    ros::spinOnce();
    loopRate.sleep();
  }

  ROS_DEBUG_STREAM("[TEST] Execute tests.");

  testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();

  return res;
}
