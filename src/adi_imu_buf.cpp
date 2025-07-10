/*
 * Copyright (c) 2022, 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 */

#include "adi_imu_buf.h"

AdiImuBuf::AdiImuBuf(ros::NodeHandle* nodehandle):
  nh_local_("~"), nh_(*nodehandle), p_imu_buf_(NULL), p_adi_imu_(NULL)
{
  // Initialize object
  ROS_INFO("Class constructor for ADI IMU + IMU Buffer");
  mode_of_operation_ = STREAMING;
  b_hw_initialized_=false;
  b_read_pub_data_success_=true;
  p_imu_buf_= new ImuBuf(&imu_dev_);
  p_adi_imu_ = new AdiImu(&imu_dev_);

}

AdiImuBuf::~AdiImuBuf()
{
  // Destroy object
  delete p_imu_buf_;
  delete p_adi_imu_;

  // Manually close serial port connection
  close(imu_dev_.uartDev.fd);
}

void AdiImuBuf::loadParams(void)
{
  bool b_default = false;

  p_imu_buf_->loadParams(&nh_local_);

  p_adi_imu_->loadParams(&nh_local_);

  ROS_INFO("Loading ROS Parameters");

  b_default = nh_local_.param<std::string>("topic_name", topic_name_, "imu/data_raw");
  ROS_DEBUG_STREAM_COND(b_default, "Topic name: " << topic_name_);
  ROS_WARN_STREAM_COND(!b_default, "Using default topic name: " << topic_name_);

  b_default = nh_local_.param<std::string>("frame_name", frame_name_, "imu");
  ROS_DEBUG_STREAM_COND(b_default, "Frame name: " << frame_name_);
  ROS_WARN_STREAM_COND(!b_default, "Using default frame name: " << frame_name_);

  b_default = nh_local_.param("msg_type", msg_type_, (int)SENSOR_MSGS_IMU);
  ROS_DEBUG_STREAM_COND(b_default, "MSG Type: " << msg_type_);
  ROS_WARN_STREAM_COND(!b_default, "Using default MSG type: " << msg_type_);

  int mode_of_operation_int=0;
  b_default = nh_local_.param("mode_of_operation", mode_of_operation_int, (int)STREAMING);
  mode_of_operation_=static_cast<mode_of_operation_e>(mode_of_operation_int);
  ROS_DEBUG_STREAM_COND(b_default, "Mode of operation: " << mode_of_operation_);
  ROS_WARN_STREAM_COND(!b_default, "Using default Mode of operation: " << mode_of_operation_);

  ROS_DEBUG_STREAM("imu.prodId = " << imu_dev_.prodId);
  ROS_DEBUG_STREAM("imu.g = " << imu_dev_.g);
  ROS_DEBUG_STREAM("imu.enable_buffer = " << imu_dev_.enable_buffer);
  ROS_DEBUG_STREAM("imu.devtype = " << imu_dev_.devType);
  ROS_DEBUG_STREAM("imu.uartDev.dev = " << imu_dev_.uartDev.dev);
  ROS_DEBUG_STREAM("imu.uartDev.baud = " << imu_dev_.uartDev.baud);
}

bool AdiImuBuf::init(void)
{
  bool b_success = false;

  // Check Lib Build Info
  adi_imu_BuildInfo_t binfo = adi_imu_GetBuildInfo(&imu_dev_);
  ROS_INFO_STREAM("IMU_LIB_VERSION= " << binfo.version_full);
  ROS_INFO_STREAM("IMU_LIB_BUILD_TIME= " << binfo.build_time);
  ROS_INFO_STREAM("IMU_LIB_BUILD_TYPE= " << binfo.build_type);

  // Initialize device connected to Host

  ROS_INFO_STREAM("Initialize device connected to Host...");
#if defined(ADRD2121_IMU_WITH_HW)
  int ret=hw_Init(&imu_dev_);
#else
  int ret=0;
#endif
  ROS_ERROR_STREAM_COND(ret<0,"Device initialization unsuccessful. ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
  ROS_INFO_STREAM_COND(ret>=0,"Initialized device connected to Host successfully");

  if(ret>=0)
  {
    b_success=true;
    b_hw_initialized_=true;
  }
  else
  {
    b_success=false;
    b_hw_initialized_=false;
  }

  if(b_success)
  {

    //Check if Buffer board is detected (Will also initialize imubuf uart)
    // Ignore any failures since it can be fixed during STREAMING/RECOVERY
    p_imu_buf_->detect();
    // Print out initial status
    uint16_t status;
    std::string status_description="";
    p_imu_buf_->getStatus(&status, &status_description);

    ROS_INFO_STREAM_COND(mode_of_operation_==STREAMING,"Initializing as STREAMING mode...");
    ROS_INFO_STREAM_COND(mode_of_operation_==RECOVERY,"Initializing as RECOVERY mode...");
    switch(mode_of_operation_)
    {
      case(STREAMING):
        //Initialize ADRD2121
        b_success=p_imu_buf_->init();

        //Initialize Imu
        if(b_success)
        {
          b_success=p_adi_imu_->init();
        }

        // If initialization failed, go to RECOVERY mode
        if(!b_success)
        {
          mode_of_operation_=RECOVERY;
        }
        // Do not break, need to continue to initServiceServers
      case(RECOVERY):
        p_imu_buf_->initServiceServers(&nh_local_,mode_of_operation_);
        break;
      default:
        // Undefined mode of operation
        // Do nothing
        break;
    }
    p_imu_buf_->setModeOfOperation(mode_of_operation_);
  }
  else
  {
    ROS_ERROR_STREAM("Hardware is not initialized.");
  }

  return b_success;
}

bool AdiImuBuf::config(void)
{
  bool b_success=false;
  double imu_data_rate_hz=0;
  int buf_burst_count=1; //default value

  nh_local_.getParam("buf_burst_count", buf_burst_count);

  ROS_INFO_STREAM("Configure ADI IMU...");
  b_success=p_adi_imu_->config();

  if(b_success)
  {
    ROS_INFO_STREAM("Configure ADRD2121...");
    b_success=p_imu_buf_->config();
  }

  if(b_success)
  {
    // Advertise IMU topic
    if(SENSOR_MSGS_IMU==msg_type_)
    {
      imu_pub_=nh_.advertise<sensor_msgs::Imu>(topic_name_.c_str(), 1000);
    }
    else if(ADI_IMU_MSG==msg_type_)
    {
      imu_pub_=nh_.advertise<adrd2121_imu::AdiImu>(topic_name_.c_str(), 1000);
    }
    else
    {
      b_success=false;
    }

    if(b_success)
    {
      // Make sure the ROS loop rate is greater than (IMU data rate/Burst Count)
      imu_data_rate_hz = p_adi_imu_->getImuDataRateHz();
      ros_loop_rate_hz_ = 2*(imu_data_rate_hz/buf_burst_count);
      ROS_INFO_STREAM("ROS Loop Rate set to "<< ros_loop_rate_hz_<<"hz");

      ros::Rate period(ros_loop_rate_hz_);

      imu_pub_timer_callback_=nh_.createTimer(ros::Duration(period), &AdiImuBuf::readPubData, this);
    }

    if(imu_dev_.prodId == 16470 || imu_dev_.prodId == 16495)
    {
      // Advertise service
      imu_glob_cmd_service_ = nh_.advertiseService("trigger_imu_glob_cmd", &AdiImuBuf::triggerImuGlobCmd, this);
    }
    else if(imu_dev_.prodId == 16500)
    {
      ROS_WARN_STREAM("trigger_imu_glob_cmd is not supported in ADIS16500");
    }
  }


  return b_success;
}

void AdiImuBuf::readPubData(const ros::TimerEvent& event)
{
  b_read_pub_data_success_ = p_imu_buf_->readPubData(&imu_pub_,frame_name_,(msg_type_e)msg_type_);
  ROS_ERROR_STREAM_COND(!b_read_pub_data_success_,"Error data read");
}

bool AdiImuBuf::getHwInitialized(void)
{
  return b_hw_initialized_;
}

bool AdiImuBuf::triggerImuGlobCmd(adrd2121_imu::ImuGlobCmd::Request &req, \
  adrd2121_imu::ImuGlobCmd::Response &res)
{
  ROS_INFO_STREAM("[trigger_imu_glob_cmd service] Write to IMU GLOB_CMD Register...");
  bool b_success=false;
  bool b_valid_data=false;
  uint16_t data=1<<req.bit;

  // Check Validity of input
  // Currently only GLOB_CMD_BIAS_CORR_UPD is supported (Bit 0)
  uint16_t bitm_glob_cmd_bias_corr_upd = 0x0000;
  if(ADIS1647x == imu_dev_.imuProd)
    bitm_glob_cmd_bias_corr_upd = BITM_GLOB_CMD_BIAS_CORR_UPD_47x;
  else if(ADIS1649x == imu_dev_.imuProd)
    bitm_glob_cmd_bias_corr_upd = BITM_GLOB_CMD_BIAS_CORR_UPD_49x;
  if(data & bitm_glob_cmd_bias_corr_upd)
  {
    b_valid_data=true;
  }
  else
  {
    res.success=false;
    res.message="Triggering bit not supported";
    ROS_WARN_STREAM("[trigger_imu_glob_cmd service] Failed. Triggering bit not supported");
  }

  if(b_valid_data)
  {
    // Stop data capture first
    ROS_INFO_STREAM("Stop ADRD2121 Data Capture. Will pause publishing IMU data.");
    b_success = p_imu_buf_->stopBufferRead();

    if(b_success)
    {
      if(data & bitm_glob_cmd_bias_corr_upd)
      {
        b_success=p_adi_imu_->triggerBiasCorrectionUpdate();
      }

      if(b_success)
      {
        res.success=true;
        res.message="Triggered IMU Bias Correction Update Successfully";
        ROS_INFO_STREAM("[trigger_imu_glob_cmd service] Success.");
      }
      else
      {
        res.success=false;
        res.message="Trigger IMU Bias Correction Update FAILED";
        ROS_WARN_STREAM("[trigger_imu_glob_cmd service] Failed.");
      }

      // Start data capture first
      ROS_INFO_STREAM("Start ADRD2121 Data Capture. Will resume publishing IMU data.");
      b_success = p_imu_buf_->startBufferRead();

      if(!b_success)
      {
        ROS_ERROR_STREAM("[trigger_imu_glob_cmd service] Failed to restart ADRD2121 Data Capture. No data\
          will be published. Need to re-launch node.");
      }

    }
    else
    {
      res.success=false;
      res.message="Trigger IMU Bias Correction Update FAILED.";
      ROS_WARN_STREAM("[trigger_imu_glob_cmd service] Failed because IMU data capture was not stopped.");
    }
  }

  return true;
}

mode_of_operation_e AdiImuBuf::getModeOfOperation(void)
{
  return mode_of_operation_;
}

bool AdiImuBuf::getReadPubDataSuccess(void)
{
  return b_read_pub_data_success_;
}
