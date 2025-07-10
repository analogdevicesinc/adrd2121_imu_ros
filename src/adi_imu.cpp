/*
 * Copyright (c) 2022, 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 */

#include "adi_imu.h"

AdiImu::AdiImu(adi_imu_Device_t* p_device):
  p_device_(NULL),
  imu_accl_bias_(IMU_ACCL_BIAS_SIZE,0),
  imu_gyro_bias_(IMU_ACCL_BIAS_SIZE,0),
  imu_accl_scale_(IMU_ACCL_SCALE_SIZE,0),
  imu_gyro_scale_(IMU_ACCL_SCALE_SIZE,0)
{
    // Initialize object
  p_device_=p_device;
}

AdiImu::~AdiImu()
{
  // Destroy object
}

void AdiImu::loadParams(ros::NodeHandle* p_nh_local)
{
  bool b_default=false;

  ROS_INFO("Loading [ADI IMU] Parameters...");

  b_default = p_nh_local->param("imu_prod_id", imu_prod_id_, 16470);
  ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Product ID: " << imu_prod_id_);
  ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Product ID: " << imu_prod_id_);

  b_default = p_nh_local->param("gravity", gravity_, 1.0);
  ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting Gravity: " << gravity_);
  ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default Gravity: " << gravity_);

  b_default = p_nh_local->param("imu_data_format", imu_data_format_, static_cast<int>(IMU_DATA_32BIT));
  ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Data Format: " << imu_data_format_);
  ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Data Format: " << imu_data_format_);

  b_default = p_nh_local->param("imu_data_rate", imu_data_rate_, 100);
  ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Data Rate (Hz): " << imu_data_rate_);
  ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Data Rate (Hz): " << imu_data_rate_);

  b_default = p_nh_local->param("imu_data_rdy_line", imu_data_rdy_line_, static_cast<int>(IMU_DIO1));
  ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Data Ready Line: DIO" << (imu_data_rdy_line_+1));
  ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Data Ready Line: DIO" << (imu_data_rdy_line_+1));

  b_default = p_nh_local->param("imu_data_rdy_pol", imu_data_rdy_pol_, 1);
  ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Data Ready Polarity: " << imu_data_rdy_pol_);
  ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Data Ready Polarity: "<< imu_data_rdy_pol_);

  b_default = p_nh_local->param("enable_imu_sync_clk", b_imu_sync_clk_, false);
  ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Sync Clock: " << BoolToString(b_imu_sync_clk_));
  ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Sync Clock: " << BoolToString(b_imu_sync_clk_));

  if(b_imu_sync_clk_)
  {
    b_default = p_nh_local->param("imu_sync_clk_mode", imu_sync_clk_mode_, 0);
    ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Sync Clock Mode: " << imu_sync_clk_mode_);
    ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Sync Clock Mode: " << imu_sync_clk_mode_);

    b_default = p_nh_local->param("imu_sync_clk_line", imu_sync_clk_line_, static_cast<int>(IMU_DIO1));
    ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Sync Clock Line: DIO" << (imu_data_rdy_line_+1));
    ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Sync Clock Line: DIO" << (imu_data_rdy_line_+1));

    b_default = p_nh_local->param("imu_sync_clk_pol", imu_sync_clk_pol_, 1);
    ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Sync Clock Polarity: " << imu_sync_clk_pol_);
    ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Sync Clock Polarity: " << imu_sync_clk_pol_);
  }

  b_default = p_nh_local->param("enable_imu_lin_g_comp", b_imu_linear_g_comp_, false);
  ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Linear g compensation: " << \
    BoolToString(b_imu_linear_g_comp_));
  ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Linear g compensation: " << \
    BoolToString(b_imu_linear_g_comp_));

  b_default = p_nh_local->param("enable_imu_pp_align", b_imu_pp_align_, false);
  ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Point of Percussion Alignment: " << \
    BoolToString(b_imu_pp_align_));
  ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Point of Percussion Alignment: " << \
    BoolToString(b_imu_pp_align_));

  if(imu_prod_id_!=16500) // No NULL_CNFG/CBE in ADIS16500
  {
    b_default = p_nh_local->param("update_imu_bias_corr", b_imu_update_bias_corr_, false);
    ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Bias Correction Update: " << \
      BoolToString(b_imu_update_bias_corr_));
    ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Bias Correction Update:" << \
      BoolToString(b_imu_update_bias_corr_));

    if(b_imu_update_bias_corr_)
    {
      b_default = p_nh_local->param("imu_time_base_control", imu_time_base_control_, 10);
      ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Time Base Control: " << imu_time_base_control_);
      ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Time Base Control: " << imu_time_base_control_);

      b_default = p_nh_local->param("enable_imu_accl_z_bias_null", imu_accl_z_bias_null_, 0);
      ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Z-axis Accel bias null cmd: " << ((imu_accl_z_bias_null_) ?\
        "enabled" : "disabled"));
      ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Z-axis Accel bias null cmd: " << \
        ((imu_accl_z_bias_null_) ? "enabled" : "disabled"));

      b_default = p_nh_local->param("enable_imu_accl_y_bias_null", imu_accl_y_bias_null_, 0);
      ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Y-axis Accel bias null cmd: " << ((imu_accl_y_bias_null_) ?\
        "enabled" : "disabled"));
      ROS_WARN_STREAM_COND(!b_default, "Using default IMU Y-axis Accel bias null cmd: " << ((imu_accl_y_bias_null_) ? \
                                        "enabled" : "disabled"));
      b_default = p_nh_local->param("enable_imu_accl_x_bias_null", imu_accl_x_bias_null_, 0);
      ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU X-axis Accel bias null cmd: " << ((imu_accl_x_bias_null_) ?\
        "enabled" : "disabled"));
      ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU X-axis Accel bias null cmd: " << \
        ((imu_accl_x_bias_null_) ? "enabled" : "disabled"));

      b_default = p_nh_local->param("enable_imu_gyro_z_bias_null", imu_gyro_z_bias_null_, 1);
      ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Z-axis Gyro bias null cmd: " << ((imu_gyro_z_bias_null_) ? \
        "enabled" : "disabled"));
      ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Z-axis Gyro bias null cmd: " << \
        ((imu_gyro_z_bias_null_) ? "enabled" : "disabled"));

      b_default = p_nh_local->param("enable_imu_gyro_y_bias_null", imu_gyro_y_bias_null_, 1);
      ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Y-axis Gyro bias null cmd: " << ((imu_gyro_y_bias_null_) ? \
        "enabled" : "disabled"));
      ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Y-axis Gyro bias null cmd: " << \
        ((imu_gyro_y_bias_null_) ? "enabled" : "disabled"));

      b_default = p_nh_local->param("enable_imu_gyro_x_bias_null", imu_gyro_x_bias_null_, 1);
      ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU X-axis Gyro bias null cmd: " << ((imu_gyro_x_bias_null_) ? \
        "enabled" : "disabled"));
      ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU X-axis Gyro bias null cmd: " << \
        ((imu_gyro_x_bias_null_) ? "enabled" : "disabled"));
    }
  }

  b_default = p_nh_local->param("update_imu_accl_bias", b_update_imu_accl_bias_, false);
  ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Accl Bias Update: " << BoolToString(b_update_imu_accl_bias_));
  ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Accl Bias Update:" <<\
    BoolToString(b_update_imu_accl_bias_));

  if(b_update_imu_accl_bias_)
  {
    b_default=p_nh_local->getParam("imu_accl_bias",imu_accl_bias_);
    ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Accl Bias: [" << imu_accl_bias_[0]<<","\
      <<imu_accl_bias_[1]<<","<<imu_accl_bias_[2]<<"]");
    ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default Accl IMU Bias: [0,0,0]");
  }

  b_default = p_nh_local->param("update_imu_gyro_bias", b_update_imu_gyro_bias_, false);
  ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Gyro Bias Update: " << BoolToString(b_update_imu_gyro_bias_));
  ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Gyro Bias Update:" << \
    BoolToString(b_update_imu_gyro_bias_));

  if(b_update_imu_gyro_bias_)
  {
    b_default=p_nh_local->getParam("imu_gyro_bias",imu_gyro_bias_);
    ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Gyro Bias: [" << imu_gyro_bias_[0]<<","\
      <<imu_gyro_bias_[1]<<","<<imu_gyro_bias_[2]<<"]");
    ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Gyro Bias: [0,0,0]");
  }

  if(imu_prod_id_==16495) // Only supported in 16495
  {
    b_default = p_nh_local->param("update_imu_accl_scale", b_update_imu_accl_scale_, false);
    ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Accl Scale Update: " << \
      BoolToString(b_update_imu_accl_scale_));
    ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Accl Scale Update:" <<\
      BoolToString(b_update_imu_accl_scale_));

    if(b_update_imu_accl_scale_)
    {
      b_default=p_nh_local->getParam("imu_accl_scale",imu_accl_scale_);
      ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Accl Scale: [" << imu_accl_scale_[0]<<","\
        <<imu_accl_scale_[1]<<","<<imu_accl_scale_[2]<<"]");
      ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default Accl IMU Scale: [0,0,0]");
    }

    b_default = p_nh_local->param("update_imu_gyro_scale", b_update_imu_gyro_scale_, false);
    ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Gyro Scale Update: " << \
      BoolToString(b_update_imu_gyro_scale_));
    ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Gyro Scale Update:" << \
      BoolToString(b_update_imu_gyro_scale_));

    if(b_update_imu_gyro_scale_)
    {
      b_default=p_nh_local->getParam("imu_gyro_scale",imu_gyro_scale_);
      ROS_DEBUG_STREAM_COND(b_default, "[ADI IMU] Setting IMU Gyro Scale: [" << imu_gyro_scale_[0]<<","\
        <<imu_gyro_scale_[1]<<","<<imu_gyro_scale_[2]<<"]");
      ROS_WARN_STREAM_COND(!b_default, "[ADI IMU] Using default IMU Gyro Scale: [0,0,0]");
    }
  }

  // Update the adi_imu_Device_t
  p_device_->prodId=imu_prod_id_;
  p_device_->g=gravity_;
  p_device_->dataFormat=static_cast<adi_imu_DataFormat_e>(imu_data_format_);

  ROS_INFO("Loaded [ADI IMU]  Parameters.");
}

bool AdiImu::init(void)
{
  bool b_success=false;
  int retry_id_check=0;
  int ret = -1;

  //Initialize IMU
#if defined(ADRD2121_IMU_WITH_HW)
  while((3>retry_id_check))
  {
    ret=adi_imu_Init(p_device_);
    if(Err_imu_ProdIdVerifyFailed_e==ret)
    {
      ROS_WARN_STREAM("[ADI IMU] Failed to verify IMU Product ID. Will try again...");
      retry_id_check++;
    }
    else
    {
      break;// Continue if successful or if there are other failures apart ProdIdVerifyFailed
    }
  }
#else
  int ret=0;
#endif
  ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] initialization unsuccessful. ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
  ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] initialization successful");
  b_success=(ret>=0)? true : false;

  return b_success;
}

bool AdiImu::config(void)
{
  bool b_success=false;
  int ret=-1;
#if defined(ADRD2121_IMU_WITH_HW)
  // Set Output Data Rate
  ret=adi_imu_SetOutputDataRate(p_device_,imu_data_rate_);
#else
  ret=0;
#endif
  ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Set IMU Data Rate unsuccessful. ERROR CODE: "<< ret \
    <<" Check adi_imu_Error_e.");
  ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Set IMU Data Rate successful");
  b_success=(ret>=0)? true : false;

  // Configure and Enable Data Ready
  if(b_success)
  {
    if(p_device_->prodId==16495)
    {
#if defined(ADRD2121_IMU_WITH_HW)
      ret=adi_imu_ConfigDataReady(p_device_,static_cast<adi_imu_GPIO_e>(imu_data_rdy_line_),\
        static_cast<adi_imu_Polarity_e>(imu_data_rdy_pol_));
#else
      ret=0;
#endif
      ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Data Ready Config unsuccessful. ERROR CODE: "<< ret \
        <<" Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Data Ready Config successful");
      b_success=(ret>=0)? true : false;

#if defined(ADRD2121_IMU_WITH_HW)
      ret=adi_imu_SetDataReady(p_device_,IMU_ENABLE);
#else
      ret=0;
#endif
      ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Enable IMU Data Ready unsuccessful. ERROR CODE: "<< ret <<" \
        Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Enable IMU Data Ready successful");
      b_success=(ret>=0)? true : false;
    }
    else if(p_device_->prodId==16470 || p_device_->prodId==16500)
    {
      ret=adi_imu_ConfigDataReady(p_device_,static_cast<adi_imu_GPIO_e>(IMU_NULL),\
        static_cast<adi_imu_Polarity_e>(imu_data_rdy_pol_));
      ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Data Ready Config unsuccessful. ERROR CODE: "<< ret \
        <<" Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Data Ready Config successful");
      b_success=(ret>=0)? true : false;

      ROS_INFO_STREAM("[ADI IMU] IMU Data Ready always enabled in " << p_device_->prodId);
    }
  }

  // Configure and Enable Sync Clock
  if(b_success && b_imu_sync_clk_)
  {
    if(p_device_->prodId==16495)
    {
#if defined(ADRD2121_IMU_WITH_HW)
      ret=adi_imu_ConfigSyncClkMode(p_device_,static_cast<adi_imu_ClockMode_e>(imu_sync_clk_mode_),IMU_ENABLE,\
        static_cast<adi_imu_EdgeType_e>(imu_sync_clk_pol_),static_cast<adi_imu_GPIO_e>(imu_sync_clk_line_));
#else
      ret=0;
#endif
    }
    else if(p_device_->prodId==16470 || p_device_->prodId==16500)
    {
      ret=adi_imu_ConfigSyncClkMode(p_device_,static_cast<adi_imu_ClockMode_e>(imu_sync_clk_mode_),\
        static_cast<adi_imu_EnDis_e>(IMU_NULL), static_cast<adi_imu_EdgeType_e>(imu_sync_clk_pol_),\
        static_cast<adi_imu_GPIO_e>(IMU_NULL));
    }
    ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Sync Clk Config unsuccessful. ERROR CODE: "<< ret \
      <<" Check adi_imu_Error_e.");
    ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Sync Clk Config successful");
    b_success=(ret>=0)? true : false;
  }

  // Set Linear G compensation
  if(b_success)
  {
    adi_imu_EnDis_e val = (b_imu_linear_g_comp_)? IMU_ENABLE : IMU_DISABLE;
#if defined(ADRD2121_IMU_WITH_HW)
    ret=adi_imu_SetLineargComp(p_device_,val);
#else
    ret=0;
#endif
    ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Set IMU Linear g Compensation unsuccessful.\
      ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
    ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Set IMU Linear g Compensation successful");
    b_success=(ret>=0)? true : false;
  }

  // Set Point of Percussion Alignment
  if(b_success)
  {
    adi_imu_EnDis_e val = (b_imu_pp_align_)? IMU_ENABLE : IMU_DISABLE;
#if defined(ADRD2121_IMU_WITH_HW)
    ret=adi_imu_SetPPercAlignment(p_device_,val);
#else
    ret=0;
#endif
    ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Set IMU Point of Percussion Alignment unsuccessful.\
      ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
    ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Set IMU Point of Percussion Alignment successful");
    b_success=(ret>=0)? true : false;
  }

  if(p_device_->prodId==16495 || p_device_->prodId==16470)
  {
    // Configure CBE
    if(b_success && b_imu_update_bias_corr_)
    {
      ret=adi_imu_ConfigBiasCorrectionTime(p_device_,imu_time_base_control_); // Not declared in header
      ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Configure IMU Bias Correction Accumulation Time unsuccessful.\
        ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Configure IMU Bias Correction Accumulation Time successful");
      if(ret>=0)
        b_success=true;

      ret=adi_imu_SelectBiasConfigAxes(p_device_,static_cast<adi_imu_EnDis_e>(imu_gyro_x_bias_null_), \
          static_cast<adi_imu_EnDis_e>(imu_gyro_y_bias_null_),static_cast<adi_imu_EnDis_e>(imu_gyro_z_bias_null_), \
          static_cast<adi_imu_EnDis_e>(imu_accl_x_bias_null_),static_cast<adi_imu_EnDis_e>(imu_accl_y_bias_null_), \
          static_cast<adi_imu_EnDis_e>(imu_accl_z_bias_null_));
      ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Configure IMU Sensors to be Nulled unsuccessful.\
       ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Configure IMU Sensors to be Nulled successful");
      if(ret>=0)
        b_success=true;

      // Trigger Bias Correction Update
  #if defined(ADRD2121_IMU_WITH_HW)
      ret=adi_imu_UpdateBiasCorrection(p_device_);
  #else
      ret=0;
  #endif
      ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Trigger IMU Bias Correction unsuccessful.\
        ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Trigger IMU Bias Correction successful");
        b_success=(ret>=0)? true : false;
    }
  }

  // Setting Accelerometer Bias (*_ACCL_BIAS Register)
  if(b_success && b_update_imu_accl_bias_)
  {
    adi_imu_AcclBiasRaw32_t accl_bias;
    accl_bias.x=static_cast<int32_t>(imu_accl_bias_[0]);
    accl_bias.y=static_cast<int32_t>(imu_accl_bias_[1]);
    accl_bias.z=static_cast<int32_t>(imu_accl_bias_[2]);
    ret=adi_imu_SetAcclBias(p_device_, accl_bias);
    ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Configure IMU Accl Bias unsuccessful.\
      ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
    ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Configure IMU Accl Bias successful");
    if(ret>=0)
      b_success=true;

    if((ret=adi_imu_GetAcclBias(p_device_, &accl_bias)) < 0) return ret;
    ROS_INFO_STREAM("[ADI IMU] Read Accl Bias. Result: x:"<<accl_bias.x<<", y:"<<accl_bias.y<<", z:"<<accl_bias.z);
  }

  // Setting Gyroscope Bias (*_GYRO_BIAS Register)
  if(b_success && b_update_imu_gyro_bias_)
  {
    adi_imu_GyroBiasRaw32_t gyro_bias;
    gyro_bias.x=static_cast<int32_t>(imu_gyro_bias_[0]);
    gyro_bias.y=static_cast<int32_t>(imu_gyro_bias_[1]);
    gyro_bias.z=static_cast<int32_t>(imu_gyro_bias_[2]);
    ret=adi_imu_SetGyroBias(p_device_, gyro_bias);
    ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Configure IMU Gyro Bias unsuccessful.\
      ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
    ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Configure IMU Gyro Bias successful");
    if(ret>=0)
      b_success=true;

    if((ret=adi_imu_GetGyroBias(p_device_, &gyro_bias)) < 0) return ret;
    ROS_INFO_STREAM("[ADI IMU] Read Gyro Bias. Result: x:"<<gyro_bias.x<<", y:"<<gyro_bias.y<<", z:"<<gyro_bias.z);
  }

  if(p_device_->prodId==16495)
  {
    // Setting Accelerometer Scale (*_ACCL_SCALE Register)
    if(b_success && b_update_imu_accl_scale_)
    {
      adi_imu_AcclScale_t accl_scale;
      accl_scale.x=static_cast<int16_t>(imu_accl_scale_[0]);
      accl_scale.y=static_cast<int16_t>(imu_accl_scale_[1]);
      accl_scale.z=static_cast<int16_t>(imu_accl_scale_[2]);
      ret=adi_imu_SetAcclScale(p_device_, accl_scale);
      ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Configure IMU Accl Scale unsuccessful.\
        ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Configure IMU Accl Scale successful");
      if(ret>=0)
        b_success=true;

      if((ret=adi_imu_GetAcclScale(p_device_, &accl_scale)) < 0) return ret;
      ROS_INFO_STREAM("[ADI IMU] Read Accl Scale. Result: x:"<<accl_scale.x<<", y:"<<accl_scale.y<<", z:"<<accl_scale.z);
    }

    // Setting Gyroscope Scale (*_GYRO_SCALE Register)
    if(b_success && b_update_imu_gyro_scale_)
    {
      adi_imu_GyroScale_t gyro_scale;
      gyro_scale.x=static_cast<int16_t>(imu_gyro_scale_[0]);
      gyro_scale.y=static_cast<int16_t>(imu_gyro_scale_[1]);
      gyro_scale.z=static_cast<int16_t>(imu_gyro_scale_[2]);
      ret=adi_imu_SetGyroScale(p_device_, gyro_scale);
      ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Configure IMU Gyro Scale unsuccessful.\
        ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Configure IMU Gyro Scale successful");
      if(ret>=0)
        b_success=true;

      if((ret=adi_imu_GetGyroScale(p_device_, &gyro_scale)) < 0) return ret;
      ROS_INFO_STREAM("[ADI IMU] Read Gyro Scale. Result: x:"<<gyro_scale.x<<", y:"<<gyro_scale.y<<", z:"<<gyro_scale.z);
    }
  }

  ROS_INFO_STREAM_COND(b_success,"[ADI IMU] Configuration successful.");

  if(p_device_->prodId==16500)
  {
    if(b_success)
    {
      /* Set IMU Burst Data Format */
      ret = adi_imu_ConfigBurstDataFormat(p_device_, p_device_->dataFormat);
      ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Configure Burst Data Format unsuccessful. ERROR CODE: "<< ret <<" \
        Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Configure Burst Data Format successful");
      b_success=(ret>=0)? true : false;
    }
  }

#if defined(ADRD2121_IMU_WITH_HW)
  /* Read and print IMU device info and config */
  adi_imu_DevInfo_t imuInfo;
  if ((ret = adi_imu_GetDevInfo(p_device_, &imuInfo)) < 0) return ret;
  if ((ret = adi_imu_PrintDevInfo(p_device_, &imuInfo)) < 0) return ret;
  if(imu_prod_id_ == 16470 || imu_prod_id_ == 16500)
  {
    actual_imu_data_rate_ = 2000 / (imuInfo.decimationRate + 1);
  }
  else if (imu_prod_id_ == 16495)
  {
    actual_imu_data_rate_ = 4250 / (imuInfo.decimationRate + 1);
  }

  if(actual_imu_data_rate_ != imu_data_rate_)
  {
    ROS_WARN_STREAM("[ADI IMU] Actual IMU Rate set: ~"<< actual_imu_data_rate_ <<" HZ");
  }
  else
  {
    ROS_INFO_STREAM("[ADI IMU] Actual IMU Rate set: ~"<< actual_imu_data_rate_ <<" HZ");
  }
#else
      ret=0;
#endif

  return b_success;
}

int AdiImu::getImuDataRateHz(void)
{
  return actual_imu_data_rate_;
}

bool AdiImu::triggerBiasCorrectionUpdate(void)
{
  bool b_success=false;

  // Trigger Bias Correction Update
  ROS_INFO_STREAM("[ADI IMU] Trigger IMU Bias Correction Update.");
#if ADRD2121_IMU_WITH_HW
  int ret=adi_imu_UpdateBiasCorrection(p_device_);
#else
  ret=0;
#endif
  ROS_ERROR_STREAM_COND(ret<0,"[ADI IMU] Trigger IMU Bias Correction unsuccessful.\
    ERROR CODE: "<< ret <<" Check adi_imu_Error_e.");
  ROS_INFO_STREAM_COND(ret>=0,"[ADI IMU] Trigger IMU Bias Correction successful");
  b_success=(ret>=0)? true : false;

  return b_success;
}
