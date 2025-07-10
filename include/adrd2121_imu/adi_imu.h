/*
 * Copyright (c) 2022, 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 */

#ifndef ADI_IMU_H
#define ADI_IMU_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdint.h>
#include <ros/console.h>

// Include adi_imu_driver headers
#include "adi_imu_driver.h"

// Include adrd2121_imu headers
#include "adrd2121_imu_common.h"

//! \brief Constant for imu_accl_bias_ vector size
const int IMU_ACCL_BIAS_SIZE = 3;

//! \brief Constant for imu_gyro_bias_ vector size
const int IMU_GYRO_BIAS_SIZE = 3;

//! \brief Constant for imu_accl_scale_ vector size
const int IMU_ACCL_SCALE_SIZE = 3;

//! \brief Constant for imu_gyro_scale_ vector size
const int IMU_GYRO_SCALE_SIZE = 3;

//! \class AdiImu
//! \brief Class for ADI IMU
class AdiImu
{
private:

  //! \brief IMU Product ID (e.g. 16495, 16470, 16500)
  int imu_prod_id_;

  //! \brief Gravity Constant; Multiplier to accelerometer data
  double gravity_;

  //! \brief IMU Data Format (for Burst Read or non-Burst Read); 16 or 32 bit
  int imu_data_format_;

  //! \brief IMU Data Rate in Hz
  int imu_data_rate_;

  //! \brief Boolean for enabling IMU Data Ready
  bool b_imu_data_rdy_;

  //! \brief IMU Data ready line selection
  int imu_data_rdy_line_;

  //! \brief IMU Data ready polarity
  int imu_data_rdy_pol_;

  //! \brief Boolean for enabling configuration IMU Sync clock input
  bool b_imu_sync_clk_;

  //! \brief IMU Sync clock mode
  int imu_sync_clk_mode_;

  //! \brief IMU Sync clock input line selection
  int imu_sync_clk_line_;

  //! \brief IMU Sync clock input polarity
  int imu_sync_clk_pol_;

  //! \brief Boolean for enabling/disabling Linear g compensation for gyroscopes
  bool b_imu_linear_g_comp_;

  //! \brief Boolean for enabling/disabling Point of percussion alignment
  bool b_imu_pp_align_;

  //! \brief Booalean to Trigger Bias Correction Update in IMU GLOB_CMD register
  bool b_imu_update_bias_corr_;

  //! \brief Time Base Control (TBC)
  int imu_time_base_control_;

  //! \brief Enable/Disable Z-axis acceleration bias correction
  int imu_accl_z_bias_null_;

  //! \brief Enable/Disable Y-axis acceleration bias correction
  int imu_accl_y_bias_null_;

  //! \brief Enable/Disable X-axis acceleration bias correction
  int imu_accl_x_bias_null_;

  //! \brief Enable/Disable Z-axis gyroscope bias correction
  int imu_gyro_z_bias_null_;

  //! \brief Enable/Disable Y-axis gyroscope bias correction
  int imu_gyro_y_bias_null_;

  //! \brief  Enable/Disable X-axis gyroscope bias correction
  int imu_gyro_x_bias_null_;

  //! \brief Boolean to Set Accelerometer bias
  bool b_update_imu_accl_bias_;

  //! \brief Accelerometer bias values
  std::vector<int> imu_accl_bias_;

  //! \brief Boolean to Set Gyroscope bias
  bool b_update_imu_gyro_bias_;

  //! \brief Gyroscope bias values
  std::vector<int> imu_gyro_bias_;

  //! \brief Boolean to Set Accelerometer scale
  bool b_update_imu_accl_scale_;

  //! \brief Accelerometer scale values
  std::vector<int> imu_accl_scale_;

  //! \brief Boolean to Set Gyroscope scale
  bool b_update_imu_gyro_scale_;

  //! \brief Gyroscope scale values
  std::vector<int> imu_gyro_scale_;

  //! \brief Pointer to main device struct
  adi_imu_Device_t* p_device_;

  //! \brief IMU Data Rate in Hz
  double actual_imu_data_rate_;

public:
  //! \brief Constructor for AdiImu class
  //!
  //! \param[in] p_device   Pointer to adi_imu_Device_t
  AdiImu(adi_imu_Device_t* p_device);

  //! \brief Destructor for AdiImu class
  ~AdiImu();

  //! \brief Loads Parameters ADI IMU
  //!
  //! \param[in] p_nh_local   Pointer to local nodehandle
  void loadParams(ros::NodeHandle* p_nh_local);

  //! \brief Initializes the ADI IMU
  //!
  //! \return Boolean if successful (true) or not (false)
  bool init(void);

  //! \brief Configures the ADI IMU based on parameters
  //!
  //! \return Boolean if successful (true) or not (false)
  bool config(void);

  //! \brief Return IMU Data rate
  //!
  //! \return IMU Data rate in Hz
  int getImuDataRateHz(void);

  //! \brief Trigger IMU Bias Correction
  //!
  //! \return Boolean if successful (true) or not (false)
  bool triggerBiasCorrectionUpdate(void);
};

#endif //ADI_IMU_H
