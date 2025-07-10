/*
 * Copyright (c) 2022, 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 */

#ifndef IMU_BUF_H
#define IMU_BUF_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <stdint.h>
#include <ros/console.h>

// Include adi_imu_driver headers
#include "adi_imu_driver.h"
#include "imu_spi_buffer.h"

// Include adrd2121_imu headers
#include "adrd2121_imu_common.h"

// Include headers for custom msg and srv
#include "adrd2121_imu/AdiImu.h"
#include "adrd2121_imu/BufStatus.h"

//! \enum Buffer DIO Pin
enum buf_dio_pin_e
{
  BUF_NONE=0,
  BUF_DIO1=1,
  BUF_DIO2=2,
  BUF_DIO3=4,
  BUF_DIO4=8
};

//! \enum Mode of Operation
enum mode_of_operation_e
{
  STREAMING = 1,
  RECOVERY = 2,
};

//! \class ImuBuf
//! \brief Class for ADRD2121
class ImuBuf
{
private:

  //! \brief Name of USB Device
  std::string usb_dev_;

  //! \brief USB Baud Rate
  int usb_baud_;

  //! \brief Enable/Disable IMU Burst Read
  bool b_imu_burst_;

  //! \brief Buffer overflow behavior
  int buf_overflow_;

  //! \brief Boolean to trigger PPS_ENABLE
  bool b_buf_pps_;

  //! \brief IMU DIO output pin is treated as data ready
  int buf_data_rdy_sel_;

  //! \brief Data ready trigger polarity
  int buf_data_rdy_pol_;

  //! \brief PPS trigger polarity
  int buf_pps_pol_;

  //! \brief Host processor DIO output pin acts as a Pulse Per Second (PPS) input
  int buf_pps_sel_;

  //! \brief PPS Input Frequency is (10 ^ (buf_pps_freq) Hz)
  int buf_pps_freq_;

  //! \brief Pins which are directly connected from the host processor to the IMU using an ADG1611
  int buf_pin_pass_;

  //! \brief Pin for the buffer watermark interrupt signal
  int buf_watermark_int_;

  //! \brief Pin for the buffer overflow interrupt signal
  int buf_overflow_int_;

  //! \brief Pin for the error interrupt signal
  int buf_error_int_;

  //! \brief Counter for Previous Data
  uint32_t prev_data_count_;

  //! \brief Counter for IMU Data
  uint32_t imu_data_count_;

  //! \brief Counter for initial data count
  uint64_t start_data_count_;

  //! \brief Counter for data received from Buffer Board
  uint64_t driver_data_count_;

  //! \brief Rollover Counter (in excess of 65536)
  uint32_t rollover_count_;

  //! \brief Counter of dropped data
  uint32_t drop_count_;

  // Initialize Data Array and Counters for Buffer Burst
  //! \brief Buffer length
  uint16_t buf_len_;

  //! \brief Raw data from ADRD2121
  uint16_t burst_raw_[MAX_BUF_LEN_BYTES *10];

  //! \brief Struct for scaled data from ADRD2121
  imubuf_BurstOutput_t buf_burst_out_;

  //! \brief Time stamps
  uint32_t utc_time_, utc_time_us_;

  //! \brief Struct for scaled IMU data from buf_burst_out_
  adi_imu_BurstOutput_t burst_out_;

  //! \brief Current Buffer count
  uint16_t cur_buf_count_;

  //! \brief Boolean for first reading
  //!  Needed when initializing before reading buffer data_raw
  bool b_first_read;

  //! \brief Pointer to main device struct
  adi_imu_Device_t* p_device_;

  //! \brief ROS Message type of imu_pub_
  msg_type_e msg_type_;

  //! \brief Buffer burst count
  int buf_burst_count_;

  //! \brief Boolean to perform recovery during initialization
  bool b_enable_init_recovery_;

  //! \brief ROS Service Server to reset board to factory settings
  ros::ServiceServer factory_reset_service_;

  //! \brief ROS Service Server to clear fault of board
  ros::ServiceServer clear_fault_service_;

  //! \brief ROS Service Server to update flash of board
  ros::ServiceServer flash_update_service_;

  //! \brief ROS Service Server to get status of board
  ros::ServiceServer get_buffer_status_service_;

  //! \brief Mode of Operation
  mode_of_operation_e mode_of_operation_;

  //! \brief Timeout (in ms) when clearing/flushing serial port
  int clear_buffer_timeout_;

public:
  //! \brief Constructor for ImuBuf class
  //!
  //! \param[in] p_device   Pointer to adi_imu_Device_t
  ImuBuf(adi_imu_Device_t* p_device);

  //! \brief Destructor for ImuBuf class
  ~ImuBuf();

  //! \brief Loads Parameters ADRD2121
  //!
  //! \param[in] p_nh_local   Pointer to local nodehandle
  void loadParams(ros::NodeHandle* p_nh_local);

  //! \brief Initializes the ADRD2121
  //!
  //! \return Boolean if successful (true) or not (false)
  bool init(void);

  //! \brief Configures the ADRD2121 based on parameters
  //!
  //! \return Boolean if successful (true) or not (false)
  bool config(void);

  //! \brief Starts capture of ADRD2121 Data
  //! Performs an initial read and discards it
  //!
  //! \return Boolean if successful (true) or not (false)
  bool startBufferRead(void);

  //! \brief Read and Publish Data
  //! Calls startBufferRead during first read;
  //! Calls readPubBufBurstData on succeeding reads
  //!
  //! \param      p_pub       ROS Publisher
  //! \param[in]  frame_name  ROS frame of data to be published
  //! \param[in]  msg_type    ROS message type of data to be published
  //!
  //! \return Boolean if successful (true) or not (false)
  bool readPubData(ros::Publisher* p_pub, std::string frame_name, msg_type_e msg_type);

  //! \brief Read and Publish Data when Buffer Burst read is enabled
  //! For USB as communication interface, Buffer Burst Read is not applicable
  //!
  //! \param      p_pub       ROS Publisher
  //! \param[in]  frame_name  ROS frame of data to be published
  //!
  //! \return Boolean if successful (true) or not (false)
  bool readPubBufBurstData(ros::Publisher* p_pub, std::string frame_name);

  //! \brief Publish Data
  //!
  //! \param      p_pub       ROS Publisher
  //! \param[in]  frame_name  ROS frame of data to be published
  void publishData(ros::Publisher* p_pub, std::string frame_name);

  //! \brief Validate Data
  //!
  //! \return Boolean if valid (true) or not (false)
  bool validateData(void);

  //! \brief Stops capture of ADRD2121 Data
  //!
  //! \return Boolean if successful (true) or not (false)
  bool stopBufferRead(void);

  //! \brief Get status
  //!
  //! \param      p_status      Pointer to status
  //! \param      p_description Pointer to a string for description
  //!
  //! \return Boolean if successful (true) or not (false)
  bool getStatus(uint16_t* p_status, std::string* p_description);

  //! \brief Get status description
  //!
  //! \param      status    Status value
  //!
  //! \return String containing the description of the status
  std::string getStatusDescription(uint16_t status);

  //! \brief Clears fault of board
  //!
  //! \return Boolean if successful (true) or not (false)
  bool clearFault(void);

  //! \brief Updates flash of board
  //!
  //! \return Boolean if successful (true) or not (false)
  bool flashUpdate(void);

  //! \brief Reset the board to factory settings
  //!
  //! \return Boolean if successful (true) or not (false)
  bool factoryReset(void);

  //! \brief Service Server callback for get_buffer_status_service_
  //!
  //! \param req,res  Request and Response to the Service
  //!
  //! \return Boolean if successful (true) or not (false)
  bool getStatusCallback(adrd2121_imu::BufStatus::Request &req, adrd2121_imu::BufStatus::Response &res);

  //! \brief Service Server callback for clear_fault_service_
  //!
  //! \param req,res  Request and Response to the Service
  //!
  //! \return Boolean if successful (true) or not (false)
  bool clearFaultCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  //! \brief Service Server callback for flash_update_service_
  //!
  //! \param req,res  Request and Response to the Service
  //!
  //! \return Boolean if successful (true) or not (false)
  bool flashUpdateCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  //! \brief Service Server callback for factory_reset_service_
  //!
  //! \param req,res  Request and Response to the Service
  //!
  //! \return Boolean if successful (true) or not (false)
  bool factoryResetCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  //! \brief Initialize service servers
  //!
  //! \param[in]  p_nh_local          Pointer to local Nodehandle
  //! \param[in]  mode_of_operation   Mode of operation
  void initServiceServers(ros::NodeHandle* p_nh_local, mode_of_operation_e mode_of_operation);

  //! \brief Performs recovery sequence of board
  //!
  //! \return Boolean if successful (true) or not (false)
  bool recoverBoard(void);

  //! \brief Setter of mode_of_operation_
  //!
  //! \param[in]  mode_of_operation   Mode of operation
  void setModeOfOperation(mode_of_operation_e mode_of_operation);

  //! \brief Detects if ADRD2121 is connected
  //!
  //! \return Boolean if successful (true) or not (false)
  bool detect(void);
};

#endif //IMU_BUF_H
