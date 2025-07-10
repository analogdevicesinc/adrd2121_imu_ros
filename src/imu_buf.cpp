/*
 * Copyright (c) 2022, 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 */

#include "imu_buf.h"
#include <sys/ioctl.h>
#include <termios.h>

const char * const BUFDIOtoString(int dio);

ImuBuf::ImuBuf(adi_imu_Device_t* p_device):
  buf_burst_out_{0},
  burst_out_{0},
  p_device_(NULL)
{
  // Initialize object

  // Initialize Data Counters for Checking CRC
  prev_data_count_ = 0; //prevDataCnt
  imu_data_count_ = 0; //imuDataCount
  start_data_count_ = 0; //startDataCount
  driver_data_count_ = 0; // driverDataCount
  rollover_count_ = 0;  // rolloverCnt
  drop_count_ = 0;  // dropCount

  // Initialize Data Array and Counters for Buffer Burst
  buf_len_ = 0;

  cur_buf_count_ = 0;

  b_first_read = true;

  b_enable_init_recovery_=true;

  mode_of_operation_ = STREAMING;

  p_device_=p_device;
}

ImuBuf::~ImuBuf()
{
  bool b_success=false;
  // Destroy object
  if(p_device_->uartDev.status >= IMUBUF_UART_CONFIGURED)
  {
    b_success=this->stopBufferRead();
  }
}

void ImuBuf::loadParams(ros::NodeHandle* p_nh_local)
{
  bool b_default = false;

  ROS_INFO("Loading [ADRD2121] Parameters...");

  b_default = p_nh_local->param<std::string>("usb_dev", usb_dev_, "/dev/ttyACM0");
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting USB Dev: " << usb_dev_);
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default USB Dev: " << usb_dev_);

  b_default = p_nh_local->param("usb_baud", usb_baud_, 921600);
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting USB Baud Rate: " << usb_baud_);
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default USB Baud Rate: " << usb_baud_);

  b_default = p_nh_local->param("enable_imu_burst", b_imu_burst_, true);
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting IMU Burst Mode: " << BoolToString(b_imu_burst_));
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default IMU Burst Mode: " << BoolToString(b_imu_burst_));

  b_default = p_nh_local->param("buf_overflow", buf_overflow_, 0);
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting Buffer Overflow Behavior: " << ((buf_overflow_) ? \
                                    "Replace Oldest Data" : "Stop Sampling"));
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using Buffer Overflow Behavior: " << ((buf_overflow_) ? \
    "Replace Oldest Data" : "Stop Sampling"));

  b_default = p_nh_local->param("enable_buf_pps", b_buf_pps_, false);
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting PPS Mode: " << BoolToString(b_buf_pps_));
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default PPS Mode: " << BoolToString(b_buf_pps_));

  b_default = p_nh_local->param("buf_data_rdy_sel", buf_data_rdy_sel_, static_cast<int>(BUF_DIO1));
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting DR_SELECT: " << BUFDIOtoString(buf_data_rdy_sel_));
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default DR_SELECT: " << \
    BUFDIOtoString(buf_data_rdy_sel_));

  b_default = p_nh_local->param("buf_data_rdy_pol", buf_data_rdy_pol_, 1);
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting DR_POLARITY: " << buf_data_rdy_pol_);
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default DR_POLARITY: " << buf_data_rdy_pol_);

  b_default = p_nh_local->param("buf_pps_sel", buf_pps_sel_,  static_cast<int>(BUF_NONE));
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting PPS_SELECT: " << BUFDIOtoString(buf_pps_sel_));
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default PPS_SELECT: " << BUFDIOtoString(buf_pps_sel_));

  b_default = p_nh_local->param("buf_pps_pol", buf_pps_pol_, 0);
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting PPS_POLARITY: " << buf_pps_pol_);
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default PPS_POLARITY: " << buf_pps_pol_);

  b_default = p_nh_local->param("buf_pps_freq", buf_pps_freq_, 0);
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting PPS_FREQUENCY: " << buf_pps_freq_);
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default PPS_FREQUENCY: " << buf_pps_freq_);

  b_default = p_nh_local->param("buf_pin_pass", buf_pin_pass_,static_cast<int>(BUF_NONE));
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting PIN_PASS: " << BUFDIOtoString(buf_pin_pass_));
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default PIN_PASS: " << BUFDIOtoString(buf_pin_pass_));

  b_default = p_nh_local->param("buf_watermark_int", buf_watermark_int_, static_cast<int>(BUF_NONE));
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting WATERMARK_INT: " << \
    BUFDIOtoString(buf_watermark_int_));
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default WATERMARK_INT: " << \
    BUFDIOtoString(buf_watermark_int_));

  b_default = p_nh_local->param("buf_overflow_int", buf_overflow_int_, static_cast<int>(BUF_NONE));
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting OVERFLOW_INT: " << BUFDIOtoString(buf_overflow_int_));
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default OVERFLOW_INT: " << \
    BUFDIOtoString(buf_overflow_int_));

  b_default = p_nh_local->param("buf_error_int", buf_error_int_, static_cast<int>(BUF_NONE));
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting ERROR_INT: " << BUFDIOtoString(buf_error_int_));
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default ERROR_INT: " << BUFDIOtoString(buf_error_int_));

  b_default = p_nh_local->param("buf_burst_count", buf_burst_count_, 1);
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121]Setting Burst Count: " << buf_burst_count_);
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default Burst Count: " << buf_burst_count_);

  b_default = p_nh_local->param("enable_init_recovery", b_enable_init_recovery_, true);
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting Enable Init Recovery: " << b_enable_init_recovery_);
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default Enable Init Recovery: " <<
    b_enable_init_recovery_);

  b_default = p_nh_local->param("clear_buffer_timeout", clear_buffer_timeout_, 500);
  ROS_DEBUG_STREAM_COND(b_default, "[ADRD2121] Setting Clear Buffer Timeout (ms): " << clear_buffer_timeout_);
  ROS_WARN_STREAM_COND(!b_default, "[ADRD2121] Using default Clear Buffer Timeout (ms): " << \
    clear_buffer_timeout_);

  if((clear_buffer_timeout_ < 500) || (clear_buffer_timeout_ > 3000))
  {
    clear_buffer_timeout_=500;
    p_nh_local->setParam("clear_buffer_timeout", 500);
    ROS_WARN_STREAM("[ADRD2121] Clear Buffer timeout out of range, setting to default " <<
      clear_buffer_timeout_ << "ms");
  }

  // Update the adi_imu_Device_t
  // Only USB is supported
  p_device_->devType=IMU_HW_UART;
  p_device_->uartDev.dev=usb_dev_.c_str();
  p_device_->uartDev.baud=(uint32_t)usb_baud_;

  p_device_->enable_buffer=IMU_TRUE; //Buffer is always enabled.
  ROS_INFO("Loaded [ADRD2121] Parameters.");
}

bool ImuBuf::init(void)
{
  bool b_success=false;
  int ret =-1;
  uint16_t status=0xFFFF;
  std::string status_decription="";

  if(b_enable_init_recovery_)
  {
    b_success=this->recoverBoard();
  }

  b_success=this->getStatus(&status, &status_decription);

  if(b_success)
  {
    // Check if there are any errors
    uint16_t status_error_mask = BITM_ISENSOR_STATUS_SPI_ERROR | BITM_ISENSOR_STATUS_SPI_OVRFLW | \
      BITM_ISENSOR_STATUS_OVERRUN | BITM_ISENSOR_STATUS_DMA_ERROR | BITM_ISENSOR_STATUS_SCRIPT_ERROR | \
      BITM_ISENSOR_STATUS_FLASH_ERROR | BITM_ISENSOR_STATUS_FLASH_UPD_ERROR | BITM_ISENSOR_STATUS_FAULT | \
      BITM_ISENSOR_STATUS_WATCHDOG;

    if(status_error_mask & status)
    {
      ROS_ERROR_STREAM("[ADRD2121] Board has an error status.");
      b_success = false;
    }
    else
    {
      b_success=this->detect();
    }
  }

  if(b_success)
  {
#if defined(ADRD2121_IMU_WITH_HW)
    ret=imubuf_init(p_device_);
#else
    ret=0;
#endif
    ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] initialization unsuccessful. ERROR CODE: "<< ret \
      <<" Check adi_imu_Error_e.");
    ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] initialization successful");
    b_success=(ret>=0)? true : false;
  }

  return b_success;
}

bool ImuBuf::config(void)
{
  bool b_success=false;
  int ret=-1;
  imubuf_ImuDioConfig_t buf_dio_config;
#if defined(ADRD2121_IMU_WITH_HW)
  // Configure ADRD2121 Buffer DIO Pins
  buf_dio_config.dataReadyPin = buf_data_rdy_sel_;
  buf_dio_config.dataReadyPolarity = buf_data_rdy_pol_;
  if(b_buf_pps_)
  {
    buf_dio_config.ppsPin = buf_pps_sel_;
    buf_dio_config.ppsPolarity = buf_pps_pol_;
  }
  else
  {
    buf_dio_config.ppsPin = 0;
    buf_dio_config.ppsPolarity = 0;
  }
  buf_dio_config.passThruPin = buf_pin_pass_;
  buf_dio_config.watermarkIrqPin= buf_watermark_int_;
  buf_dio_config.overflowIrqPin = buf_overflow_int_;
  buf_dio_config.errorIrqPin = buf_error_int_;

  ret=imubuf_SetDioConfig(p_device_,&buf_dio_config);
  ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Set DIO Pin Config unsuccessful. ERROR CODE: "<< ret <<"\
    Check adi_imu_Error_e.");
  ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Set DIO Pin Config successful");
  b_success=(ret>=0)? true : false;

  // Enable/Disable PPS Sync
  if(b_success && b_buf_pps_)
  {
    ret=imubuf_EnablePPSSync(p_device_);
    ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Enable PPS Sync unsuccessful. ERROR CODE: "<< ret <<"\
      Check adi_imu_Error_e.");
    ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Enable PPS Sync successful");
    b_success=(ret>=0)? true : false;

    // Check if UTC time is properly set
    uint32_t epoch_time = time(NULL);
    uint32_t epoch_readback = 0;
    if(b_success)
    {
      ret=imubuf_SetUTC(p_device_,epoch_time);
      ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Set UTC unsuccessful. ERROR CODE: "<< ret \
        <<" Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Set UTC successful");
      b_success=(ret>=0)? true : false;
    }

    if(b_success)
    {
      ret=imubuf_GetUTC(p_device_,&epoch_readback);
      ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Get UTC unsuccessful. ERROR CODE: "<< ret \
        <<" Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Get UTC successful");
      b_success=(ret>=0)? true : false;;
    }

    if(b_success && (epoch_readback<epoch_time))
    {
      ROS_ERROR_STREAM("[ADRD2121] UTC Time not set properly.");
      b_success=false;
    }
  }
  else if (b_success && !b_buf_pps_)
  {
    ret=imubuf_DisablePPSSync(p_device_);
    ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Disable PPS Sync unsuccessful. ERROR CODE: "<< ret <<"\
      Check adi_imu_Error_e.");
    ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Disable PPS Sync successful");
    b_success=(ret>=0)? true : false;
  }

  if(b_success && b_imu_burst_)
  {
    ret=imubuf_SetPatternImuBurst(p_device_);
    ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Set Pattern Imu Burst unsuccessful. ERROR CODE: "<< ret <<\
      " Check adi_imu_Error_e.");
    ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Set Pattern Imu Burst successful");
    b_success=(ret>=0)? true : false;
  }
  else if(b_success && !b_imu_burst_)
  {
    if(p_device_->prodId==16495)
    {
      uint16_t buf_pattern[] = {
          IMUBUF_PATTERN_READ_REG(REG_SYS_E_FLAG_49x),\
          IMUBUF_PATTERN_READ_REG(REG_TEMP_OUT_49x),\
          IMUBUF_PATTERN_READ_REG(REG_X_GYRO_LOW_49x),\
          IMUBUF_PATTERN_READ_REG(REG_X_GYRO_OUT_49x),\
          IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_LOW_49x),\
          IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_OUT_49x),\
          IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_LOW_49x),\
          IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_OUT_49x),\
          IMUBUF_PATTERN_READ_REG(REG_X_ACCL_LOW_49x),\
          IMUBUF_PATTERN_READ_REG(REG_X_ACCL_OUT_49x),\
          IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_LOW_49x),\
          IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_OUT_49x),\
          IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_LOW_49x),\
          IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_OUT_49x),\
          IMUBUF_PATTERN_READ_REG(REG_DATA_CNT_49x),\
          IMUBUF_PATTERN_READ_REG(REG_CRC_LWR_49x),\
          IMUBUF_PATTERN_READ_REG(REG_CRC_UPR_49x), 0x0000,\
      };
      uint16_t buf_pattern_len = (uint16_t) (sizeof(buf_pattern)/sizeof(uint16_t));
      ret=imubuf_SetPatternRaw(p_device_,buf_pattern_len,buf_pattern);
      ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Set Pattern Imu unsuccessful. ERROR CODE: "<< ret <<\
        " Check adi_imu_Error_e.");
      ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Set Pattern Imu successful");
      b_success=(ret>=0)? true : false;
    }
    else if(p_device_->prodId==16470 || p_device_->prodId==16500)
    {
      // TODO
    }
  }

  // Configure and Set Burst Mode
  imubuf_BufConfig_t buf_config;
  buf_config.overflowAction=buf_overflow_;
  buf_config.imuBurstEn=b_imu_burst_;
  buf_config.bufBurstEn=0; // Always disabled for USB
  if(p_device_->prodId == 16495)
  {
    buf_config.imuPageAddr = 1;
  }
  else if(p_device_->prodId == 16470 || p_device_->prodId == 16500)
  {
    buf_config.imuPageAddr = 0;
  }

  if(b_success)
  {
    ret=imubuf_SetBufConfig(p_device_,&buf_config);
    ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Set Buf Config unsuccessful. ERROR CODE: "<< ret \
      <<" Check adi_imu_Error_e.");
    ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Set Buf Config successful");
    b_success=(ret>=0)? true : false;
  }

#else
  b_success=true;
#endif

  ROS_INFO_STREAM_COND(b_success,"[ADRD2121] Configuration successful.");

#if defined(ADRD2121_IMU_WITH_HW)
  imubuf_DevInfo_t imuBufInfo;
  if ((ret = imubuf_GetInfo(p_device_, &imuBufInfo)) < 0) return ret;
  if ((ret = imubuf_PrintInfo(p_device_, &imuBufInfo)) < 0) return ret;
#endif

  return b_success;
}

bool ImuBuf::startBufferRead(void)
{
  bool b_success=false;
  int ret=-1;

#if defined(ADRD2121_IMU_WITH_HW)
  // Start Capture in ADRD2121
  ret=imubuf_StartCapture(p_device_,IMU_FALSE,&cur_buf_count_);
#else
  ret=0;
#endif
  ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Start Capture unsuccessful. ERROR CODE: "<< ret \
    <<" Check adi_imu_Error_e.");
  ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Start Capture successful");
  b_success= (ret>=0)? true : false;

  if(b_success)
  {
#if defined(ADRD2121_IMU_WITH_HW)
    // Do initial burst read and discard it
    ret=imubuf_ReadBurstN(p_device_,5,(uint16_t*)burst_raw_, &buf_len_);
#else
    ret=0;
#endif
    ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Initial Burst Read unsuccessful. ERROR CODE: "<< ret <<\
      " Check adi_imu_Error_e.");
    ROS_DEBUG_STREAM_COND(ret>=0,"[ADRD2121] Initial Burst Read successful");
    b_success= (ret>=0)? true : false;
  }

  return b_success;
}

bool ImuBuf::readPubData(ros::Publisher* p_pub, std::string frame_name, msg_type_e msg_type)
{
  int ret = -1;
  bool b_success = false;

  if(b_first_read)
  {
    b_success=this->startBufferRead();
    // Set Message type
    msg_type_ = msg_type;
    b_first_read=false;
  }
  else
  {
    b_success=true;
  }

  // Read buffer data
  ROS_DEBUG_STREAM("Read Buffer Data Stream (USB)");
  b_success=readPubBufBurstData(p_pub,frame_name);

  return b_success;
}

bool ImuBuf::readPubBufBurstData(ros::Publisher* p_pub, std::string frame_name)
{
  int ret=-1;
  bool b_success=false;
  bool b_valid_data=false;

#if defined(ADRD2121_IMU_TIMING_DEBUG)
  print_time(std::string("T0 @"));
#endif
  // Read Buffer Burst depending on buf_burst_count_
#if defined(ADRD2121_IMU_WITH_HW)
  ret=imubuf_ReadBurstN(p_device_,buf_burst_count_,(uint16_t *)burst_raw_, &buf_len_);
#else
  ret=0;
#endif
  ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Buffer Burst Read unsuccessful. ERROR CODE: "<< ret <<\
      " Check adi_imu_Error_e.");
  ROS_DEBUG_STREAM_COND(ret>=0,"[ADRD2121] Buffer Burst Read successful");
  b_success= (ret>=0) ? true : false;

#if defined(ADRD2121_IMU_TIMING_DEBUG)
  print_time(std::string("T1 @"));
#endif

  for(int i=0;i<buf_burst_count_;i++)
  {
    // imubuf_BurstOutputRaw_t to imubuf_BurstOutput_t
    if(b_success)
    {
#if defined(ADRD2121_IMU_WITH_HW)
      ret=imubuf_ScaleBurstOut(p_device_,(imubuf_BurstOutputRaw_t*)(burst_raw_ + buf_len_*i),&buf_burst_out_);
#else
      ret=0;
#endif
      ROS_DEBUG_STREAM_COND(ret>=0,"[ADRD2121] Scale Burst Out successful");
      b_success= (ret>=0) ? true : false;
      if(ret<0)
      {
        ROS_ERROR_STREAM("[ADRD2121] Scale Burst Out unsuccessful. ERROR CODE: "<< ret <<\
        " Check adi_imu_Error_e.");
        break;
      }
    }

    // Check if bytes need to be swapped
    memset((uint8_t*)&burst_out_,0,sizeof(burst_out_));
    adi_imu_Boolean_e en_byte_swap;
    if(p_device_->devType==IMU_HW_UART && p_device_->uartDev.status >= IMUBUF_UART_READY)
    {
      en_byte_swap=IMU_FALSE;
    }
    else
    {
      en_byte_swap=IMU_TRUE;
    }

    if(b_success)
    {
#if defined(ADRD2121_IMU_WITH_HW)
      if(p_device_->prodId == 16495)
      {
        ret=adi_imu_ScaleBurstOut(p_device_, buf_burst_out_.data, IMU_TRUE,en_byte_swap, &burst_out_);
      }
      else if(p_device_->prodId == 16470 || p_device_->prodId == 16500)
      {
        // No Burst ID in ADIS16470 and ADIS16400
        ret=adi_imu_ScaleBurstOut(p_device_, buf_burst_out_.data, IMU_FALSE,en_byte_swap, &burst_out_);
      }
#else
      ret=0;
      burst_out_.gyro.x =1;
      burst_out_.gyro.y =1;
      burst_out_.gyro.z =1;
      burst_out_.accl.x =1;
      burst_out_.accl.y =1;
      burst_out_.accl.z =1;
#endif
      ROS_ERROR_STREAM_COND(ret<0,"ADI IMU Scale Burst Out unsuccessful. ERROR CODE: "<< ret <<\
        " Check adi_imu_Error_e.");
      ROS_DEBUG_STREAM_COND(ret>=0,"ADI IMU Scale Burst Out successful");
      b_success= (ret>=0) ? true : false;

      if(ret==Err_imu_BurstFrameInvalid_e)
      {
        ROS_ERROR_STREAM("ADI IMU Scale Burst Out: BurstFrameInvalid; will continue loop.");
        b_success=true;
        continue;
      }
    }

    if(b_success)
    {
      // Parse UTC timestamps
      utc_time_=buf_burst_out_.bufUtcTime;
      utc_time_us_=buf_burst_out_.bufTimestamp;

      b_valid_data=this->validateData();
      if(b_valid_data)
      {
#if defined(ADRD2121_IMU_TIMING_DEBUG)
        print_time(std::string("T2 @"));
#endif
        this->publishData(p_pub,frame_name);
#if defined(ADRD2121_IMU_TIMING_DEBUG)
        print_time(std::string("T3 @"));
#endif
      }
    }
  }
  return b_success;
}

bool ImuBuf::validateData(void)
{
  bool b_success=false;
  // Initialize Data Counters for Checking CRC
  uint32_t total_data_count = 0; // driverCntPlusDropCnt
  uint32_t drop_count_current = 0;  // dropCountCurrent

  uint32_t calculated_crc = 0;

#if defined(ADRD2121_IMU_TIMING_DEBUG)
      print_time(std::string("T4 @"));
#endif

  /* Calculate checksum / CRC */
  if(p_device_->dataFormat == IMU_DATA_32BIT)
  {
    if(p_device_->prodId == 16500)
    {

    /** BURST READ FORMAT
      * For 16500, BRF for:
      *   0x0000, DIAG_STAT, X_GYRO_LOW, X_GYRO_OUT, Y_GYRO_LOW, Y_GYRO_OUT, Z_GYRO_LOW, Z_GYRO_OUT,
      *   X_ACCL_LOW, X_ACCL_OUT, Y_ACCL_LOW, Y_ACCL_OUT, Z_ACCL_LOW, Z_ACCL_OUT, TEMP_OUT, DATA_CNT, CRC
      *
      * CRC = DIAG_STAT[15:8] + DIAG_STAT[7:0] +
      *       X_GYRO_LOW[15:8] + X_GYRO_LOW[7:0] + X_GYRO_OUT[15:8] + X_GYRO_OUT[7:0] +
      *       Y_GYRO_LOW[15:8] + Y_GYRO_LOW[7:0] + Y_GYRO_OUT[15:8] + Y_GYRO_OUT[7:0] +
      *       Z_GYRO_LOW[15:8] + Z_GYRO_LOW[7:0] + Z_GYRO_OUT[15:8] + Z_GYRO_OUT[7:0] +
      *       X_ACCL_LOW[15:8] + X_ACCL_LOW[7:0] + X_ACCL_OUT[15:8] + X_ACCL_OUT[7:0] +
      *       Y_ACCL_LOW[15:8] + Y_ACCL_LOW[7:0] + Y_ACCL_OUT[15:8] + Y_ACCL_OUT[7:0] +
      *       Z_ACCL_LOW[15:8] + Z_ACCL_LOW[7:0] + Z_ACCL_OUT[15:8] + Z_ACCL_OUT[7:0] +
      *       TEMP_OUT[15:8] + TEMP_OUT[7:0] + DATA_CNT[15:8] + DATA_CNT[7:0]
      *
      **/

      ROS_DEBUG("[DEBUG] Burst IMU Data Raw: %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X \
        %X %X %X %X %X %X %X %X %X %X %X %X %X", \
        buf_burst_out_.data[0], buf_burst_out_.data[1], buf_burst_out_.data[2], buf_burst_out_.data[3], \
        buf_burst_out_.data[4], buf_burst_out_.data[5], buf_burst_out_.data[6], buf_burst_out_.data[7], \
        buf_burst_out_.data[8], buf_burst_out_.data[9], buf_burst_out_.data[10], buf_burst_out_.data[11], \
        buf_burst_out_.data[12],buf_burst_out_.data[13],buf_burst_out_.data[14], buf_burst_out_.data[15], \
        buf_burst_out_.data[16],buf_burst_out_.data[17],buf_burst_out_.data[18], buf_burst_out_.data[19], \
        buf_burst_out_.data[20],buf_burst_out_.data[21],buf_burst_out_.data[22], buf_burst_out_.data[23], \
        buf_burst_out_.data[24],buf_burst_out_.data[25],buf_burst_out_.data[26], buf_burst_out_.data[27], \
        buf_burst_out_.data[28],buf_burst_out_.data[29],buf_burst_out_.data[30], buf_burst_out_.data[31], \
        buf_burst_out_.data[32], buf_burst_out_.data[33]);

      calculated_crc = (uint16_t)(buf_burst_out_.data[2] + buf_burst_out_.data[3] + \
        buf_burst_out_.data[4] + buf_burst_out_.data[5] + buf_burst_out_.data[6] + buf_burst_out_.data[7] + \
        buf_burst_out_.data[8] + buf_burst_out_.data[9] + buf_burst_out_.data[10] + buf_burst_out_.data[11] + \
        buf_burst_out_.data[12] +buf_burst_out_.data[13] +buf_burst_out_.data[14] + buf_burst_out_.data[15] + \
        buf_burst_out_.data[16] +buf_burst_out_.data[17] +buf_burst_out_.data[18] + buf_burst_out_.data[19] + \
        buf_burst_out_.data[20] +buf_burst_out_.data[21] +buf_burst_out_.data[22] + buf_burst_out_.data[23] + \
        buf_burst_out_.data[24] +buf_burst_out_.data[25] +buf_burst_out_.data[26] + buf_burst_out_.data[27] + \
        buf_burst_out_.data[28] +buf_burst_out_.data[29] +buf_burst_out_.data[30] + buf_burst_out_.data[31]);
    }
    else if(p_device_->prodId == 16495)
    {
    /** BURST READ FORMAT
      * For 16495, BRF for (fsclk < 3MHz):
      *   0x0000, 0xA5A5 (BURST_ID), SYS_E_FLAG, TEMP_OUT, X_GYRO_LOW, X_GYRO_OUT, Y_GYRO_LOW, Y_GYRO_OUT,
      *   Z_GYRO_LOW, Z_GYRO_OUT, X_ACCL_LOW, X_ACCL_OUT, Y_ACCL_LOW, Y_ACCL_OUT, Z_ACCL_LOW, Z_ACCL_OUT,
      *   DATA_CNT, CRC_LWR, CRC_UPR
      *
      * CRC is based on CRC32 Calculation
      **/

      ROS_DEBUG("[DEBUG] Burst IMU Data Raw: %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X \
        %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X", \
        buf_burst_out_.data[0], buf_burst_out_.data[1], buf_burst_out_.data[2], buf_burst_out_.data[3], \
        buf_burst_out_.data[4], buf_burst_out_.data[5], buf_burst_out_.data[6], buf_burst_out_.data[7], \
        buf_burst_out_.data[8], buf_burst_out_.data[9], buf_burst_out_.data[10], buf_burst_out_.data[11], \
        buf_burst_out_.data[12],buf_burst_out_.data[13],buf_burst_out_.data[14], buf_burst_out_.data[15], \
        buf_burst_out_.data[16],buf_burst_out_.data[17],buf_burst_out_.data[18], buf_burst_out_.data[19], \
        buf_burst_out_.data[20],buf_burst_out_.data[21],buf_burst_out_.data[22], buf_burst_out_.data[23], \
        buf_burst_out_.data[24],buf_burst_out_.data[25],buf_burst_out_.data[26], buf_burst_out_.data[27], \
        buf_burst_out_.data[28],buf_burst_out_.data[29],buf_burst_out_.data[30], buf_burst_out_.data[31], \
        buf_burst_out_.data[32], buf_burst_out_.data[33], buf_burst_out_.data[34], buf_burst_out_.data[35], \
        buf_burst_out_.data[36], buf_burst_out_.data[37], buf_burst_out_.data[38], buf_burst_out_.data[39]);

      calculated_crc = (unsigned)0xFFFFFFFF;
      uint16_t *p_data = (uint16_t*)(buf_burst_out_.data + 6);
      calculated_crc = crc32_block(calculated_crc,p_data,15);
      calculated_crc = calculated_crc ^ (unsigned)0xFFFFFFFF;
    }
  }
  else if(p_device_->dataFormat == IMU_DATA_16BIT)
  {
    /** BURST READ FORMAT
      * For 16500 or 16470, BRF for:
      *   0x0000, DIAG_STAT, X_GYRO_OUT, Y_GYRO_OUT, Z_GYRO_OUT, X_ACCL_OUT, Y_ACCL_OUT, Z_ACCL_OUT,
      *   TEMP_OUT, DATA_CNT, CRC
      *
      * CRC = DIAG_STAT[15:8] + DIAG_STAT[7:0] +
      *       X_GYRO_OUT[15:8] + X_GYRO_OUT[7:0] +
      *       Y_GYRO_OUT[15:8] + Y_GYRO_OUT[7:0] +
      *       Z_GYRO_OUT[15:8] + Z_GYRO_OUT[7:0] +
      *       X_ACCL_OUT[15:8] + X_ACCL_OUT[7:0] +
      *       Y_ACCL_OUT[15:8] + Y_ACCL_OUT[7:0] +
      *       Z_ACCL_OUT[15:8] + Z_ACCL_OUT[7:0] +
      *       TEMP_OUT[15:8] + TEMP_OUT[7:0] +
      *       DATA_CNT[15:8] + DATA_CNT[7:0]
      **/

    ROS_DEBUG("[DEBUG] Burst IMU Data Raw: %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X", \
      buf_burst_out_.data[0], buf_burst_out_.data[1], buf_burst_out_.data[2], buf_burst_out_.data[3], \
      buf_burst_out_.data[4], buf_burst_out_.data[5], buf_burst_out_.data[6], buf_burst_out_.data[7], \
      buf_burst_out_.data[8], buf_burst_out_.data[9], buf_burst_out_.data[10], buf_burst_out_.data[11], \
      buf_burst_out_.data[12],buf_burst_out_.data[13],buf_burst_out_.data[14], buf_burst_out_.data[15], \
      buf_burst_out_.data[16],buf_burst_out_.data[17],buf_burst_out_.data[18], buf_burst_out_.data[19], \
      buf_burst_out_.data[20],buf_burst_out_.data[21]);

    calculated_crc = (uint16_t)(buf_burst_out_.data[2] + buf_burst_out_.data[3] + \
      buf_burst_out_.data[4] + buf_burst_out_.data[5] + buf_burst_out_.data[6] + buf_burst_out_.data[7] + \
      buf_burst_out_.data[8] + buf_burst_out_.data[9] + buf_burst_out_.data[10] + buf_burst_out_.data[11] + \
      buf_burst_out_.data[12] +buf_burst_out_.data[13] +buf_burst_out_.data[14] + buf_burst_out_.data[15] + \
      buf_burst_out_.data[16] +buf_burst_out_.data[17] +buf_burst_out_.data[18] + buf_burst_out_.data[19]);
  }

#if defined(ADRD2121_IMU_TIMING_DEBUG)
      print_time(std::string("T5 @"));
#endif

  ROS_DEBUG("[DEBUG] CRC: %X", burst_out_.crc);
  ROS_DEBUG("[DEBUG] Calculated CRC: %X", calculated_crc);

  /* process only valid burst data */
  if (burst_out_.crc == calculated_crc)
  {
    /* update data counters for the first time */
    if (driver_data_count_ == 0)
    {
      if (burst_out_.dataCntOrTimeStamp > 0)
      {
        prev_data_count_ = burst_out_.dataCntOrTimeStamp - 1;
      }
      else
      {
        prev_data_count_ = 0;
      }
      driver_data_count_ = burst_out_.dataCntOrTimeStamp;
      start_data_count_ = driver_data_count_;
    }
    else driver_data_count_++;

    /* update rollover count on every overflow (i.e. 65535 to 0 transition) */
    if ( (prev_data_count_ > 0) && (burst_out_.dataCntOrTimeStamp < prev_data_count_)) rollover_count_++;
    prev_data_count_ = burst_out_.dataCntOrTimeStamp;
    imu_data_count_ = burst_out_.dataCntOrTimeStamp + (rollover_count_ * 65536);
    total_data_count = driver_data_count_ + drop_count_;

    if (imu_data_count_ > total_data_count)
    {
      drop_count_current = imu_data_count_ - total_data_count;
      drop_count_ += drop_count_current;
      ROS_DEBUG_STREAM("drop_count= " << drop_count_);
    }
    else if (imu_data_count_ < total_data_count)
    {
      ROS_WARN_STREAM("Data count invalid. IMU Count=" << imu_data_count_ << " Driver count=" << driver_data_count_\
        << " Drop Count=" << drop_count_ << " dataCntOrTimeStamp: " << burst_out_.dataCntOrTimeStamp);
    }

    ROS_DEBUG_STREAM("IMU Count=" << imu_data_count_ << " Driver count=" << driver_data_count_\
      << " Drop Count=" << drop_count_ << " dataCntOrTimeStamp: " << burst_out_.dataCntOrTimeStamp << " rollover_count_= " << rollover_count_ \
      << " total_data_count= " << total_data_count);

    b_success=true;
  }
  else
  {
    ROS_WARN("Invalid CRC. Received: %X Actual: %X", burst_out_.crc, calculated_crc);
  }

  return b_success; // For debugging purposes, always return true
}

void ImuBuf::publishData(ros::Publisher* p_pub, std::string frame_name)
{
  if(SENSOR_MSGS_IMU==msg_type_)
  {
    sensor_msgs::Imu imu_msg;

    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = frame_name.c_str();
    imu_msg.angular_velocity.x = burst_out_.gyro.x*(M_PI/180);
    imu_msg.angular_velocity.y = burst_out_.gyro.y*(M_PI/180);
    imu_msg.angular_velocity.z = burst_out_.gyro.z*(M_PI/180);
    imu_msg.linear_acceleration.x = burst_out_.accl.x;
    imu_msg.linear_acceleration.y = burst_out_.accl.y;
    imu_msg.linear_acceleration.z = burst_out_.accl.z;
    p_pub->publish(imu_msg);
  }
  else if (ADI_IMU_MSG==msg_type_)
  {
    adrd2121_imu::AdiImu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = frame_name.c_str();
    imu_msg.angular_velocity.x = burst_out_.gyro.x*(M_PI/180);
    imu_msg.angular_velocity.y = burst_out_.gyro.y*(M_PI/180);
    imu_msg.angular_velocity.z = burst_out_.gyro.z*(M_PI/180);
    imu_msg.linear_acceleration.x = burst_out_.accl.x;
    imu_msg.linear_acceleration.y = burst_out_.accl.y;
    imu_msg.linear_acceleration.z = burst_out_.accl.z;
    imu_msg.imu_count = imu_data_count_;
    imu_msg.driver_count = driver_data_count_;
    imu_msg.drop_count = drop_count_;
    imu_msg.buf_utc_sec = utc_time_;
    imu_msg.buf_utc_usec = utc_time_us_;
    p_pub->publish(imu_msg);
  }

}

bool ImuBuf::stopBufferRead()
{
  int ret=-1;
  bool b_success=false;
  ros::Time start_time;
  ros::Time current_time;
  bool b_clear_timeout_reached=false;
  int buffer_size=0;

  if(p_device_->uartDev.status >= IMUBUF_UART_CONFIGURED)
  {
    ROS_INFO_STREAM("[ADRD2121] Stop Capture.");
#if ADRD2121_IMU_WITH_HW
    ret=imubuf_StopCapture(p_device_, &cur_buf_count_);
#else
    ret=0;
#endif
    ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Stop Capture unsuccessful. ERROR CODE: " \
                          << ret <<" Check adi_imu_Error_e.");
    ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Stop Capture successful");
  }

  if(ret>=0)
  {
    b_success = true;

    // Reset driver data count
    driver_data_count_ = 0;
    rollover_count_ = 0;
  }

  start_time=ros::Time::now();
  do
  {
    ROS_INFO_STREAM_ONCE("[ADRD2121] Flushing serial port...");
    current_time=ros::Time::now();
    double duration_s = (current_time.toSec() - start_time.toSec());
    int duration_ms = (static_cast<int>(duration_s) * 1000);
    if(duration_ms < clear_buffer_timeout_)
    {
      // Manually flush INPUT
      tcflush(p_device_->uartDev.fd, TCIFLUSH);
      // Manually get the number of data in UART Buffer
      ioctl(p_device_->uartDev.fd, FIONREAD, &buffer_size);
    }
    else
    {
      ROS_WARN_STREAM("[ADRD2121] Flushing serial port reached timeout..." << buffer_size << " bytes left.");
      b_clear_timeout_reached = true;
    }
  } while ((!b_clear_timeout_reached) && (buffer_size > 0));

  ROS_INFO_STREAM_COND(buffer_size==0,"[ADRD2121] Flushing serial port successful... ");
  return b_success;
}

bool ImuBuf::getStatus(uint16_t* p_status, std::string* p_description)
{
  bool b_success=false;
  imubuf_SysStatus_t buf_status;
  int ret = -1;

  ROS_INFO("================================");
  ret=imubuf_GetSysStatus(p_device_,&buf_status);
  ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Getting status of Board failed. ERROR CODE: "<< ret \
    <<" Check adi_imu_Error_e.");
  ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Getting status of Board successful.");
  b_success=(ret>=0)? true : false;

  if(b_success)
  {
    uint16_t buf_status_decimal = (buf_status.bufWaterMark << BITP_ISENSOR_STATUS_BUF_WTRMRK) |
      (buf_status.bufFull << BITP_ISENSOR_STATUS_BUF_FULL) | \
      (buf_status.spiError << BITP_ISENSOR_STATUS_SPI_ERROR) | \
      (buf_status.spiOverflow << BITP_ISENSOR_STATUS_SPI_OVRFLW) | \
      (buf_status.overrun << BITP_ISENSOR_STATUS_OVERRUN) | \
      (buf_status.dmaError << BITP_ISENSOR_STATUS_DMA_ERROR) | \
      (buf_status.ppsUnlock << BITP_ISENSOR_STATUS_PPS_UNLOCK) | \
      (buf_status.tempWarning << BITP_ISENSOR_STATUS_TEMP_WARNING) | \
      (buf_status.scriptError << BITP_ISENSOR_STATUS_SCRIPT_ERROR) | \
      (buf_status.scriptActive << BITP_ISENSOR_STATUS_SCRIPT_ACTIVE) | \
      (buf_status.flashError << BITP_ISENSOR_STATUS_FLASH_ERROR) | \
      (buf_status.flashUpdateError << BITP_ISENSOR_STATUS_FLASH_UPD_ERROR) | \
      (buf_status.fault << BITP_ISENSOR_STATUS_FAULT) | \
      (buf_status.watchdog << BITP_ISENSOR_STATUS_WATCHDOG);

    *p_status = buf_status_decimal;

    ROS_INFO("IMU BUF System Status: 0x%04X", buf_status_decimal);
    *p_description=this->getStatusDescription(*p_status);

    ROS_DEBUG_STREAM("\nBuffer status: \n" <<
      "  BUF_WATERMARK: " << static_cast<int>(buf_status.bufWaterMark) << "\n" <<
      "  BUF_FULL: " << static_cast<int>(buf_status.bufFull) << "\n" <<
      "  SPI_ERROR: " << static_cast<int>(buf_status.spiError) << "\n" <<
      "  SPI_OVERFLOW: " << static_cast<int>(buf_status.spiOverflow) << "\n" <<
      "  OVERRUN: " << static_cast<int>(buf_status.overrun) << "\n" <<
      "  DMA_ERROR: " << static_cast<int>(buf_status.dmaError) << "\n" <<
      "  PPS_UNLOCK: " << static_cast<int>(buf_status.ppsUnlock) << "\n" <<
      "  TEMP_WARNING: " << static_cast<int>(buf_status.tempWarning) << "\n" <<
      "  SCRIPT_ERROR: " << static_cast<int>(buf_status.scriptError) << "\n" <<
      "  SCRIPT_ACTIVE: " << static_cast<int>(buf_status.scriptActive) << "\n" <<
      "  FLASH_ERROR: " << static_cast<int>(buf_status.flashError) << "\n" <<
      "  FLASH_UPDATE_ERROR: " << static_cast<int>(buf_status.flashUpdateError) << "\n" <<
      "  FAULT: " << static_cast<int>(buf_status.fault)<< "\n" <<
      "  WATCHDOG: " << static_cast<int>(buf_status.watchdog) << "\n");

    ROS_INFO("================================");
  }
  return b_success;
}

std::string ImuBuf::getStatusDescription(uint16_t status)
{
  std::string message="";

  if(BITM_ISENSOR_STATUS_BUF_WTRMRK & status)
  {
    message.append("  [INFO] Buffer watermark is set.");
    ROS_INFO_STREAM("  Buffer watermark is set.");
  }
  if(BITM_ISENSOR_STATUS_BUF_FULL & status)
  {
    message.append("  [INFO] Buffer is FULL.");
    ROS_INFO_STREAM("  Buffer is FULL.");
  }
  if(BITM_ISENSOR_STATUS_SPI_ERROR & status)
  {
    message.append("  [ERROR] SPI Error.");
    ROS_ERROR_STREAM("  SPI Error.");
  }
  if(BITM_ISENSOR_STATUS_SPI_OVRFLW & status)
  {
    message.append("  [ERROR] SPI overflow error.");
    ROS_ERROR_STREAM("  SPI overflow error.");
  }
  if(BITM_ISENSOR_STATUS_OVERRUN & status)
  {
    message.append("  [ERROR] Data capture overrun error.");
    ROS_ERROR_STREAM("  Data capture overrun error.");
  }
  if(BITM_ISENSOR_STATUS_DMA_ERROR & status)
  {
    message.append("  [ERROR] DMA error.");
    ROS_ERROR_STREAM("  DMA error.");
  }
  if(BITM_ISENSOR_STATUS_PPS_UNLOCK & status)
  {
    message.append("  [INFO] PPS is UNLOCKED.");
    ROS_INFO_STREAM("  PPS is UNLOCKED.");
  }
  if(BITM_ISENSOR_STATUS_TEMP_WARNING & status)
  {
    message.append("  [WARN] Temperature outside safe range [-40C to 85C].");
    ROS_WARN_STREAM("  Temperature outside safe range [-40C to 85C].");
  }
  if(BITM_ISENSOR_STATUS_SCRIPT_ERROR & status)
  {
    message.append("  [ERROR] Script launch error.");
    ROS_ERROR_STREAM("  Script launch error.");
  }
  if(BITM_ISENSOR_STATUS_SCRIPT_ACTIVE & status)
  {
    message.append("  [INFO] Script is ACTIVE.");
    ROS_INFO_STREAM("  Script is ACTIVE.");
  }
  if(BITM_ISENSOR_STATUS_FLASH_ERROR & status)
  {
    message.append("  [ERROR] FLASH verify failed.");
    ROS_ERROR_STREAM("  FLASH verify failed.");
  }
  if(BITM_ISENSOR_STATUS_FLASH_UPD_ERROR & status)
  {
    message.append("  [ERROR] FLASH update failed.");
    ROS_ERROR_STREAM("  FLASH update failed.");
  }
  if(BITM_ISENSOR_STATUS_FAULT & status)
  {
    message.append("  [ERROR] Processor fault occured.");
    ROS_ERROR_STREAM("  Processor fault occured.");
  }
  if(BITM_ISENSOR_STATUS_WATCHDOG & status)
  {
    message.append("  [ERROR] Processor reset due to watchdog.");
    ROS_ERROR_STREAM("  Processor reset due to watchdog.");
  }
  return message;
}

bool ImuBuf::clearFault(void)
{
  bool b_success=false;
  int ret = -1;

  ret=imubuf_ClearFault(p_device_);
  ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Clear Fault failed. ERROR CODE: "<< ret \
    <<" Check adi_imu_Error_e.");
  ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Clear Fault successful.");
  b_success=(ret>=0)? true : false;
  // Add delay
  ros::Duration(2.0).sleep();
  return b_success;
}

bool ImuBuf::flashUpdate(void)
{
  bool b_success=false;
  int ret = -1;

  ret=imubuf_FlashUpdate(p_device_);
  ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Flash Update failed. ERROR CODE: "<< ret \
    <<" Check adi_imu_Error_e.");
  ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Flash Update successful.");
  b_success=(ret>=0)? true : false;
  // Add delay
  ros::Duration(2.0).sleep();
  return b_success;
}

bool ImuBuf::factoryReset(void)
{
  bool b_success=false;
  int ret = -1;

  ret=imubuf_FactoryReset(p_device_);
  ROS_ERROR_STREAM_COND(ret<0,"[ADRD2121] Factory reset failed. ERROR CODE: "<< ret \
    <<" Check adi_imu_Error_e.");
  ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Factory reset successful.");
  b_success=(ret>=0)? true : false;

  return b_success;
}

bool ImuBuf::getStatusCallback(adrd2121_imu::BufStatus::Request &req, \
  adrd2121_imu::BufStatus::Response &res)
{
  ROS_INFO_STREAM("[ADRD2121] /get_buffer_status Service called.");

  bool b_success = false;
  uint16_t status=0xFFFF;
  std::string message="";

  if(STREAMING == mode_of_operation_)
  {
    // Stop data capture first
    ROS_INFO_STREAM("Stop ADRD2121 Data Capture. Will pause publishing IMU data.");
    b_success = this->stopBufferRead();
  }
  else if(RECOVERY == mode_of_operation_)
  {
    b_success=true; // Else, no need to stop capture.
  }

  if(b_success)
  {
    b_success=this->getStatus(&status, &message);

    if(b_success)
    {
      res.success=true;
      res.status=status;

      res.message=message;
    }
    else
    {
      res.success=false;
      res.status=status;
      res.status_hex="";
      res.message="Failed to get status.";
    }

    if(STREAMING == mode_of_operation_)
    {
      // Start data capture first
      ROS_INFO_STREAM("Start ADRD2121 Data Capture. Will resume publishing IMU data.");
      b_success = this->startBufferRead();
    }
    else if(RECOVERY == mode_of_operation_)
    {
      b_success=true; // Else, no need to start stream
    }

    if(!b_success)
    {
      ROS_ERROR_STREAM("[get_buffer_status service] Failed to restart ADRD2121 Data Capture. No data\
        will be published. Need to re-launch node.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[get_buffer_status service] Failed to stop ADRD2121 Data Capture. Try again.");
  }

  return true;
}

bool ImuBuf::clearFaultCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO_STREAM("[ADRD2121] /clear_fault Service called.");
  bool b_success = false;

  b_success=this->clearFault();
  if(b_success)
  {
    res.success=true;
    res.message="Successfully triggered /clear_fault";
  }
  else
  {
    res.success=false;
    res.message="Failed to trigger /clear_fault";
  }
  return true;
}

bool ImuBuf::flashUpdateCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO_STREAM("[ADRD2121] /flash_update Service called.");
  bool b_success = false;

  b_success=this->flashUpdate();
  if(b_success)
  {
    res.success=true;
    res.message="Successfully triggered /flash_update";
  }
  else
  {
    res.success=false;
    res.message="Failed to trigger /flash_update";
  }
  return true;
}

bool ImuBuf::factoryResetCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO_STREAM("[ADRD2121] /factory_reset Service called.");
  bool b_success = false;

  b_success=this->factoryReset();
  if(b_success)
  {
    res.success=true;
    res.message="Successfully triggered /factory_reset";
  }
  else
  {
    res.success=false;
    res.message="Failed to trigger /factory_reset";
  }
  return true;
}

void ImuBuf::initServiceServers(ros::NodeHandle* p_nh_local, mode_of_operation_e mode_of_operation)
{
  ROS_INFO_STREAM("[ADRD2121] Initializing service servers.");
  if(STREAMING == mode_of_operation)
  {
    // Do nothing
  }
  else if(RECOVERY == mode_of_operation)
  {
    factory_reset_service_=p_nh_local->advertiseService("factory_reset", &ImuBuf::factoryResetCallback, this);

    clear_fault_service_=p_nh_local->advertiseService("clear_fault", &ImuBuf::clearFaultCallback, this);

    flash_update_service_=p_nh_local->advertiseService("flash_update", &ImuBuf::flashUpdateCallback, this);
  }

  get_buffer_status_service_=p_nh_local->advertiseService("get_buffer_status", &ImuBuf::getStatusCallback, this);
}

bool ImuBuf::recoverBoard(void)
{
  bool b_success=false;
  uint16_t status=0xFFFF;
  int try_count = 3; // Fixed to 3 for now
  bool b_has_error = true;
  std::string status_description="";
  bool b_buffer_detected = false;

  ROS_INFO_STREAM("[ADRD2121] Recover Board if any error is found.");
  while((try_count >=0) && (b_has_error))
  {
    // Check if ADRD2121 is detected
    b_buffer_detected=this->detect();

    ROS_INFO_STREAM("[ADRD2121] Check board status.");
    b_success=this->getStatus(&status, &status_description);
    if(b_success)
    {
      uint16_t status_error_mask = BITM_ISENSOR_STATUS_SPI_ERROR | BITM_ISENSOR_STATUS_SPI_OVRFLW | \
      BITM_ISENSOR_STATUS_OVERRUN | BITM_ISENSOR_STATUS_DMA_ERROR | BITM_ISENSOR_STATUS_SCRIPT_ERROR | \
      BITM_ISENSOR_STATUS_FLASH_ERROR | BITM_ISENSOR_STATUS_FLASH_UPD_ERROR | BITM_ISENSOR_STATUS_FAULT | \
      BITM_ISENSOR_STATUS_WATCHDOG;

      if((status_error_mask & status) || (!b_buffer_detected))
      {
        ROS_WARN_STREAM("[ADRD2121] Error found. Will try to recover board.");
        b_success=this->factoryReset();
        if(b_success)
        {
          b_success=this->clearFault();
        }
        if(b_success)
        {
          b_success=this->flashUpdate();
        }
      }
      else
      {
        ROS_INFO_STREAM_COND(try_count==3,"[ADRD2121] No errors found. Will not execute recovery sequence.");
        ROS_INFO_STREAM_COND(try_count<3,"[ADRD2121] No errors found.");
        b_has_error=false;
      }
    }
    try_count--;
  }

  if((try_count == -1) && (b_has_error))
  {
    ROS_WARN_STREAM("[ADRD2121] Failed to recover board.");
    b_success=false;
  }
  else if(!b_success)
  {
    ROS_INFO_STREAM("[ADRD2121] Successfully recovered board.");
    b_success=true;
  }
  return b_success;
}

void ImuBuf::setModeOfOperation(mode_of_operation_e mode_of_operation)
{
  mode_of_operation_ = mode_of_operation;
}

bool ImuBuf::detect(void)
{
  int ret=-1;
  bool b_success=false;
  ROS_INFO_STREAM("[ADRD2121] Detect if ADRD2121 is connected.");
#if defined(ADRD2121_IMU_WITH_HW)
  //Detect if ADRD2121 is connected
  ret=imubuf_Detect(p_device_);
#else
  ret=0;
#endif
  ROS_WARN_STREAM_COND(ret<0,"[ADRD2121] Board NOT detected. ERROR CODE: "<< ret \
    <<" Check adi_imu_Error_e.");
  ROS_INFO_STREAM_COND(ret>=0,"[ADRD2121] Board is connected to Host");
  b_success=(ret>=0)? true : false;

  return b_success;
}

/* Helper function/s */
const char * const BUFDIOtoString(int dio)
{
  const char *output= new char[20];

  switch(dio)
  {
    case 0:
      output="Disabled/None";
      break;
    case 1:
      output="DIO1";
      break;
    case 2:
      output="DIO2";
      break;
    case 4:
      output="DIO3";
      break;
    case 8:
      output="DIO4";
      break;
  }
  return output;
}
