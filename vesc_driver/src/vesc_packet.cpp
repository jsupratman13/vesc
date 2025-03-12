/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Softbank Corp. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/** NOTE *************************************************************
 * This program had been developed by Michael T. Boulet at MIT under
 * the BSD 3-clause License until Dec. 2016. Since Nov. 2019, Softbank
 * Corp. takes over development as new packages.
 ********************************************************************/

#include "vesc_driver/vesc_packet.hpp"
#include "vesc_driver/data_map.hpp"

namespace vesc_driver
{
/**
 * @brief Constructor
 * @param payload_size Specified payload size
 **/
VescFrame::VescFrame(const int16_t payload_size)
{
  assert(payload_size >= 0 && payload_size <= 1024);

  if (payload_size < 256)
  {
    // single byte payload size
    frame_.resize(VESC_MIN_FRAME_SIZE + payload_size);
    *(frame_.begin()) = 2;
    *(frame_.begin() + 1) = payload_size;
    payload_end_.first = frame_.begin() + 2;
  }
  else
  {
    // two byte payload size
    frame_.resize(VESC_MIN_FRAME_SIZE + 1 + payload_size);
    *(frame_.begin()) = 3;
    *(frame_.begin() + 1) = payload_size >> 8;
    *(frame_.begin() + 2) = payload_size & 0xFF;
    payload_end_.first = frame_.begin() + 3;
  }

  payload_end_.second = payload_end_.first + payload_size;
  *(frame_.end() - 1) = 3;
}

/**
 * @brief Constructor
 * @param frame Reference of a buffer with constant range
 * @param payload_size Specified payload size
 **/
VescFrame::VescFrame(const BufferRangeConst& frame, const BufferRangeConst& payload)
{
  /* VescPacketFactory::createPacket() should make sure that
   *  the input is valid, but run a few cheap checks anyway */
  assert(boost::distance(frame) >= VESC_MIN_FRAME_SIZE);
  assert(boost::distance(frame) <= VESC_MAX_FRAME_SIZE);
  assert(boost::distance(payload) <= VESC_MAX_PAYLOAD_SIZE);
  assert(std::distance(frame.first, payload.first) > 0 && std::distance(payload.second, frame.second) > 0);

  frame_.resize(std::distance(boost::begin(frame), boost::end(frame)));
  frame_.assign(boost::begin(frame), boost::end(frame));
  payload_end_.first = frame_.begin() + std::distance(frame.first, payload.first);
  payload_end_.second = frame_.begin() + std::distance(frame.first, payload.second);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 * @param name Packet name
 * @param payload_size Specified payload size
 * @param payload_id ID of payload
 **/
VescPacket::VescPacket(const std::string& name, const int16_t payload_size, const int16_t payload_id)
  : VescFrame(payload_size), name_(name)
{
  assert(payload_id >= 0 && payload_id < 256);
  assert(boost::distance(payload_end_) > 0);
  *payload_end_.first = payload_id;
}

/**
 * @brief Constructor
 * @param name Packet name
 * @param raw Pointer of a frame
 **/
VescPacket::VescPacket(const std::string& name, std::shared_ptr<VescFrame> raw) : VescFrame(*raw), name_(name)
{
  uint16_t original_payload_size = std::distance(payload_end_.first, payload_end_.second);
  payload_end_.first = frame_.begin() + 2;
  payload_end_.second = std::min(payload_end_.first + original_payload_size, frame_.end());
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 * @param raw Pointer of VescFrame
 **/
VescPacketFWVersion::VescPacketFWVersion(std::shared_ptr<VescFrame> raw) : VescPacket("FWVersion", raw)
{
}

/**
 * @brief Gets major farmware version
 * @return Major farmware version
 **/
int16_t VescPacketFWVersion::fwMajor() const
{
  return *(payload_end_.first + 1);
}

/**
 * @brief Gets minor farmware version
 * @return Minor farmware version
 **/
int16_t VescPacketFWVersion::fwMinor() const
{
  return *(payload_end_.first + 2);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketRequestFWVersion::VescPacketRequestFWVersion() : VescPacket("RequestFWVersion", 1, COMM_FW_VERSION)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketValues::VescPacketValues(std::shared_ptr<VescFrame> raw) : VescPacket("Values", raw)
{
}

/**
 * @brief Gets temperature of MOSFETs
 * @return Temperature of MOSFETs
 **/
double VescPacketValues::getMosTemp() const
{
  return readBuffer(TEMP_MOS, 2) / 10.0;
}

/**
 * @brief Gets temperature of the motor
 * @return Temperature of the motor
 **/
double VescPacketValues::getMotorTemp() const
{
  return readBuffer(TEMP_MOTOR, 2) / 10.0;
}

/**
 * @brief Gets motor current
 * @return Motor current
 **/
double VescPacketValues::getMotorCurrent() const
{
  return readBuffer(CURRENT_MOTOR, 4) / 100.0;
}

/**
 * @brief Gets input current
 * @return Input current
 **/
double VescPacketValues::getInputCurrent() const
{
  return readBuffer(CURRENT_IN, 4) / 100.0;
}

/**
 * @brief Gets the current duty value
 * @return The current duty value
 **/
double VescPacketValues::getDuty() const
{
  int16_t duty_raw = static_cast<int32_t>(readBuffer(DUTY_NOW, 2));

  // inverts to derive a negative value
  if (duty_raw > 1000)
  {
    duty_raw = !duty_raw;
  }

  return static_cast<double>(duty_raw) / 1000.0;
}

/**
 * @brief Gets the current angular velocity
 * @return The current angular velocity
 **/
double VescPacketValues::getVelocityERPM() const
{
  return readBuffer(ERPM, 4);
}

/**
 * @brief Gets input voltage
 * @return Input voltage
 **/
double VescPacketValues::getInputVoltage() const
{
  return readBuffer(VOLTAGE_IN, 2) / 10.0;
}

/**
 * @brief Gets consumed charge
 * @return Consumed charge
 **/
double VescPacketValues::getConsumedCharge() const
{
  return readBuffer(AMP_HOURS, 4) / 10000.0;
}

/**
 * @brief Gets input charge
 * @return Input charge
 **/
double VescPacketValues::getInputCharge() const
{
  return readBuffer(AMP_HOURS_CHARGED, 4) / 10000.0;
}

/**
 * @brief Gets consumed power
 * @return Consumed power
 **/
double VescPacketValues::getConsumedPower() const
{
  return readBuffer(WATT_HOURS, 4) / 10000.0;
}

/**
 * @brief Gets input power
 * @return Input power
 **/
double VescPacketValues::getInputPower() const
{
  return readBuffer(WATT_HOURS_CHARGED, 4) / 10000.0;
}

/**
 * @brief Gets the current tachometer value
 * @return The current tachometer value
 **/
double VescPacketValues::getTachometer() const
{
  return readBuffer(TACHOMETER, 4);
}

/**
 * @brief Gets absolute displacement in tachometer
 * @return Absolute displacement in tachometer
 **/
double VescPacketValues::getDisplacement() const
{
  return readBuffer(TACHOMETER_ABS, 4);
}

/**
 * @brief Gets fault code
 * @return Fault code
 **/
int VescPacketValues::getFaultCode() const
{
  return static_cast<int32_t>(*(payload_end_.first + FAULT_CODE));
}

/**
 * @brief Gets the position in deg.
 * @return The current position between 0 to 360 deg.
 **/
 double VescPacketValues::getPosition() const
 {
   return readBuffer(PID_POS, 4) / 1000000.0;
 }

 /**
 * @brief Gets controller id
 * @return Fault code
 **/
int VescPacketValues::getControllerID() const
{
  return static_cast<int32_t>(*(payload_end_.first + CONTROLLER_ID));
}

/**
 * @brief Reads a value from the buffer
 * @param map_id start address to read
 * @param size the number of bytes to read
 * @return Required value
 **/
double VescPacketValues::readBuffer(const uint8_t map_id, const uint8_t size) const
{
  int32_t value = 0;
  switch (size)
  {
    case 2:
      value += static_cast<int32_t>(*(payload_end_.first + map_id) << 8);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 1));
      break;
    case 4:
      value += static_cast<int32_t>(*(payload_end_.first + map_id) << 24);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 1) << 16);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 2) << 8);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 3));
      break;
  }

  return static_cast<double>(value);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketRequestValues::VescPacketRequestValues() : VescPacket("RequestValues", 1, COMM_GET_VALUES)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
 VescPacketValuesSetup::VescPacketValuesSetup(std::shared_ptr<VescFrame> raw) : VescPacket("ValuesSetup", raw)
{
}

/**
 * @brief Gets temperature of MOSFETs
 * @return Temperature of MOSFETs
 **/
double VescPacketValuesSetup::getMosTemp() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::TEMP_MOS), 2) / 10.0;
}

/**
 * @brief Gets temperature of the motor
 * @return Temperature of the motor
 **/
double VescPacketValuesSetup::getMotorTemp() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::TEMP_MOTOR), 2) / 10.0;
}
 
/**
 * @brief Gets total motor current
 * @return Total motor current
 **/
double VescPacketValuesSetup::getTotalMotorCurrent() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::TOTAL_CURRENT_MOTOR), 4) / 100.0;
}
 
/**
 * @brief Gets total input current
 * @return Total input current
 **/
double VescPacketValuesSetup::getTotalInputCurrent() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::TOTAL_CURRENT_IN), 4) / 100.0;
}
 
/**
 * @brief Gets the current duty value
 * @return The current duty value
 **/
double VescPacketValuesSetup::getDuty() const
{
  int16_t duty_raw = static_cast<int32_t>(readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::DUTY_NOW), 2));

  // inverts to derive a negative value
  if (duty_raw > 1000)
  {
    duty_raw = !duty_raw;
  }

  return static_cast<double>(duty_raw) / 1000.0;
}
 
/**
 * @brief Gets the current erpm
 * @return The current erpm
 **/
double VescPacketValuesSetup::getVelocityERPM() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::ERPM), 4);
}

/**
 * @brief Gets the current velocity based on wheel diameter, gearing and motor poles
 * @return The current velocity
 **/
double VescPacketValuesSetup::getVelocity() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::SPEED), 4);
}

/**
 * @brief Gets input voltage
 * @return Input voltage
 **/
double VescPacketValuesSetup::getInputVoltage() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::VOLTAGE_IN), 2) / 10.0;
}

/**
 * @brief Get battery level
 * @brief Battery level
 **/
double VescPacketValuesSetup::getBatteryLevel() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::BATTERY_LEVEL), 2);
}

/**
 * @brief Gets total consumed charge
 * @return Total consumed charge
 **/
double VescPacketValuesSetup::getTotalConsumedCharge() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::TOTAL_AMP_HOURS), 4) / 10000.0;
}
 
/**
 * @brief Gets total input charge
 * @return Total input charge
 **/
double VescPacketValuesSetup::getTotalInputCharge() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::TOTAL_AMP_HOURS_CHARGED), 4) / 10000.0;
}

/**
 * @brief Gets total consumed power
 * @return Total consumed power
 **/
double VescPacketValuesSetup::getTotalConsumedPower() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::TOTAL_WATT_HOURS), 4) / 10000.0;
}

/**
 * @brief Gets total input power
 * @return Total input power
 **/
double VescPacketValuesSetup::getTotalInputPower() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::TOTAL_WATT_HOURS_CHARGED), 4) / 10000.0;
}

/**
 * @brief Gets the distance traveled based on wheel diameter, gearing and motor poles
 * @return The distance traveled
 **/
double VescPacketValuesSetup::getDistance() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::DISTANCE), 4);
}

/**
 * @brief Gets the distance traveled based on wheel diameter, gearing and motor poles
 * @return The absolute distance traveled
 **/
double VescPacketValuesSetup::getAbsDistance() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::ABSOLUTE_DISTANCE), 4);
}

/**
 * @brief Gets the position in deg.
 * @return The current position between 0 to 360 deg.
 **/
double VescPacketValuesSetup::getPosition() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::PID_POS), 4) / 1000000.0;
}

/**
 * @brief Gets fault code
 * @return Fault code
 **/
int VescPacketValuesSetup::getFaultCode() const
{
  return static_cast<int32_t>(*(payload_end_.first + static_cast<uint8_t>(PACKET_VALUES_SETUP::FAULT_CODE)));
}

/**
 * @brief Gets controller id
 * @return Fault code
 **/
int VescPacketValuesSetup::getControllerID() const
{
  return static_cast<int32_t>(*(payload_end_.first + static_cast<uint8_t>(PACKET_VALUES_SETUP::CONTROLLER_ID)));
}

/**
 * @brief Gets controller id
 * @return Fault code
 **/
int VescPacketValuesSetup::getNumVescs() const
{
  return static_cast<int32_t>(*(payload_end_.first + static_cast<uint8_t>(PACKET_VALUES_SETUP::NUM_VESCS)));
}

/**
 * @brief Get battery level
 * @brief Battery level
 **/
double VescPacketValuesSetup::getBatteryRemaining() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::BATTERY_REMAINING), 4);
}

/**
 * @brief Gets current odometer value in meters
 * @return The odometer value in meters
 **/
double VescPacketValuesSetup::getOdometer() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::ODOMETER), 4);
}

/**
 * @brief Gets the current system times in ticks
 * @return The system time in ticks
 **/
double VescPacketValuesSetup::getSystemTime() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_VALUES_SETUP::SYSTEM_TIME), 4);
}

/**
 * @brief Reads a value from the buffer
 * @param map_id start address to read
 * @param size the number of bytes to read
 * @return Required value
 **/
double VescPacketValuesSetup::readBuffer(const uint8_t map_id, const uint8_t size) const
{
  int32_t value = 0;
  switch (size)
  {
    case 2:
      value += static_cast<int32_t>(*(payload_end_.first + map_id) << 8);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 1));
      break;
    case 4:
      value += static_cast<int32_t>(*(payload_end_.first + map_id) << 24);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 1) << 16);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 2) << 8);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 3));
      break;
  }

  return static_cast<double>(value);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketRequestValuesSetup::VescPacketRequestValuesSetup() : VescPacket("RequestValuesSetup", 1, COMM_GET_VALUES_SETUP)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketMCConf::VescPacketMCConf(std::shared_ptr<VescFrame> raw) : VescPacket("MCConf", raw)
{
  int map_id = 1;
  auto signature = readBuffer(map_id, 4); map_id += 4;
  config_.pwm_mode = static_cast<PWM_MODE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.comm_mode = static_cast<COMM_MODE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.sensor_mode = static_cast<SENSOR_MODE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.motor_type = static_cast<MOTOR_TYPE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.l_current_max = readBuffer(map_id, 4); map_id += 4;
  config_.l_current_min = readBuffer(map_id, 4); map_id += 4;
  config_.l_in_current_max = readBuffer(map_id, 4); map_id += 4;
  config_.l_in_current_min = readBuffer(map_id, 4); map_id += 4;
  config_.l_abs_current_max = readBuffer(map_id, 4); map_id += 4;
  config_.l_min_erpm = readBuffer(map_id, 4); map_id += 4;
  config_.l_max_erpm = readBuffer(map_id, 4); map_id += 4;
  config_.l_erpm_start = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_max_erpm_fbrake = readBuffer(map_id, 4); map_id += 4;
  config_.l_max_erpm_fbrake_cc = readBuffer(map_id, 4); map_id += 4;
  config_.l_min_vin = readBuffer(map_id, 4); map_id += 4;
  config_.l_max_vin = readBuffer(map_id, 4); map_id += 4;
  config_.l_battery_cut_start = readBuffer(map_id, 4); map_id += 4;
  config_.l_battery_cut_end = readBuffer(map_id, 4); map_id += 4;
  config_.l_slow_abs_current = static_cast<bool>(*(payload_end_.first + map_id)); map_id += 1;
  config_.l_temp_fet_start = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.l_temp_fet_end = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.l_temp_motor_start = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.l_temp_motor_end = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.l_temp_accel_dec = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_min_duty = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_max_duty = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_watt_max = readBuffer(map_id, 4); map_id += 4;
  config_.l_watt_min = readBuffer(map_id, 4); map_id += 4;
  config_.l_current_max_scale = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_current_min_scale = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_duty_start = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.sl_min_erpm = readBuffer(map_id, 4); map_id += 4;
  config_.sl_min_erpm_cycle_int_limit = readBuffer(map_id, 4); map_id += 4;
  config_.sl_max_fullbreak_current_dir_change = readBuffer(map_id, 4); map_id += 4;
  config_.sl_cycle_int_limit = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.sl_phase_advance_at_br = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.sl_cycle_int_rpm_br = readBuffer(map_id, 4); map_id += 4;
  config_.sl_bemf_coupling_k = readBuffer(map_id, 4); map_id += 4;
  config_.hall_table[0] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.hall_table[1] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.hall_table[2] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.hall_table[3] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.hall_table[4] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.hall_table[5] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.hall_table[6] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.hall_table[7] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.hall_sl_erpm = readBuffer(map_id, 4); map_id += 4;
  config_.foc_current_kp = readBuffer(map_id, 4); map_id += 4;
  config_.foc_current_ki = readBuffer(map_id, 4); map_id += 4;
  config_.foc_f_zv = readBuffer(map_id, 4); map_id += 4;
  config_.foc_dt_us = readBuffer(map_id, 4); map_id += 4;
  config_.foc_encoder_inverted = static_cast<bool>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_encoder_offset = readBuffer(map_id, 4); map_id += 4;
  config_.foc_encoder_ratio = readBuffer(map_id, 4); map_id += 4;
  config_.foc_encoder_sin_gain = readBuffer(map_id, 4); map_id += 4;
  config_.foc_encoder_cos_gain = readBuffer(map_id, 4); map_id += 4;
  config_.foc_encoder_sin_offset = readBuffer(map_id, 4); map_id += 4;
  config_.foc_encoder_cos_offset = readBuffer(map_id, 4); map_id += 4;
  config_.foc_encoder_sincos_filter_constant = readBuffer(map_id, 4); map_id += 4;
  config_.foc_sensor_mode = static_cast<FOC_SENSOR_MODE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_pll_kp = readBuffer(map_id, 4); map_id += 4;
  config_.foc_pll_ki = readBuffer(map_id, 4); map_id += 4;
  config_.foc_motor_l = readBuffer(map_id, 4); map_id += 4;
  config_.foc_motor_ld_lq_diff = readBuffer(map_id, 4); map_id += 4;
  config_.foc_motor_r = readBuffer(map_id, 4); map_id += 4;
  config_.foc_motor_flux_linkage = readBuffer(map_id, 4); map_id += 4;
  config_.foc_observer_gain = readBuffer(map_id, 4); map_id += 4;
  config_.foc_observer_gain_slow = readBuffer(map_id, 4); map_id += 4;
  config_.foc_observer_offset = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.foc_duty_dowmramp_kp = readBuffer(map_id, 4); map_id += 4;
  config_.foc_duty_dowmramp_ki = readBuffer(map_id, 4); map_id += 4;
  config_.foc_openloop_rpm = readBuffer(map_id, 4); map_id += 4;
  config_.foc_openloop_rpm_low = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.foc_d_gain_scale_start = readBuffer(map_id, 4); map_id += 4;
  config_.foc_d_gain_scale_max_mod = readBuffer(map_id, 4); map_id += 4;
  config_.foc_sl_openloop_hyst = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.foc_sl_openloop_time_lock = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.foc_sl_openloop_time_ramp = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.foc_sl_openloop_time = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.foc_hall_table[0] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_hall_table[1] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_hall_table[2] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_hall_table[3] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_hall_table[4] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_hall_table[5] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_hall_table[6] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_hall_table[7] = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_hall_interp_erpm = readBuffer(map_id, 4); map_id += 4;
  config_.foc_sl_erpm = readBuffer(map_id, 4); map_id += 4;
  config_.foc_sample_v0_v7 = static_cast<bool>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_sample_high_current = static_cast<bool>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_sat_comp = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.foc_temp_comp = static_cast<bool>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_temp_comp_base_temp = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.foc_current_filter_const = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_cc_decoupling = static_cast<FOC_CC_DECOUPLING_MODE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_observer_type = static_cast<FOC_OBSERVER_TYPE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_hfi_voltage_start = readBuffer(map_id, 4); map_id += 4;
  config_.foc_hfi_voltage_run = readBuffer(map_id, 4); map_id += 4;
  config_.foc_hfi_voltage_max = readBuffer(map_id, 4); map_id += 4;
  config_.foc_sl_erpm_hfi = readBuffer(map_id, 4); map_id += 4;
  config_.foc_hfi_start_samples = readBuffer(map_id, 2); map_id += 2;
  config_.foc_hfi_obs_ovr_sec = readBuffer(map_id, 4); map_id += 4;
  config_.foc_hfi_samples = static_cast<FOC_HFI_SAMPLES>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_offsets_cal_on_boot = static_cast<bool>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_offsets_current[0] = readBuffer(map_id, 4); map_id += 4;
  config_.foc_offsets_current[1] = readBuffer(map_id, 4); map_id += 4;
  config_.foc_offsets_current[2] = readBuffer(map_id, 4); map_id += 4;
  config_.foc_offsets_voltage[0] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_offsets_voltage[1] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_offsets_voltage[2] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_offsets_voltage_undriven[0] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_offsets_voltage_undriven[1] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_offsets_voltage_undriven[2] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_phase_filter_enable = static_cast<bool>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_phase_filter_max_erpm = readBuffer(map_id, 4); map_id += 4;
  config_.foc_mtpa_mode = static_cast<MTPA_MODE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.foc_fw_current_max = readBuffer(map_id, 4); map_id += 4;
  config_.foc_fw_duty_start = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_fw_ramp_time  = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.foc_fw_q_current_factor = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.gpd_buffer_notify_left = readBuffer(map_id, 2); map_id += 2;
  config_.gpd_buffer_interpol    = readBuffer(map_id, 2); map_id += 2;
  config_.gpd_current_filter_const = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.gpd_current_kp = readBuffer(map_id, 4); map_id += 4;
  config_.gpd_current_ki = readBuffer(map_id, 4); map_id += 4;
  config_.sp_pid_loop_rate = static_cast<PID_RATE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.s_pid_kp = readBuffer(map_id, 4); map_id += 4;
  config_.s_pid_ki = readBuffer(map_id, 4); map_id += 4;
  config_.s_pid_kd = readBuffer(map_id, 4); map_id += 4;
  config_.s_pid_kd_filter = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.s_pid_min_erpm = readBuffer(map_id, 4); map_id += 4;
  config_.s_pid_allow_braking = static_cast<bool>(*(payload_end_.first + map_id)); map_id += 1;
  config_.s_pid_ramp_erpms_s = readBuffer(map_id, 4); map_id += 4;
  config_.p_pid_kp = readBuffer(map_id, 4); map_id += 4;
  config_.p_pid_ki = readBuffer(map_id, 4); map_id += 4;
  config_.p_pid_kd = readBuffer(map_id, 4); map_id += 4;
  config_.p_pid_kd_proc = readBuffer(map_id, 4); map_id += 4;
  config_.p_pid_kd_filter = readBuffer(map_id, 4); map_id += 4;
  config_.p_pid_ang_div = readBuffer(map_id, 4); map_id += 4;
  config_.p_pid_gain_dec_angle = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.p_pid_offset = readBuffer(map_id, 4); map_id += 4;
  config_.cc_startup_boost_duty = readBuffer(map_id, 4); map_id += 4;
  config_.cc_min_current = readBuffer(map_id, 4); map_id += 4;
  config_.cc_gain = readBuffer(map_id, 4); map_id += 4;
  config_.cc_ramp_step_max = readBuffer(map_id, 4); map_id += 4;
  config_.m_fault_stop_time_ms = readBuffer(map_id, 4); map_id += 4;
  config_.m_duty_ramp_step = readBuffer(map_id, 4); map_id += 4;
  config_.m_current_backoff_gain = readBuffer(map_id, 4); map_id += 4;
  config_.m_encoder_counts = readBuffer(map_id, 4); map_id += 4;
  config_.m_sensor_port_mode = static_cast<SENSOR_PORT_MODE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.m_invert_direction = static_cast<bool>(*(payload_end_.first + map_id)); map_id += 1;
  config_.m_drv8301_oc_mode = static_cast<DRV8301_OC_MODE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.m_drv8301_oc_adj = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.m_bldc_f_sw_min = readBuffer(map_id, 4);
  config_.m_bldc_f_sw_max = readBuffer(map_id, 4);
  config_.m_dc_f_sw = readBuffer(map_id, 4);
  config_.m_ntc_motor_beta = readBuffer(map_id, 4);
  config_.m_out_aux_mode = static_cast<OUT_AUX_MODE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.m_motor_temp_sens_type = static_cast<TEMP_SENSOR_TYPE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.m_ptc_motor_coeff = readBuffer(map_id, 4); map_id += 4;
  config_.m_hall_extra_samples = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.si_motor_poles = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.si_gear_ratio = readBuffer(map_id, 4); map_id += 4;
  config_.si_wheel_diameter = readBuffer(map_id, 4); map_id += 4;
  config_.si_battery_type = static_cast<BATTERY_TYPE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.si_battery_cells = static_cast<int>(*(payload_end_.first + map_id)); map_id += 1;
  config_.si_battery_ah = readBuffer(map_id, 4); map_id += 4;
  config_.si_motor_nl_current = readBuffer(map_id, 4); map_id += 4;
  config_.bms.type = static_cast<BMS_TYPE>(*(payload_end_.first + map_id)); map_id += 1;
  config_.bms.t_limit_start = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.bms.t_limit_end   = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.bms.soc_limit_start = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.bms.soc_limit_end   = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.bms.fwd_can_mode = static_cast<BMS_FWD_CAN_MODE>(*(payload_end_.first + map_id)); map_id += 1;
}

/**
 * @brief Get MC configuration
 * @return The MC configuration
 **/
MCConfiguration VescPacketMCConf::getConfig() const
{
  return config_;
}

/**
 * @brief Reads a value from the buffer
 * @param map_id start address to read
 * @param size the number of bytes to read
 * @return Required value
 **/
double VescPacketMCConf::readBuffer(const int map_id, const uint8_t size) const
{
  int32_t value = 0;
  switch (size)
  {
    case 2:
      value += static_cast<int32_t>(*(payload_end_.first + map_id) << 8);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 1));
      break;
    case 4:
      value += static_cast<int32_t>(*(payload_end_.first + map_id) << 24);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 1) << 16);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 2) << 8);
      value += static_cast<int32_t>(*(payload_end_.first + map_id + 3));
      break;
  }

  return static_cast<double>(value);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketRequestMCConf::VescPacketRequestMCConf() : VescPacket("RequestMCConf", 1, COMM_GET_MCCONF)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetDuty::VescPacketSetDuty(double duty) : VescPacket("SetDuty", 5, COMM_SET_DUTY)
{
  // checks the range of duty
  if (duty > 1.0)
  {
    duty = 1.0;
  }
  else if (duty < -1.0)
  {
    duty = -1.0;
  }

  const int32_t v = static_cast<int32_t>(duty * 100000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetCurrent::VescPacketSetCurrent(double current) : VescPacket("SetCurrent", 5, COMM_SET_CURRENT)
{
  const int32_t v = static_cast<int32_t>(current * 1000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetCurrentBrake::VescPacketSetCurrentBrake(double current_brake)
  : VescPacket("SetCurrentBrake", 5, COMM_SET_CURRENT_BRAKE)
{
  const int32_t v = static_cast<int32_t>(current_brake * 1000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetVelocityERPM::VescPacketSetVelocityERPM(double vel_erpm) : VescPacket("SetERPM", 5, COMM_SET_ERPM)
{
  const int32_t v = static_cast<int32_t>(vel_erpm);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetPos::VescPacketSetPos(double pos) : VescPacket("SetPos", 5, COMM_SET_POS)
{
  /** @todo range check pos */

  const int32_t v = static_cast<int32_t>(pos * 1000000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 24) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>((v >> 16) & 0xFF);
  *(payload_end_.first + 3) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 4) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetServoPos::VescPacketSetServoPos(double servo_pos) : VescPacket("SetServoPos", 3, COMM_SET_SERVO_POS)
{
  /** @todo range check pos */

  uint16_t v = static_cast<uint16_t>(servo_pos * 1000.0);

  *(payload_end_.first + 1) = static_cast<uint8_t>((v >> 8) & 0xFF);
  *(payload_end_.first + 2) = static_cast<uint8_t>(v & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_.end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_.end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------*/
/*
VescPacketSetDetect::VescPacketSetDetect(uint8_t mode) :
  VescPacket("SetDetect", 3, COMM_SET_DETECT)
{
  *(payload_end_.first + 1) = mode;

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_end_.first), boost::distance(payload_end_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}
*/

}  // namespace vesc_driver
