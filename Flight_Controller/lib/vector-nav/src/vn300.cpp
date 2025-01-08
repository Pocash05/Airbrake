/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#else
#include "core/core.h"
#endif
#include "vn300.h"  // NOLINT
#include "vector_nav.h"  // NOLINT
#include "registers.h"  // NOLINT

namespace bfs {

constexpr char Vn300::PROD_NAME_[];

bool Vn300::Begin() {
  vn_.Init();
  error_code_ = vn_.ReadRegister(&serial_num_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  if (serial_num_.payload.serial_num == 0) {
    error_code_ = VectorNav::ERROR_NO_COMM;
    return false;
  }
  error_code_ = vn_.ReadRegister(&model_num_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  for (size_t i = 0; i < sizeof(PROD_NAME_) - 1; i++) {
    if (model_num_.payload.product_name[i] != PROD_NAME_[i]) {
      error_code_ = VectorNav::ERROR_WRONG_MODEL;
      return false;
    }
  }
  return true;
}

bool Vn300::EnableDrdyInt(const DrdyMode mode, const uint16_t srd) {
  error_code_ = vn_.ReadRegister(&sync_cntrl_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  enum SyncOutPolarity : uint8_t {
    NEG_PULSE = 0,
    POS_PULSE = 1
  };
  sync_cntrl_.payload.sync_out_mode = static_cast<uint8_t>(mode);
  sync_cntrl_.payload.sync_out_polarity = POS_PULSE;
  sync_cntrl_.payload.sync_out_pulse_width = 500000;
  sync_cntrl_.payload.sync_out_skip_factor = srd;
  error_code_ = vn_.WriteRegister(sync_cntrl_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::DisableDrdyInt() {
  error_code_ = vn_.ReadRegister(&sync_cntrl_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  sync_cntrl_.payload.sync_out_mode = 0;
  error_code_ = vn_.WriteRegister(sync_cntrl_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::SetMagFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.mag_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.mag_window_size = window;
  error_code_ = vn_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetMagFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.mag_filter_mode);
  *window = filter_.payload.mag_window_size;
  return true;
}

bool Vn300::SetAccelFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.accel_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.accel_window_size = window;
  error_code_ = vn_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetAccelFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.accel_filter_mode);
  *window = filter_.payload.accel_window_size;
  return true;
}

bool Vn300::SetGyroFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.gyro_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.gyro_window_size = window;
  error_code_ = vn_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetGyroFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.gyro_filter_mode);
  *window = filter_.payload.gyro_window_size;
  return true;
}

bool Vn300::SetTemperatureFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.temp_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.temp_window_size = window;
  error_code_ = vn_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetTemperatureFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.temp_filter_mode);
  *window = filter_.payload.temp_window_size;
  return true;
}

bool Vn300::SetPressureFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.pres_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.pres_window_size = window;
  error_code_ = vn_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetPressureFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.pres_filter_mode);
  *window = filter_.payload.pres_window_size;
  return true;
}

bool Vn300::DrdyCallback(const uint8_t int_pin, void (*function)()) {
  if (!function) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  pinMode(int_pin, INPUT);
  attachInterrupt(int_pin, function, RISING);
  error_code_ = VectorNav::ERROR_SUCCESS;
  return true;
}

bool Vn300::Read() {
  error_code_ = vn_.ReadRegister(&ins_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  error_code_ = vn_.ReadRegister(&gnss_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  error_code_ = vn_.ReadRegister(&comp_imu_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  error_code_ = vn_.ReadRegister(&uncomp_imu_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  /* INS status parsing */
  ins_status_buff_[0] = ins_.payload.status & 0xFF;
  ins_status_buff_[1] = ins_.payload.status >> 8 & 0xFF;
  ins_mode_ = static_cast<InsMode>(ins_status_buff_[0] & 0x03);
  ins_gnss_fix_ = ins_status_buff_[0] & 0x04;
  ins_time_error_ = ins_status_buff_[0] & 0x08;
  ins_imu_error_ = ins_status_buff_[0] & 0x10;
  ins_mag_press_error_ = ins_status_buff_[0] & 0x20;
  ins_gnss_error_ = ins_status_buff_[0] & 0x40;
  ins_error_ = ins_time_error_ || ins_imu_error_ ||
               ins_mag_press_error_ || ins_gnss_error_;
  ins_gnss_heading_ = ins_status_buff_[1] & 0x01;
  ins_gnss_compass_ = ins_status_buff_[1] & 0x02;
  return true;
}

}  // namespace bfs
