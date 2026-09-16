// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "dh_gripper_driver/default_driver.hpp"
#include <dh_gripper_driver/driver_exception.hpp>

#include <modbus/modbus.h>
#include <cerrno>

#include <rclcpp/rclcpp.hpp>

namespace dh_gripper_driver
{
const auto kLogger = rclcpp::get_logger("DefaultDriver");

constexpr uint8_t kReadFunctionCode = 0x03;
constexpr uint16_t kFirstOutputRegister = 0x0200;
constexpr int kReadResponseSize = 7;

constexpr uint8_t kWriteFunctionCode = 0x06;
constexpr uint16_t kActionRequestRegister = 0x0100;
constexpr int kWriteResponseSize = 8;

// If the gripper connection is not stable we may want to try sending the command again.
constexpr auto kMaxRetries = 5;

DefaultDriver::DefaultDriver(const std::string& port, uint32_t baudrate, uint8_t slave_address)
  : port_{ port }
  , baudrate_{ baudrate }
  , slave_address_{ slave_address }
  , commanded_gripper_speed_(0x80)
  , commanded_gripper_force_(0x80)
{
}

std::vector<uint8_t> DefaultDriver::send(const std::vector<uint8_t>& request, size_t response_size) const
{
  (void)request;
  (void)response_size;
  // libmodbus 버전에서는 직접 프레임을 만들지 않고 high-level API를 사용하므로
  // send는 더 이상 사용되지 않습니다. 남겨두지만 빈 구현입니다.
  return {};
}

bool DefaultDriver::connect()
{
  if (modbus_ctx_ != nullptr)
  {
    return true;
  }

  modbus_t* ctx = modbus_new_rtu(port_.c_str(), static_cast<int>(baudrate_), 'N', 8, 1);
  if (ctx == nullptr)
  {
    RCLCPP_ERROR(kLogger, "Failed to create modbus RTU context");
    return false;
  }

  if (modbus_set_slave(ctx, slave_address_) == -1)
  {
    RCLCPP_ERROR(kLogger, "Failed to set modbus slave: %s", modbus_strerror(errno));
    modbus_free(ctx);
    return false;
  }

  if (modbus_connect(ctx) == -1)
  {
    RCLCPP_ERROR(kLogger, "Unable to connect to gripper: %s", modbus_strerror(errno));
    modbus_free(ctx);
    return false;
  }

  modbus_ctx_ = ctx;
  RCLCPP_INFO(kLogger, "Modbus RTU connected to gripper (port=%s, baudrate=%d)", port_.c_str(), baudrate_);
  return true;
}

void DefaultDriver::disconnect()
{
  if (modbus_ctx_ != nullptr)
  {
    modbus_t* ctx = static_cast<modbus_t*>(modbus_ctx_);
    modbus_close(ctx);
    modbus_free(ctx);
    modbus_ctx_ = nullptr;
    RCLCPP_INFO(kLogger, "Modbus connection to gripper closed.");
  }
}

void DefaultDriver::set_slave_address(uint8_t slave_address)
{
  slave_address_ = slave_address;
  if (modbus_ctx_ != nullptr)
  {
    modbus_set_slave(static_cast<modbus_t*>(modbus_ctx_), slave_address_);
  }
}

void DefaultDriver::activate()
{
  RCLCPP_INFO(kLogger, "Activate...");

  // rACT = 1
  if (modbus_ctx_ == nullptr)
  {
    throw DriverException{ "Modbus context not initialized. Call connect() first." };
  }
  auto* ctx = static_cast<modbus_t*>(modbus_ctx_);

  if (modbus_write_register(ctx, kActionRequestRegister, 0x0001) != 1)
  {
    throw DriverException{ "Failed to activate the gripper via Modbus." };
  }

  update_status();
  if (activation_status_ == ActivationStatus::ACTIVE)
  {
    return;
  }
  while (activation_status_ != ActivationStatus::ACTIVE)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    update_status();
  }
}

void DefaultDriver::deactivate()
{
  // RCLCPP_INFO(kLogger, "Deactivate...");

  // const auto request = create_write_command(kActionRequestRegister, { 0x0000, 0x0000, 0x0000, 0x0000 });
  // auto response = send(request, kWriteResponseSize);
  // if (response.empty())
  // {
  //   throw DriverException{ "Failed to deactivate the gripper." };
  // }
}

void DefaultDriver::set_gripper_position(uint16_t pos)
{
  if (modbus_ctx_ == nullptr)
  {
    throw DriverException{ "Modbus context not initialized. Call connect() first." };
  }
  auto* ctx = static_cast<modbus_t*>(modbus_ctx_);

  // force 레지스터 설정 (0x0101)
  if (modbus_write_register(ctx, kActionRequestRegister + 1, static_cast<uint16_t>(100)) != 1)
  {
    throw DriverException{ "Failed to set gripper force via Modbus." };
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // position 레지스터 (0x0103)
  if (modbus_write_register(ctx, kActionRequestRegister + 3, pos) != 1)
  {
    throw DriverException{ "Failed to set gripper position via Modbus." };
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(5));


  
}

uint16_t DefaultDriver::get_gripper_position()
{
  update_status();
  // std::cout  << "grip pos driver = " << std::dec << unsigned(gripper_position_) << std::endl;
  return gripper_position_;
}

bool DefaultDriver::gripper_is_moving()
{
  update_status();
  return object_detection_status_ == ObjectDetectionStatus::MOVING;
}

void DefaultDriver::set_speed(uint8_t speed)
{
  commanded_gripper_speed_ = speed;
}

void DefaultDriver::set_force(uint8_t force)
{
  commanded_gripper_force_ = force;
}

void DefaultDriver::update_status()
{
  if (modbus_ctx_ == nullptr)
  {
    throw DriverException{ "Modbus context not initialized. Call connect() first." };
  }
  auto* ctx = static_cast<modbus_t*>(modbus_ctx_);

  uint16_t reg = 0;
  // 0x0200: 상태 레지스터 (rACT 등)
  if (modbus_read_registers(ctx, kFirstOutputRegister, 1, &reg) != 1)
  {
    throw DriverException{ "Failed to read gripper status via Modbus." };
  }

  activation_status_ = (reg & 0x0001) ? ActivationStatus::ACTIVE : ActivationStatus::RESET;

  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // 0x0202: 현재 위치 레지스터
  if (modbus_read_registers(ctx, kFirstOutputRegister + 2, 1, &reg) != 1)
  {
    throw DriverException{ "Failed to read gripper position via Modbus." };
  }
  gripper_position_ = reg;

  // log gripper position
  // RCLCPP_INFO(kLogger, "Response2: %s", data_utils::to_hex(response2).c_str());

  std::this_thread::sleep_for(std::chrono::milliseconds(5));


}
}  // namespace dh_gripper_driver
