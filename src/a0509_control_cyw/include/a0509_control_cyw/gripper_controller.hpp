#ifndef GRIPPER_CONTROLLER_HPP_
#define GRIPPER_CONTROLLER_HPP_

#include <modbus/modbus.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <cstdint>

// TCP Protocol definitions (must match dsr2/gripper_protocol.py)
namespace gripper_protocol {
  enum Command : uint8_t {
    SET_POSITION = 0x01,
    GET_STATE = 0x02,
    INIT = 0x03,
    STOP = 0x04,
    SET_FORCE = 0x05,
    OPEN = 0x06,
    CLOSE = 0x07
  };

  enum Status : uint8_t {
    OK = 0x00,
    ERROR = 0x01,
    NOT_INITIALIZED = 0x02
  };

  // Packet sizes
  constexpr size_t COMMAND_SIZE = 5;   // cmd:1 + position:2 + force:2
  constexpr size_t RESPONSE_SIZE = 6;  // status:1 + position:2 + force:2 + flags:1
}

class GripperController
{
public:
  GripperController(const std::string& device, rclcpp::Logger logger, bool mock_mode = false);
  ~GripperController();

  bool initialize();
  void setForce(int force);
  void setPosition(int position);
  void open();
  void close(double mm);

  // State getters
  int getPosition() const { return current_position_; }
  int getForce() const { return current_force_; }
  bool isObjectHeld() const { return object_held_; }
  bool isInitialized() const { return is_initialized_; }

private:
  int convertDistanceToPosition(double mm);

  // TCP external mode methods
  bool connectTcp();
  void disconnectTcp();
  bool sendTcpCommand(uint8_t cmd, uint16_t position = 0, uint16_t force = 0);

  modbus_t *modbus_;
  rclcpp::Logger logger_;
  bool is_initialized_ = false;
  bool mock_mode_ = false;      // Enable mock mode for virtual testing
  bool external_mode_ = false;  // Enable TCP external mode (host gripper server)

  // TCP connection for external mode
  int tcp_socket_ = -1;
  std::string tcp_host_;
  int tcp_port_ = 9999;

  // State tracking
  int current_position_ = 0;
  int current_force_ = 100;
  bool object_held_ = false;
};

#endif // GRIPPER_CONTROLLER_HPP_