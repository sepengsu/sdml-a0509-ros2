#include "a0509_control_cyw/gripper_controller.hpp"
#include <unistd.h>
#include <algorithm>
#include <cstring>

GripperController::GripperController(const std::string& device, rclcpp::Logger logger, bool mock_mode)
: modbus_(nullptr), logger_(logger), mock_mode_(mock_mode)
{
  // Check environment variables
  const char* env_mock = std::getenv("GRIPPER_MOCK");
  const char* env_external = std::getenv("GRIPPER_EXTERNAL");
  const char* env_host = std::getenv("GRIPPER_HOST");
  const char* env_port = std::getenv("GRIPPER_PORT_TCP");

  // Priority: GRIPPER_MOCK > GRIPPER_EXTERNAL > direct Modbus
  if (env_mock != nullptr && (std::string(env_mock) == "true" || std::string(env_mock) == "1")) {
    mock_mode_ = true;
    RCLCPP_WARN(logger_, "GripperController running in MOCK MODE - no real hardware");
    return;
  }

  if (env_external != nullptr && (std::string(env_external) == "true" || std::string(env_external) == "1")) {
    external_mode_ = true;
    tcp_host_ = env_host ? env_host : "host.docker.internal";
    tcp_port_ = env_port ? std::stoi(env_port) : 9999;
    RCLCPP_INFO(logger_, "GripperController running in EXTERNAL MODE - connecting to %s:%d",
                tcp_host_.c_str(), tcp_port_);
    return;
  }

  // Direct Modbus mode
  modbus_ = modbus_new_rtu(device.c_str(), 115200, 'N', 8, 1);
  if (modbus_ == nullptr) {
    RCLCPP_ERROR(logger_, "Failed to create modbus context for device: %s", device.c_str());
    return;
  }
  modbus_set_slave(modbus_, 1);
  RCLCPP_INFO(logger_, "GripperController using direct Modbus on %s", device.c_str());
}

GripperController::~GripperController()
{
  if (external_mode_) {
    disconnectTcp();
  } else if (modbus_ != nullptr) {
    modbus_close(modbus_);
    modbus_free(modbus_);
    RCLCPP_INFO(logger_, "Gripper Modbus connection closed.");
  }
}

bool GripperController::connectTcp()
{
  if (tcp_socket_ >= 0) {
    return true;  // Already connected
  }

  // Resolve hostname
  struct addrinfo hints, *result;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  std::string port_str = std::to_string(tcp_port_);
  int status = getaddrinfo(tcp_host_.c_str(), port_str.c_str(), &hints, &result);
  if (status != 0) {
    RCLCPP_ERROR(logger_, "Failed to resolve host %s: %s", tcp_host_.c_str(), gai_strerror(status));
    return false;
  }

  // Create socket
  tcp_socket_ = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
  if (tcp_socket_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to create socket: %s", strerror(errno));
    freeaddrinfo(result);
    return false;
  }

  // Disable Nagle's algorithm for low latency
  int flag = 1;
  setsockopt(tcp_socket_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

  // Set timeout
  struct timeval timeout;
  timeout.tv_sec = 5;
  timeout.tv_usec = 0;
  setsockopt(tcp_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(tcp_socket_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  // Connect
  if (connect(tcp_socket_, result->ai_addr, result->ai_addrlen) < 0) {
    RCLCPP_ERROR(logger_, "Failed to connect to %s:%d: %s",
                 tcp_host_.c_str(), tcp_port_, strerror(errno));
    close(tcp_socket_);
    tcp_socket_ = -1;
    freeaddrinfo(result);
    return false;
  }

  freeaddrinfo(result);
  RCLCPP_INFO(logger_, "Connected to gripper server at %s:%d", tcp_host_.c_str(), tcp_port_);
  return true;
}

void GripperController::disconnectTcp()
{
  if (tcp_socket_ >= 0) {
    close(tcp_socket_);
    tcp_socket_ = -1;
    RCLCPP_INFO(logger_, "Disconnected from gripper server");
  }
}

bool GripperController::sendTcpCommand(uint8_t cmd, uint16_t position, uint16_t force)
{
  if (tcp_socket_ < 0) {
    if (!connectTcp()) {
      return false;
    }
  }

  // Build command packet: [cmd:1][position:2][force:2] (little-endian)
  uint8_t packet[gripper_protocol::COMMAND_SIZE];
  packet[0] = cmd;
  packet[1] = position & 0xFF;
  packet[2] = (position >> 8) & 0xFF;
  packet[3] = force & 0xFF;
  packet[4] = (force >> 8) & 0xFF;

  // Send command
  ssize_t sent = send(tcp_socket_, packet, sizeof(packet), 0);
  if (sent != sizeof(packet)) {
    RCLCPP_ERROR(logger_, "Failed to send command: %s", strerror(errno));
    disconnectTcp();
    return false;
  }

  // Receive response: [status:1][position:2][force:2][flags:1]
  uint8_t response[gripper_protocol::RESPONSE_SIZE];
  ssize_t received = recv(tcp_socket_, response, sizeof(response), 0);
  if (received != sizeof(response)) {
    RCLCPP_ERROR(logger_, "Failed to receive response: %s", strerror(errno));
    disconnectTcp();
    return false;
  }

  // Parse response
  uint8_t status = response[0];
  current_position_ = response[1] | (response[2] << 8);
  current_force_ = response[3] | (response[4] << 8);
  uint8_t flags = response[5];

  is_initialized_ = (flags & 0x01) != 0;
  object_held_ = (flags & 0x02) != 0;

  return status == gripper_protocol::OK;
}

bool GripperController::initialize()
{
  if (mock_mode_) {
    RCLCPP_INFO(logger_, "[MOCK] Gripper initialized (simulated)");
    is_initialized_ = true;
    return true;
  }

  if (external_mode_) {
    if (sendTcpCommand(gripper_protocol::INIT)) {
      RCLCPP_INFO(logger_, "[EXTERNAL] Gripper initialized via TCP");
      return true;
    }
    RCLCPP_ERROR(logger_, "[EXTERNAL] Failed to initialize gripper");
    return false;
  }

  // Direct Modbus mode
  if (modbus_ == nullptr) {
    RCLCPP_ERROR(logger_, "Modbus context not available");
    return false;
  }

  if (modbus_connect(modbus_) == -1) {
    RCLCPP_ERROR(logger_, "Unable to connect to gripper: %s", modbus_strerror(errno));
    modbus_free(modbus_);
    modbus_ = nullptr;
    return false;
  }
  RCLCPP_INFO(logger_, "Modbus connected to gripper");

  modbus_write_register(modbus_, 0x0100, 0x01); // Init command
  usleep(500000); // 0.5s wait
  RCLCPP_INFO(logger_, "Gripper initialized.");
  is_initialized_ = true;
  return true;
}

void GripperController::setForce(int force)
{
  if (!is_initialized_) return;
  force = std::clamp(force, 20, 100);

  if (mock_mode_) {
    RCLCPP_INFO(logger_, "[MOCK] Set gripper force: %d%% (simulated)", force);
    current_force_ = force;
    return;
  }

  if (external_mode_) {
    if (sendTcpCommand(gripper_protocol::SET_FORCE, 0, force)) {
      RCLCPP_INFO(logger_, "[EXTERNAL] Set gripper force: %d%%", force);
    }
    return;
  }

  modbus_write_register(modbus_, 0x0101, force);
  RCLCPP_INFO(logger_, "Set gripper force: %d%%", force);
  current_force_ = force;
  usleep(20000); // short delay
}

void GripperController::setPosition(int position)
{
  if (!is_initialized_) return;
  position = std::clamp(position, 0, 1000);

  if (mock_mode_) {
    RCLCPP_INFO(logger_, "[MOCK] Set gripper position: %d/1000 (simulated)", position);
    current_position_ = position;
    object_held_ = position < 200;  // Simulate object detection
    return;
  }

  if (external_mode_) {
    if (sendTcpCommand(gripper_protocol::SET_POSITION, position, 0)) {
      RCLCPP_INFO(logger_, "[EXTERNAL] Set gripper position: %d/1000", position);
    }
    return;
  }

  modbus_write_register(modbus_, 0x0103, position);
  RCLCPP_INFO(logger_, "Set gripper position: %d/1000", position);
  current_position_ = position;
}

void GripperController::open()
{
  if (mock_mode_) {
    RCLCPP_INFO(logger_, "[MOCK] Gripper opening (simulated)");
    current_position_ = 1000;
    object_held_ = false;
    return;
  }

  if (external_mode_) {
    if (sendTcpCommand(gripper_protocol::OPEN)) {
      RCLCPP_INFO(logger_, "[EXTERNAL] Gripper opened");
    }
    return;
  }

  RCLCPP_INFO(logger_, "Opening gripper to maximum position.");
  setForce(255); // Use max force for opening
  setPosition(1000); // Max opening position
}

void GripperController::close(double mm)
{
  if (mock_mode_) {
    RCLCPP_INFO(logger_, "[MOCK] Gripper closing to %f mm (simulated)", mm);
    current_position_ = convertDistanceToPosition(mm);
    object_held_ = true;
    return;
  }

  int pos = convertDistanceToPosition(mm);

  if (external_mode_) {
    setForce(100);
    if (sendTcpCommand(gripper_protocol::CLOSE, pos, 0)) {
      RCLCPP_INFO(logger_, "[EXTERNAL] Gripper closed to %f mm (pos=%d)", mm, pos);
    }
    return;
  }

  RCLCPP_INFO(logger_, "Closing gripper to %f mm.", mm);
  setForce(255); // Use max force for gripping
  setPosition(pos);
}

int GripperController::convertDistanceToPosition(double mm)
{
    std::vector<double> mm_table    = {0, 9, 18, 35, 67, 96, 122, 145};
    std::vector<int>    pos_table   = {0, 50, 100, 200, 400, 600, 800, 1000};

    if (mm <= mm_table.front()) return pos_table.front();
    if (mm >= mm_table.back())  return pos_table.back();

    for (size_t i = 0; i < mm_table.size() - 1; ++i) {
        if (mm >= mm_table[i] && mm <= mm_table[i + 1]) {
            double ratio = (mm - mm_table[i]) / (mm_table[i + 1] - mm_table[i]);
            return static_cast<int>(pos_table[i] + ratio * (pos_table[i + 1] - pos_table[i]));
        }
    }
    return 0; // Should not happen
}
