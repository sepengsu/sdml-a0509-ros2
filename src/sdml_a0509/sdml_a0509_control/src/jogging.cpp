#include <memory>
#include <string>
#include <array>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "dsr_msgs2/msg/jog_multi_axis.hpp"
#include "dsr_msgs2/srv/jog_multi.hpp"
#include "dsr_msgs2/srv/move_stop.hpp"

class JogMultiAxisBridgeNode : public rclcpp::Node
{
public:
  JogMultiAxisBridgeNode()
  : Node("jogging")
  {
    this->declare_parameter<std::string>("jog_multi_axis_topic", "jog_multi_axis");
    this->declare_parameter<std::string>("jog_multi_service", "motion/jog_multi");
    this->declare_parameter<std::string>("move_stop_service", "motion/move_stop");
    this->declare_parameter<int>("move_stop_mode", 2);

    std::string topic_name =
      this->get_parameter("jog_multi_axis_topic").get_parameter_value().get<std::string>();
    std::string service_name =
      this->get_parameter("jog_multi_service").get_parameter_value().get<std::string>();
    move_stop_service_name_ =
      this->get_parameter("move_stop_service").get_parameter_value().get<std::string>();
    move_stop_mode_ = this->get_parameter("move_stop_mode").get_parameter_value().get<int>();

    jog_multi_client_ =
      this->create_client<dsr_msgs2::srv::JogMulti>(service_name);
    move_stop_client_ =
      this->create_client<dsr_msgs2::srv::MoveStop>(move_stop_service_name_);

    last_cmd_time_ = this->now();
    has_cmd_ = false;
    stopped_by_watchdog_ = false;

    jog_multi_axis_sub_ = this->create_subscription<dsr_msgs2::msg::JogMultiAxis>(
      topic_name,
      rclcpp::QoS(10),
      std::bind(&JogMultiAxisBridgeNode::jogMultiAxisCallback, this, std::placeholders::_1));

    using namespace std::chrono_literals;
    watchdog_timer_ = this->create_wall_timer(
      100ms,
      std::bind(&JogMultiAxisBridgeNode::watchdogTimerCallback, this));
  }

private:
  void jogMultiAxisCallback(const dsr_msgs2::msg::JogMultiAxis::SharedPtr msg)
  {
    if (!jog_multi_client_->service_is_ready()) {
      return;
    }

    last_cmd_time_ = this->now();
    has_cmd_ = true;
    stopped_by_watchdog_ = false;

    auto request = std::make_shared<dsr_msgs2::srv::JogMulti::Request>();
    for (size_t i = 0; i < msg->jog_axis.size(); ++i) {
      request->jog_axis[i] = static_cast<double>(msg->jog_axis[i]);
    }
    request->move_reference = msg->move_reference;
    request->speed = msg->speed;

    using ResponseFuture =
      rclcpp::Client<dsr_msgs2::srv::JogMulti>::SharedFuture;

    auto response_cb =
      [this](ResponseFuture future) {
        auto res = future.get();
        if (!res->success) {
          RCLCPP_WARN(this->get_logger(), "JogMulti service returned success=false");
        }
      };

    jog_multi_client_->async_send_request(request, response_cb);
  }

  void watchdogTimerCallback()
  {
    if (!has_cmd_) {
      return;
    }
    if (!jog_multi_client_ || !jog_multi_client_->service_is_ready()) {
      return;
    }

    const double timeout_sec = 0.5;
    auto now = this->now();
    double dt = (now - last_cmd_time_).seconds();

    if (dt < timeout_sec || stopped_by_watchdog_) {
      return;
    }

    auto stop_req = std::make_shared<dsr_msgs2::srv::JogMulti::Request>();
    stop_req->jog_axis.fill(0.0);
    stop_req->move_reference = 0;
    stop_req->speed = 0.0;

    using ResponseFuture =
      rclcpp::Client<dsr_msgs2::srv::JogMulti>::SharedFuture;

    auto cb =
      [this](ResponseFuture future) {
        auto res = future.get();
        if (!res->success) {
          RCLCPP_WARN(this->get_logger(), "Watchdog stop JogMulti failed (success=false)");
        }
        sendMoveStopAfterJogStop();
      };

    jog_multi_client_->async_send_request(stop_req, cb);
    stopped_by_watchdog_ = true;
  }

  void sendMoveStopAfterJogStop()
  {
    if (!move_stop_client_ || !move_stop_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<dsr_msgs2::srv::MoveStop::Request>();
    req->stop_mode = move_stop_mode_;
    move_stop_client_->async_send_request(
      req,
      [this](rclcpp::Client<dsr_msgs2::srv::MoveStop>::SharedFuture future) {
        auto r = future.get();
        if (!r->success) {
          RCLCPP_WARN(this->get_logger(), "move_stop returned success=false");
        }
      });
  }

  rclcpp::Subscription<dsr_msgs2::msg::JogMultiAxis>::SharedPtr jog_multi_axis_sub_;
  rclcpp::Client<dsr_msgs2::srv::JogMulti>::SharedPtr jog_multi_client_;
  rclcpp::Client<dsr_msgs2::srv::MoveStop>::SharedPtr move_stop_client_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_cmd_time_;
  bool has_cmd_{false};
  bool stopped_by_watchdog_{false};
  std::string move_stop_service_name_;
  int move_stop_mode_{2};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JogMultiAxisBridgeNode>());
  rclcpp::shutdown();
  return 0;
}

