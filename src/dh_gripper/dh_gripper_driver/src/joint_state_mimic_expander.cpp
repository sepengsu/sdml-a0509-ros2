#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <urdf/model.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>

using sensor_msgs::msg::JointState;

struct MimicRule
{
  std::string driver;
  double multiplier{1.0};
  double offset{0.0};
};

class JointStateMimicExpander : public rclcpp::Node
{
public:
  JointStateMimicExpander()
  : Node("joint_state_mimic_expander")
  {
    // Parameters
    this->declare_parameter<std::string>("input_topic", "/ag95/joint_states");
    this->declare_parameter<std::string>("output_topic", "/ag95/joint_states_full");
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<bool>("publish_only_mimics", true);
    this->declare_parameter<bool>("keep_incoming_order", true);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    publish_only_mimics_ = this->get_parameter("publish_only_mimics").as_bool();
    keep_incoming_order_ = this->get_parameter("keep_incoming_order").as_bool();

    const auto urdf_xml = this->get_parameter("robot_description").as_string();
    if (trim(urdf_xml).empty()) {
      RCLCPP_ERROR(get_logger(),
        "robot_description parameter is empty. Pass the same robot_description used by robot_state_publisher.");
      throw std::runtime_error("robot_description is required");
    }

    parse_urdf_mimics_with_urdf_model(urdf_xml);

    sub_ = this->create_subscription<JointState>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&JointStateMimicExpander::cb, this, std::placeholders::_1));

    pub_ = this->create_publisher<JointState>(output_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "Subscribed: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Mimic joints found: %zu", mimic_map_.size());
  }

private:
  std::string input_topic_;
  std::string output_topic_;
  bool publish_only_mimics_{true};
  bool keep_incoming_order_{true};

  rclcpp::Subscription<JointState>::SharedPtr sub_;
  rclcpp::Publisher<JointState>::SharedPtr pub_;

  // mimic_joint -> (driver, mul, offset)
  std::unordered_map<std::string, MimicRule> mimic_map_;

  // Non-fixed joints found in URDF. (order here is not guaranteed by urdf::Model map)
  std::vector<std::string> urdf_joint_list_;
  std::unordered_set<std::string> non_fixed_joints_;

  static std::string trim(const std::string& s)
  {
    const auto wsfront = std::find_if_not(s.begin(), s.end(), ::isspace);
    const auto wsback  = std::find_if_not(s.rbegin(), s.rend(), ::isspace).base();
    if (wsback <= wsfront) return "";
    return std::string(wsfront, wsback);
  }

  void parse_urdf_mimics_with_urdf_model(const std::string& urdf_xml)
  {
    urdf::Model model;
    if (!model.initString(urdf_xml)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse robot_description using urdf::Model::initString()");
      throw std::runtime_error("URDF parse failed");
    }

    mimic_map_.clear();
    non_fixed_joints_.clear();
    urdf_joint_list_.clear();

    // model.joints_ is a map; iteration order is not URDF file order.
    // This is fine because we match by name and (optionally) keep incoming order.
    for (const auto& kv : model.joints_) {
      const std::string& joint_name = kv.first;
      const urdf::JointSharedPtr& joint = kv.second;
      if (!joint) continue;
      if (joint->type == urdf::Joint::FIXED) continue;

      non_fixed_joints_.insert(joint_name);
      urdf_joint_list_.push_back(joint_name);

      if (joint->mimic) {
        MimicRule rule;
        rule.driver = joint->mimic->joint_name;
        rule.multiplier = joint->mimic->multiplier;
        rule.offset = joint->mimic->offset;
        mimic_map_[joint_name] = rule;
      }
    }

    RCLCPP_INFO(get_logger(), "Non-fixed joints found: %zu", non_fixed_joints_.size());
  }

  double resolve_joint_value(
    const std::string& joint,
    std::unordered_map<std::string, double>& values,
    std::unordered_set<std::string>& visiting)
  {
    auto it = values.find(joint);
    if (it != values.end()) return it->second;

    auto mit = mimic_map_.find(joint);
    if (mit == mimic_map_.end()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    if (visiting.count(joint)) {
      // cycle detected
      return std::numeric_limits<double>::quiet_NaN();
    }

    visiting.insert(joint);

    const auto& rule = mit->second;
    const double driver_val = resolve_joint_value(rule.driver, values, visiting);

    visiting.erase(joint);

    if (std::isnan(driver_val)) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    return rule.multiplier * driver_val + rule.offset;
  }

  void cb(const JointState::SharedPtr msg)
  {
    const size_t n = std::min(msg->name.size(), msg->position.size());
    if (n == 0) return;

    // Build incoming name->pos map
    std::unordered_map<std::string, double> values;
    values.reserve(n + mimic_map_.size());

    for (size_t i = 0; i < n; ++i) {
      values[msg->name[i]] = msg->position[i];
    }

    // Fill missing mimic joints using mimic constraints (can handle mimic chains)
    for (const auto& kv : mimic_map_) {
      const std::string& mimic_joint = kv.first;
      if (values.find(mimic_joint) != values.end()) continue;

      std::unordered_set<std::string> visiting;
      const double v = resolve_joint_value(mimic_joint, values, visiting);
      if (!std::isnan(v)) {
        values[mimic_joint] = v;
      }
    }

    JointState out;
    out.header = msg->header;

    if (keep_incoming_order_) {
      // Keep original joint order first
      out.name.assign(msg->name.begin(), msg->name.begin() + static_cast<long>(n));
      out.position.assign(msg->position.begin(), msg->position.begin() + static_cast<long>(n));

      std::unordered_set<std::string> existing(out.name.begin(), out.name.end());

      // Append remaining joints deterministically (sorted by name for stable output)
      std::vector<std::string> candidates = urdf_joint_list_;
      std::sort(candidates.begin(), candidates.end());

      for (const auto& jn : candidates) {
        if (existing.count(jn)) continue;

        if (publish_only_mimics_ && mimic_map_.find(jn) == mimic_map_.end()) {
          continue;
        }

        auto it = values.find(jn);
        if (it != values.end()) {
          out.name.push_back(jn);
          out.position.push_back(it->second);
          existing.insert(jn);
        }
      }
    } else {
      // Publish all (or only mimics) in sorted URDF joint list for determinism
      std::vector<std::string> names = urdf_joint_list_;
      std::sort(names.begin(), names.end());

      for (const auto& jn : names) {
        if (publish_only_mimics_ && mimic_map_.find(jn) == mimic_map_.end()) {
          continue;
        }
        auto it = values.find(jn);
        if (it != values.end()) {
          out.name.push_back(jn);
          out.position.push_back(it->second);
        }
      }
    }

    pub_->publish(out);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<JointStateMimicExpander>();
    rclcpp::spin(node);
  } catch (const std::exception&) {
    // error already logged
  }
  rclcpp::shutdown();
  return 0;
}
