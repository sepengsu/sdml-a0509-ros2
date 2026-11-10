#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// MoveIt 관련 헤더
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// 로봇 모델 로드를 위한 추가 헤더
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <streambuf>
#include <iostream>
#include <thread>
#include <atomic>

// 파일 내용을 문자열로 읽어오는 헬퍼 함수
std::string load_file_to_string(const std::string& file_path) {
    std::ifstream file_stream(file_path);
    if (!file_stream) {
        throw std::runtime_error("Could not open file: " + file_path);
    }
    return std::string((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());
}

class RelativeMoveClient : public rclcpp::Node
{
public:
  RelativeMoveClient() : Node("ee_relative_move_client")
  {
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    ee_frame_   = this->declare_parameter<std::string>("ee_frame",   "tool0");
    std::string planning_group = this->declare_parameter<std::string>("planning_group", "manipulator");
    std::string model = this->declare_parameter<std::string>("model", "a0509");

    // TF 버퍼 초기화
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- MoveIt Init (arm_gripper_coordinator와 동일한 방식) ---
    RCLCPP_INFO(this->get_logger(), "Loading robot description for model: %s", model.c_str());
    
    std::string urdf_path, srdf_path;
    try {
        std::string urdf_pkg_path = ament_index_cpp::get_package_share_directory("dsr_description2");
        std::string srdf_pkg_path = ament_index_cpp::get_package_share_directory("dsr_moveit_config_" + model);
        urdf_path = urdf_pkg_path + "/urdf/" + model + ".urdf";
        srdf_path = srdf_pkg_path + "/config/dsr.srdf";
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Required package not found. Is the workspace sourced? Error: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    std::string urdf_content = load_file_to_string(urdf_path);
    std::string srdf_content = load_file_to_string(srdf_path);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.parameter_overrides({
        {"robot_description", urdf_content},
        {"robot_description_semantic", srdf_content}
    });

    // MoveGroupInterface를 위한 별도 노드 및 스레드 생성
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_node", node_options);
    executor_thread_ = std::thread([move_group_node]() {
        rclcpp::spin(move_group_node);
    });

    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, planning_group);
    move_group_interface_->setEndEffectorLink(ee_frame_);
    move_group_interface_->setMaxVelocityScalingFactor(0.5); // 안전을 위해 속도 제한
    move_group_interface_->setMaxAccelerationScalingFactor(0.5);

    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized. Ready for Cartesian control.");
  }

  ~RelativeMoveClient() {
      if (executor_thread_.joinable()) {
          executor_thread_.join();
      }
  }

  void run()
  {
    waitForTF(base_frame_, ee_frame_, 10.0);

    while (rclcpp::ok()) {
      geometry_msgs::msg::TransformStamped ee_tf;
      if (!lookupEE(ee_tf, 2.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current EE pose. Retrying...");
        continue;
      }

      RCLCPP_INFO(this->get_logger(),
        "Current EE pose: [%.3f %.3f %.3f | %.3f %.3f %.3f %.3f]",
        ee_tf.transform.translation.x,
        ee_tf.transform.translation.y,
        ee_tf.transform.translation.z,
        ee_tf.transform.rotation.x,
        ee_tf.transform.rotation.y,
        ee_tf.transform.rotation.z,
        ee_tf.transform.rotation.w);

      double dx, dy, dz, dqx, dqy, dqz, dqw;
      std::cout << "\n[Cartesian Mode] Enter EE relative translation (dx dy dz): ";
      std::cin >> dx >> dy >> dz;
      std::cout << "[Cartesian Mode] Enter relative orientation (dqx dqy dqz dqw): ";
      std::cin >> dqx >> dqy >> dqz >> dqw;

      geometry_msgs::msg::PoseStamped target;
      target.header.frame_id = base_frame_;
      target.header.stamp = now();

      // ===== 상대 좌표 계산 (기존 로직 유지) =====
// ===== 상대 좌표 계산 (수정됨: World Frame 기준 이동) =====
      tf2::Quaternion q_cur;
      tf2::fromMsg(ee_tf.transform.rotation, q_cur);
      
      // 현재 위치 (Base Frame 기준)
      tf2::Vector3 p_cur(ee_tf.transform.translation.x,
                         ee_tf.transform.translation.y,
                         ee_tf.transform.translation.z);

      // 입력받은 상대 이동량 (dx, dy, dz)
      // 사용자가 World Frame(Base Link) 기준으로 입력을 원하므로 회전 변환을 제거합니다.
      tf2::Vector3 p_delta(dx, dy, dz); 
      
      // 현재 위치에 바로 더함 (World Frame 기준 직선 이동)
      tf2::Vector3 p_target_vec = p_cur + p_delta;
      
      target.pose.position.x = p_target_vec.x();
      target.pose.position.y = p_target_vec.y();
      target.pose.position.z = p_target_vec.z();

      // 회전(Orientation)은 End-Effector 기준으로 돌리는 것이 직관적
      tf2::Quaternion q_delta(dqx, dqy, dqz, dqw);
      if (q_delta.length2() == 0.0) q_delta.setValue(0,0,0,1);
      
      // q_cur * q_delta : Local Frame (Tool) 기준 회전
      // q_delta * q_cur : World Frame (Base) 기준 회전
      tf2::Quaternion q_target = q_cur * q_delta; 
      q_target.normalize();
      target.pose.orientation = tf2::toMsg(q_target);

      // ===== Cartesian Path Planning =====
      RCLCPP_INFO(this->get_logger(), "Computing Cartesian path...");
      
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target.pose);

      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0; // 0.0으로 설정하여 점프를 방지 (엄격하게) 혹은 상황에 맞춰 조절
      const double eef_step = 0.01;      // 1cm 단위로 보간
      
      double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

      RCLCPP_INFO(this->get_logger(), "Cartesian path computed. %.2f%% of path achievable.", fraction * 100.0);

      // RViz 시각화를 위해 Trajectory Display가 필요할 수 있음.
      // MoveGroupInterface는 computeCartesianPath 호출 시 내부적으로 시각화 토픽을 발행하지 않을 수 있으므로,
      // 실행(execute) 전에 사용자가 궤적을 확인할 수 있도록 로그로만 성공률을 안내합니다.

      if (fraction < 0.9) { // 90% 이상 도달 못하면 경고
          RCLCPP_WARN(this->get_logger(), "Path incomplete! Only %.2f%% generated. Proceed with caution.", fraction * 100.0);
      }

      // ===== 사용자 확인 =====
      bool execute_plan = false;
      while (rclcpp::ok()) {
          std::cout << "--> Execute Cartesian Path ('go'), or re-plan ('replan')? ";
          std::string confirm;
          std::cin >> confirm;

          if (confirm == "go") {
              execute_plan = true;
              break;
          }
          if (confirm == "replan") {
              RCLCPP_INFO(this->get_logger(), "User requested re-plan.");
              break;
          }
          RCLCPP_WARN(this->get_logger(), "Invalid input. Please type 'go' or 'replan'.");
      }

      if (!execute_plan) {
          continue;
      }

      // ===== Execution 단계 =====
      RCLCPP_INFO(this->get_logger(), "Executing Cartesian path...");
      
      // computeCartesianPath로 생성된 trajectory를 직접 실행
      moveit::core::MoveItErrorCode exec_result = move_group_interface_->execute(trajectory);

      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Execution succeeded.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Execution failed. code=%d", exec_result.val);
      }
    }
  }

private:
  void waitForTF(const std::string& target, const std::string& source, double timeout_sec)
  {
    auto start = now();
    RCLCPP_INFO(get_logger(), "Waiting for TF from '%s' to '%s'...", source.c_str(), target.c_str());
    while (rclcpp::ok()) {
      if (tf_buffer_->canTransform(target, source, tf2::TimePointZero)) {
        RCLCPP_INFO(get_logger(), "TF is ready.");
        return;
      }
      if ((now() - start).seconds() > timeout_sec) {
        RCLCPP_WARN(this->get_logger(), "TF not ready after %.1fs. Still waiting...", timeout_sec);
        start = now();
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  bool lookupEE(geometry_msgs::msg::TransformStamped& out, double timeout_sec)
  {
    try {
      out = tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero, tf2::durationFromSec(timeout_sec));
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
      return false;
    }
  }

  // Member variables
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string base_frame_;
  std::string ee_frame_;
  std::thread executor_thread_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RelativeMoveClient>();
  node->run(); // run() 내부 루프 실행
  rclcpp::shutdown();
  return 0;
}