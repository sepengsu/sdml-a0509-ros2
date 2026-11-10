#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/kinematic_constraints/utils.h>
#include "a0509_control_cyw/gripper_controller.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <optional>

// Struct to hold the command received from TCP
struct MotionCommand {
    double dx, dy, dz, dqx, dqy, dqz, dqw, stroke;
};

class ArmGripperCoordinator : public rclcpp::Node
{
public:
    using MoveGroup = moveit_msgs::action::MoveGroup;
    using GoalHandleMoveGroup = rclcpp_action::ClientGoalHandle<MoveGroup>;

    ArmGripperCoordinator() : Node("arm_gripper_coordinator")
    {
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        ee_frame_ = this->declare_parameter<std::string>("ee_frame", "tool0");
        std::string gripper_port = this->declare_parameter<std::string>("gripper_port", "/dev/ttyACM0");

        // Initialize MoveIt components
        action_client_ = rclcpp_action::create_client<MoveGroup>(this, "move_action");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize Gripper
        gripper_ = std::make_unique<GripperController>(gripper_port, this->get_logger());
        if (!gripper_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize gripper. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Start TCP server in a separate thread
        server_thread_ = std::thread(&ArmGripperCoordinator::start_server, this);
    }

    ~ArmGripperCoordinator() {
        if (server_thread_.joinable()) {
            // To-Do: Need a clean way to shut down the server thread.
            // For now, it will be terminated when the process exits.
            server_thread_.detach();
        }
    }

    void run()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for TF and MoveGroup action server...");
        waitForTF(base_frame_, ee_frame_, 10.0);
        action_client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Ready. Waiting for command from Windows PC...");

        while (rclcpp::ok())
        {
            std::unique_lock<std::mutex> lock(mtx_);
            // 100ms 마다 깨어나서 rclcpp::ok() 상태와 command 수신 여부를 함께 확인
            cv_.wait_for(lock, std::chrono::milliseconds(100), [this]{ return command_.has_value(); });

            // Ctrl+C 등으로 종료 신호가 오면 루프를 빠져나감
            if (!rclcpp::ok()) break;

            // command가 없으면 다시 대기
            if (!command_.has_value()) continue;

            MotionCommand cmd = command_.value();
            command_.reset();
            lock.unlock();
            
            process_command(cmd);
        }
    }

private:

    bool execute_move_to_pose(const geometry_msgs::msg::PoseStamped& pose, const std::string& description)
    {
        RCLCPP_INFO(this->get_logger(), "Executing move to '%s' pose...", description.c_str());
        MoveGroup::Goal move_goal;
        move_goal.request.group_name = "manipulator";
        move_goal.request.max_velocity_scaling_factor = 0.15; // 안전을 위해 속도를 10%로 낮춤
        move_goal.request.max_acceleration_scaling_factor = 0.15;
        move_goal.planning_options.plan_only = false; // 계획과 실행을 동시에
        move_goal.request.goal_constraints.push_back(
            kinematic_constraints::constructGoalConstraints(ee_frame_, pose, 0.01, 0.01)
        );

        auto goal_handle_future = action_client_->async_send_goal(move_goal);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal for '%s'", description.c_str());
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal for '%s' was rejected", description.c_str());
            return false;
        }

        auto result_future = action_client_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get result for '%s'", description.c_str());
            return false;
        }

        auto result = result_future.get().result;
        if (result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Move to '%s' failed. Error code: %d", description.c_str(), result->error_code.val);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Move to '%s' successful.", description.c_str());
        return true;
    }

    void start_server()
    {
        int server_fd, new_socket;
        struct sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);

        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return;
        }

        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
            RCLCPP_ERROR(this->get_logger(), "setsockopt failed");
            return;
        }
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(9900);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            return;
        }
        if (listen(server_fd, 3) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Listen failed");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "TCP Server listening on port 9900");

        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Accept failed");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "WIN PC connected.");
        
        char buffer[1024] = {0};
        while (rclcpp::ok()) {
            int valread = read(new_socket, buffer, 1024);
            if (valread <= 0) {
                 RCLCPP_WARN(this->get_logger(), "WIN PC disconnected. Waiting for new connection...");
                 close(new_socket);
                 if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Accept failed");
                    break;
                 }
                 RCLCPP_INFO(this->get_logger(), "WIN PC re-connected.");
                 continue;
            }
            
            MotionCommand cmd;
            int items = sscanf(buffer, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                   &cmd.dx, &cmd.dy, &cmd.dz, &cmd.dqx, &cmd.dqy, &cmd.dqz, &cmd.dqw, &cmd.stroke);
            
            if (items == 8) {
                std::lock_guard<std::mutex> lock(mtx_);
                command_ = cmd;
                cv_.notify_one();
            } else {
                RCLCPP_WARN(this->get_logger(), "Received malformed data: %s", buffer);
            }
            memset(buffer, 0, sizeof(buffer));
        }
        close(new_socket);
        close(server_fd);
    }

    void process_command(const MotionCommand& cmd)
    {
        // ----- 1. 최종 목표 자세(target_pose) 및 사전 파지 자세(pre_grasp_pose) 계산 -----
        RCLCPP_INFO(this->get_logger(), "Received command. Calculating poses...");
        geometry_msgs::msg::TransformStamped ee_tf;
        if (!lookupEE(ee_tf, 2.0)) return;

        geometry_msgs::msg::PoseStamped target_pose;
        calculate_target_pose(ee_tf, cmd, target_pose);

        // 사전 파지 자세 계산 (+0.1m along the tool's Z-axis from the final pose)
        geometry_msgs::msg::PoseStamped pre_grasp_pose;
        tf2::Quaternion q_target;
        tf2::fromMsg(target_pose.pose.orientation, q_target);
        
        // Tool Z축 벡터 [0, 0, 1]을 목표 방향으로 회전시켜 월드 기준의 접근 벡터를 구함
        tf2::Vector3 approach_vector_in_base = tf2::quatRotate(q_target, tf2::Vector3(0, 0, 1));
        
        pre_grasp_pose.header = target_pose.header;
        pre_grasp_pose.pose.orientation = target_pose.pose.orientation;
        pre_grasp_pose.pose.position.x = target_pose.pose.position.x + 0.1 * approach_vector_in_base.x();
        pre_grasp_pose.pose.position.y = target_pose.pose.position.y + 0.1 * approach_vector_in_base.y();
        pre_grasp_pose.pose.position.z = target_pose.pose.position.z + 0.1 * approach_vector_in_base.z();

        RCLCPP_INFO(this->get_logger(), "Poses calculated. For visualization, plan a path to the final pose.");

        // ----- 2. 사용자 확인을 위해 RViz에 최종 경로만 계획하여 보여주기 -----
        MoveGroup::Goal plan_goal;
        plan_goal.request.group_name = "manipulator";
        plan_goal.planning_options.plan_only = true;
        plan_goal.request.goal_constraints.push_back(
            kinematic_constraints::constructGoalConstraints(ee_frame_, target_pose, 0.01, 0.01)
        );

        auto goal_handle_future = action_client_->async_send_goal(plan_goal);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send planning goal");
            return;
        }
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Planning goal was rejected by server");
            return;
        }

        auto result_future = action_client_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get planning result");
            return;
        }
        
        auto plan_result = result_future.get().result;
        if (plan_result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Planning failed. Error code: %d", plan_result->error_code.val);
            return;
        }

        // ----- 3. 사용자 실행 확인 -----
        std::string confirm;
        std::cin >> confirm;
        if (confirm != "go") {
            RCLCPP_INFO(this->get_logger(), "Execution cancelled.");
            return;
        }

        // ----- 4. 새로운 작업 순서 실행 -----
        // 1단계: 사전 파지 자세로 이동
        if (!execute_move_to_pose(pre_grasp_pose, "pre-grasp")) {
            RCLCPP_ERROR(this->get_logger(), "Aborting sequence due to pre-grasp move failure.");
            return;
        }

        // 2단계: 1차 그리퍼 닫기 (stroke + 10mm)
        RCLCPP_INFO(this->get_logger(), "Performing initial grasp...");
        gripper_->close(cmd.stroke + 10.0);
        this->get_clock()->sleep_for(std::chrono::milliseconds(1500));

        // 3단계: 최종 목표 자세로 이동
        if (!execute_move_to_pose(target_pose, "final grasp")) {
            RCLCPP_ERROR(this->get_logger(), "Aborting sequence due to final approach failure.");
            return;
        }

        // 4단계: 최종 그리퍼 닫기 (stroke - 5mm)
        RCLCPP_INFO(this->get_logger(), "Performing final grasp...");
        gripper_->close(cmd.stroke - 5.0);
        this->get_clock()->sleep_for(std::chrono::milliseconds(1500));

        RCLCPP_INFO(this->get_logger(), "Workflow complete. Waiting for next command.");
    }
    
    void calculate_target_pose(const geometry_msgs::msg::TransformStamped& ee_tf, const MotionCommand& cmd, geometry_msgs::msg::PoseStamped& target)
    {
        target.header.frame_id = base_frame_;
        target.header.stamp = this->now();

        tf2::Quaternion q_cur;
        tf2::fromMsg(ee_tf.transform.rotation, q_cur);
        tf2::Vector3 p_cur(ee_tf.transform.translation.x, ee_tf.transform.translation.y, ee_tf.transform.translation.z);
        tf2::Vector3 p_delta(cmd.dx, cmd.dy, cmd.dz);
        tf2::Vector3 p_delta_in_base = tf2::quatRotate(q_cur, p_delta);
        tf2::Vector3 p_target_vec = p_cur + p_delta_in_base;

        target.pose.position.x = p_target_vec.x();
        target.pose.position.y = p_target_vec.y();
        target.pose.position.z = p_target_vec.z();

        tf2::Quaternion q_delta(cmd.dqx, cmd.dqy, cmd.dqz, cmd.dqw);
        if (q_delta.length2() == 0.0) q_delta.setValue(0,0,0,1);
        tf2::Quaternion q_target = q_cur * q_delta;
        q_target.normalize();
        target.pose.orientation = tf2::toMsg(q_target);
    }

    void waitForTF(const std::string& target, const std::string& source, double timeout_sec)
    {
        auto start = this->now();
        RCLCPP_INFO(this->get_logger(), "Waiting for TF from '%s' to '%s'...", source.c_str(), target.c_str());
        while (rclcpp::ok()) {
            if (tf_buffer_->canTransform(target, source, tf2::TimePointZero)) {
                RCLCPP_INFO(this->get_logger(), "TF is ready.");
                return;
            }
            if ((this->now() - start).seconds() > timeout_sec) {
                RCLCPP_WARN(this->get_logger(), "TF not ready after %.1fs. Still waiting...", timeout_sec);
                start = this->now();
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            rclcpp::spin_some(this->get_node_base_interface());
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

    // Class members
    rclcpp_action::Client<MoveGroup>::SharedPtr action_client_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string base_frame_, ee_frame_;

    std::unique_ptr<GripperController> gripper_;
    
    std::thread server_thread_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::optional<MotionCommand> command_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmGripperCoordinator>();
    node->run();
    rclcpp::shutdown();
    return 0;
}