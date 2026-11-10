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
        // 1. Plan Path
        RCLCPP_INFO(this->get_logger(), "Received command. Planning path...");
        geometry_msgs::msg::TransformStamped ee_tf;
        if (!lookupEE(ee_tf, 2.0)) return;

        geometry_msgs::msg::PoseStamped target_pose;
        calculate_target_pose(ee_tf, cmd, target_pose);

        MoveGroup::Goal plan_goal;
        plan_goal.request.group_name = "manipulator"; // Adjust if needed
        plan_goal.request.max_velocity_scaling_factor = 0.2; // 속도 조절
        plan_goal.request.max_acceleration_scaling_factor = 0.2;
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

        RCLCPP_INFO(this->get_logger(), "Planning successful. Type 'go' to execute.");

        // 2. Wait for user confirmation
        std::string confirm;
        std::cin >> confirm;
        if (confirm != "go") {
            RCLCPP_INFO(this->get_logger(), "Execution cancelled.");
            return;
        }

        // 3. Open Gripper
        RCLCPP_INFO(this->get_logger(), "Sending command to open gripper...");
        gripper_->open();
        // ROS 통신을 방해하지 않으면서 그리퍼가 열릴 시간을 1.5초간 대기
        RCLCPP_INFO(this->get_logger(), "Waiting for gripper to open...");
        this->get_clock()->sleep_for(std::chrono::milliseconds(1500));

        // 4. Execute arm movement
        RCLCPP_INFO(this->get_logger(), "Executing arm movement...");
        MoveGroup::Goal exec_goal = plan_goal;
        exec_goal.planning_options.plan_only = false;
        
        // 목표를 비동기적으로 보내고, goal_handle을 받을 때까지 스핀하며 기다립니다.
        auto exec_goal_handle_future = action_client_->async_send_goal(exec_goal); // <-- 변수 이름 변경 (exec_ 추가)
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), exec_goal_handle_future) != // <-- 변수 이름 변경
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send execution goal");
            return;
        }

        auto exec_goal_handle = exec_goal_handle_future.get(); // <-- 변수 이름 변경
        if (!exec_goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Execution goal was rejected by server");
            return;
        }

        // 최종 결과를 받을 때까지 스핀하며 기다립니다.
        auto exec_result_future = action_client_->async_get_result(exec_goal_handle); // <-- 변수 이름 변경 (exec_ 추가)
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), exec_result_future) != // <-- 변수 이름 변경
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get execution result");
            return;
        }

        auto exec_result = exec_result_future.get().result; // <-- 변수 이름 변경
        if (exec_result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Execution failed. Error code: %d", exec_result->error_code.val);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Arm movement successful.");

        // 5. Close Gripper
        RCLCPP_INFO(this->get_logger(), "Sending command to close gripper...");
        gripper_->close(cmd.stroke);
        // 그리퍼가 닫힐 시간을 1.5초간 기다립니다.
        RCLCPP_INFO(this->get_logger(), "Waiting for gripper to close...");
        this->get_clock()->sleep_for(std::chrono::milliseconds(1000));
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