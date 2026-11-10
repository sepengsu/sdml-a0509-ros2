#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

// MoveIt 관련 헤더
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// 로봇 모델 로드를 위한 추가 헤더
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <streambuf>
#include <cmath> 
#include <yaml-cpp/yaml.h>

// 로컬 헤더
#include "a0509_control_cyw/gripper_controller.hpp"

// C++ 표준 라이브러리
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <optional>
#include <atomic>

// TCP를 통해 수신할 명령 구조체
struct MotionCommand {
    double dx, dy, dz, dqx, dqy, dqz, dqw, stroke;
};

// 파일 내용을 문자열로 읽어오는 헬퍼 함수
std::string load_file_to_string(const std::string& file_path) {
    std::ifstream file_stream(file_path);
    if (!file_stream) {
        throw std::runtime_error("Could not open file: " + file_path);
    }
    return std::string((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());
}

class ArmGripperCoordinator : public rclcpp::Node, public std::enable_shared_from_this<ArmGripperCoordinator>
{
public:
    ArmGripperCoordinator() : Node("arm_gripper_coordinator")
    {
        // ROS 파라미터 선언
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        ee_frame_ = this->declare_parameter<std::string>("ee_frame", "tool0");
        std::string gripper_port = this->declare_parameter<std::string>("gripper_port", "/dev/ttyACM0");
        std::string planning_group = this->declare_parameter<std::string>("planning_group", "manipulator");
        std::string model = this->declare_parameter<std::string>("model", "a0509");

        // TF 버퍼 및 리스너 초기화
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // MoveIt! 파라미터 로드 및 MoveGroupInterface 초기화
        RCLCPP_INFO(this->get_logger(), "Loading robot description for model: %s", model.c_str());
        
        std::string urdf_path, srdf_path;
        try {
            std::string urdf_pkg_path = ament_index_cpp::get_package_share_directory("dsr_description2");
            std::string srdf_pkg_path = ament_index_cpp::get_package_share_directory("dsr_moveit_config_" + model);
            urdf_path = urdf_pkg_path + "/urdf/" + model + ".urdf";
            srdf_path = srdf_pkg_path + "/config/dsr.srdf";
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Required package not found: %s. Is the Doosan ROS2 workspace sourced?", e.what());
            rclcpp::shutdown();
            return;
        }

        std::string urdf_content, srdf_content;
        try {
            urdf_content = load_file_to_string(urdf_path);
            srdf_content = load_file_to_string(srdf_path);
        } catch (const std::runtime_error& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to read robot description files: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        node_options.parameter_overrides({
            {"robot_description", urdf_content},
            {"robot_description_semantic", srdf_content}
        });

        auto move_group_node = rclcpp::Node::make_shared("move_group_interface_node", node_options);
        
        executor_thread_ = std::thread([move_group_node]() {
            rclcpp::spin(move_group_node);
        });

        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, planning_group);
        move_group_interface_->setEndEffectorLink(ee_frame_);
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized successfully.");

        // 그리퍼 초기화
        gripper_ = std::make_unique<GripperController>(gripper_port, this->get_logger());
        if (!gripper_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize gripper. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // 별도 스레드에서 TCP 서버 시작
        server_thread_ = std::thread(&ArmGripperCoordinator::start_server, this);
    }

    ~ArmGripperCoordinator() {
        stop_flag_.store(true);
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
        if (executor_thread_.joinable()){
            executor_thread_.join();
        }
    }

    void run()
    {
        RCLCPP_INFO(this->get_logger(), "Ready. Waiting for command from Windows PC...");

        while (rclcpp::ok())
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait_for(lock, std::chrono::milliseconds(100), [this]{ return command_.has_value(); });

            if (!rclcpp::ok()) break;
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
        socklen_t addrlen = sizeof(address);

        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(9900);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "TCP Bind failed");
            return;
        }
        if (listen(server_fd, 3) < 0) {
            RCLCPP_ERROR(this->get_logger(), "TCP Listen failed");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "TCP Server listening on port 9900");

        while (rclcpp::ok() && !stop_flag_.load()) {
            new_socket = accept(server_fd, (struct sockaddr *)&address, &addrlen);
            if (new_socket < 0) {
                if(!rclcpp::ok() || stop_flag_.load()) break;
                RCLCPP_ERROR(this->get_logger(), "TCP Accept failed");
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "WIN PC connected.");
            
            char buffer[1024] = {0};
            while (rclcpp::ok() && !stop_flag_.load()) {
                int valread = read(new_socket, buffer, 1024);
                if (valread <= 0) {
                    RCLCPP_WARN(this->get_logger(), "WIN PC disconnected.");
                    break;
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
        }
        close(server_fd);
        RCLCPP_INFO(this->get_logger(), "TCP Server shut down.");
    }

    void process_command(const MotionCommand& cmd)
    {
        // 1. 최종 목표(target_pose) 및 사전 파지(pre_grasp_pose) 자세 계산
        RCLCPP_INFO(this->get_logger(), "Received command. Calculating poses...");
        geometry_msgs::msg::TransformStamped ee_tf;
        if (!lookupEE(ee_tf, 2.0)) return;

        geometry_msgs::msg::PoseStamped target_pose;
        calculate_target_pose(ee_tf, cmd, target_pose);

        geometry_msgs::msg::PoseStamped pre_grasp_pose;
        tf2::Quaternion q_target;
        tf2::fromMsg(target_pose.pose.orientation, q_target);
        tf2::Vector3 approach_vector_in_base = tf2::quatRotate(q_target, tf2::Vector3(0, 0, 1));
        
        pre_grasp_pose.header = target_pose.header;
        pre_grasp_pose.pose.orientation = target_pose.pose.orientation;
        pre_grasp_pose.pose.position.x = target_pose.pose.position.x + 0.1 * approach_vector_in_base.x();
        pre_grasp_pose.pose.position.y = target_pose.pose.position.y + 0.1 * approach_vector_in_base.y();
        pre_grasp_pose.pose.position.z = target_pose.pose.position.z + 0.1 * approach_vector_in_base.z();

        RCLCPP_INFO(this->get_logger(), "Poses calculated. Type 'go' to execute.");

        // 2. 사용자 실행 확인
        std::string confirm;
        std::cin >> confirm;
        if (confirm != "go" || !rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Execution cancelled.");
            return;
        }

        // 3. 작업 순서 실행
        move_group_interface_->setMaxVelocityScalingFactor(0.5);
        move_group_interface_->setMaxAccelerationScalingFactor(0.5);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success;

        // // 1단계: 사전 파지 자세로 이동 (조인트 공간 이동)
        // RCLCPP_INFO(this->get_logger(), "Moving to pre-grasp pose...");
        // move_group_interface_->setPoseTarget(pre_grasp_pose);
        // success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        // if (!success) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to plan for pre-grasp pose. Aborting.");
        //     return;
        // }
        // if (move_group_interface_->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to execute pre-grasp move. Aborting.");
        //     return;
        // }

        // 1단계: 사전 파지 자세로 이동 (카테시안 직선 이동)
        RCLCPP_INFO(this->get_logger(), "Executing Cartesian path to pre-grasp pose...");
        std::vector<geometry_msgs::msg::Pose> pre_grasp_waypoints;
        pre_grasp_waypoints.push_back(pre_grasp_pose.pose);

        moveit_msgs::msg::RobotTrajectory pre_grasp_trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_interface_->computeCartesianPath(pre_grasp_waypoints, eef_step, jump_threshold, pre_grasp_trajectory);

        if (fraction < 0.9) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan Cartesian path for pre-grasp (only %.2f%% achieved). Aborting.", fraction * 100.0);
            return;
        }
        if (move_group_interface_->execute(pre_grasp_trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute Cartesian path for pre-grasp. Aborting.");
            return;
        }

        // 2단계: 1차 그리퍼 닫기
        RCLCPP_INFO(this->get_logger(), "Performing initial grasp...");
        gripper_->close(cmd.stroke + 20.0);
        this->get_clock()->sleep_for(std::chrono::milliseconds(1500));

        // 3단계: 최종 목표 자세로 이동 (카테시안 직선 이동)
        RCLCPP_INFO(this->get_logger(), "Executing Cartesian path to final grasp pose...");
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        fraction = move_group_interface_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        if (fraction < 0.9) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan Cartesian path (only %.2f%% achieved). Aborting.", fraction * 100.0);
            return;
        }
        if (move_group_interface_->execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute Cartesian path. Aborting.");
            return;
        }

        // 4단계: 최종 그리퍼 닫기
        RCLCPP_INFO(this->get_logger(), "Performing final grasp...");
        gripper_->close(cmd.stroke - 10.0);
        this->get_clock()->sleep_for(std::chrono::milliseconds(1500));

        // 5단계: 물체 들어올리기 (카테시안 직선 이동)
        RCLCPP_INFO(this->get_logger(), "Lifting object...");
        
        // 현재 자세에서 월드 Z축 방향으로 10cm 위로 이동하는 자세를 목표로 설정
        geometry_msgs::msg::Pose lift_pose = move_group_interface_->getCurrentPose().pose;
        const double lift_distance = 0.2; // 20cm
        lift_pose.position.z += lift_distance;

        std::vector<geometry_msgs::msg::Pose> lift_waypoints;
        lift_waypoints.push_back(lift_pose);

        moveit_msgs::msg::RobotTrajectory lift_trajectory;
        fraction = move_group_interface_->computeCartesianPath(lift_waypoints, 0.01, 0.0, lift_trajectory);

        if (fraction < 0.9) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan lift path (only %.2f%% achieved).", fraction * 100.0);
        } else {
            if (move_group_interface_->execute(lift_trajectory) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Lift successful.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute lift path.");
            }
        }

        // 6단계: Home Position으로 이동 (조인트 공간 이동)
        RCLCPP_INFO(this->get_logger(), "Moving to Home Position...");
        
        // 목표 조인트 각도 (degrees)
        // Top down
        // std::vector<double> joint_group_positions_deg = {0.0, 0.0, 60.0, 0.0, 120.0, 45.0};
        // Slightly tilted
        std::vector<double> joint_group_positions_deg = {4.0, -32.0, 85.0, -4.0, 115.0, 45.0};
        std::vector<double> joint_group_positions_rad;
        // 각도를 라디안으로 변환
        for(const auto& deg : joint_group_positions_deg) {
            joint_group_positions_rad.push_back(deg * M_PI / 180.0);
        }

        move_group_interface_->setJointValueTarget(joint_group_positions_rad);
        success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan for Home Position.");
        } else {
            if (move_group_interface_->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute move to Home Position.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Move to Home Position successful.");
            }
        }

        // 7단계: 그리퍼 열기
        RCLCPP_INFO(this->get_logger(), "Opening gripper...");
        gripper_->open();
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
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string base_frame_, ee_frame_;

    std::unique_ptr<GripperController> gripper_;
    
    std::thread server_thread_;
    std::thread executor_thread_;
    std::atomic<bool> stop_flag_{false};
    std::mutex mtx_;
    std::condition_variable cv_;
    std::optional<MotionCommand> command_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmGripperCoordinator>();
    if (rclcpp::ok()) {
        node->run();
    }
    rclcpp::shutdown();
    return 0;
}