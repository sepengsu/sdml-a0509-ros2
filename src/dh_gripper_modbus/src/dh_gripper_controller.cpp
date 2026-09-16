#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>
#include <iostream>
#include <unistd.h>
#include <string>
#include <algorithm>
#include <vector>

class DHGripper : public rclcpp::Node {
public:
    DHGripper() : Node("dh_gripper_node") {
        modbus_ = modbus_new_rtu("/dev/ttyACM0", 115200, 'N', 8, 1); // 동작 안 할 시 포트 확인 필요
        modbus_set_slave(modbus_, 1);

        if (modbus_connect(modbus_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Unable to connect to gripper");
            modbus_free(modbus_);
            exit(1);
        }

        RCLCPP_INFO(this->get_logger(), "Modbus connected to gripper");

        initializeGripper();
        inputLoop();
    }

    void initializeGripper() {
        modbus_write_register(modbus_, 0x0100, 0x01);  // Init command
        usleep(500000);  // 0.5s wait
        RCLCPP_INFO(this->get_logger(), "Gripper initialized.");
    }

    void setForce(int force) {
        force = std::clamp(force, 20, 1000); // 100 or 255
        modbus_write_register(modbus_, 0x0101, force);
        RCLCPP_INFO(this->get_logger(), "Set force: %d%%", force);
    }

    void setPosition(int position) {
        position = std::clamp(position, 0, 1000);
        modbus_write_register(modbus_, 0x0103, position);
        RCLCPP_INFO(this->get_logger(), "Set position: %d‰", position);

        usleep(1000000);  // wait 1s for motion

        // Read holding status (0x0120)
        uint16_t status = 0;
        int rc = modbus_read_registers(modbus_, 0x0120, 1, &status);
        if (rc == 1) {
            bool object_held = (status & 0x01);
            if (object_held) {
                RCLCPP_INFO(this->get_logger(), "Grip Success.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Grip Failure.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No signal from Gripper. Check conn..");
        }
    }

    int convertDistanceToPosition(double mm) {
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
        return 0;
    }

    void inputLoop() {
        while (rclcpp::ok()) {
            std::string input;
            std::cout << "\nEnter force (20~100%) and opening (mm) [0~145], separated by space: ";
            std::getline(std::cin, input);
            if (input == "q") break;

            int f = 0;
            double mm = 0.0;
            try {
                sscanf(input.c_str(), "%d %lf", &f, &mm);
                int pos = convertDistanceToPosition(mm);
                setForce(f);
                setPosition(pos);
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Invalid input. Please try again.");
            }
        }
    }

    ~DHGripper() {
        modbus_close(modbus_);
        modbus_free(modbus_);
        RCLCPP_INFO(this->get_logger(), "Gripper connection closed.");
    }

private:
    modbus_t *modbus_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DHGripper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
