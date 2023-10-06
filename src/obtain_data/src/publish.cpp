#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_velocity.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_velocity.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_current.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_current.hpp"

#include "obtain_data/dynamixel_node.hpp"
#include "obtain_data/crane_x7_comm.hpp"
#include "obtain_data/globals.hpp"
#include "crane_x7_comm.cpp"

using namespace std::chrono_literals;

// Set up parameters
const std::string JOINTSTATE_TOPIC = "/joint_state";

class DynamixelStatePublisher : public rclcpp::Node
{
public:
    DynamixelStatePublisher()
        : Node("joint_state_publisher"), count_(0)
    {
        // Setup pub/sub
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(JOINTSTATE_TOPIC, rclcpp::SystemDefaultsQoS());
        timer_ = this->create_wall_timer(
            50ms, std::bind(&DynamixelStatePublisher::timer_callback, this));
        main_timer_ = this->create_wall_timer(
            50ms, std::bind(&DynamixelStatePublisher::run_main_timer, this));
    }

private:
    JointState jointState;
    int flag = 1;
    int Readflag = 0;

    double present_theta[JOINT_NUM] = {0};
    double present_angvel[JOINT_NUM] = {0};
    double present_current[JOINT_NUM] = {0};

    void timer_callback()
    {
        // Create the messages we might publish
        auto joint_msg = std::make_unique<sensor_msgs::msg::JointState>();

        joint_msg->name.resize(JOINT_NUM);
        joint_msg->position.resize(JOINT_NUM);
        joint_msg->velocity.resize(JOINT_NUM);
        joint_msg->effort.resize(JOINT_NUM);

        joint_msg->header.frame_id = "CraneX7";

        if (Readflag == 1)
        {
            getCranex7JointState(present_theta, present_angvel, present_current);
        }

        for (int i = 0; i < JOINT_NUM; i++)
        {
            joint_msg->name[i] = "cranex7_j" + std::to_string(i);
            // joint_msg->position[i] = jointState.getTheta(i);
            // joint_msg->velocity[i] = jointState.getAngVel(i);
            // joint_msg->effort[i] = jointState.getCurrent(i);

            joint_msg->position[i] = present_theta[i];
            joint_msg->velocity[i] = present_angvel[i];
            joint_msg->effort[i] = present_current[i];
        }

        joint_msg->header.stamp = now();

        joint_pub_->publish(std::move(joint_msg));
    }

    int run_main_timer()
    {
        if (flag == 1)
        {
            std::cout << "Press any key to start (or press q to quit)\n";

            if (getchar() == ('q'))
            {
                flag = 0;
                return 0;
            }

            uint8_t operating_mode[JOINT_NUM] = {OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION};

            if (initilizeCranex7(operating_mode))
            {
                return 1;
            }
            setCranex7TorqueState(TORQUE_ENABLE);
            // sleeps(5);

            Readflag = 1;

            safe_start(20);
        }

        flag = 0;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelStatePublisher>());
    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(obtain_data::DynamixelStatePublisher)