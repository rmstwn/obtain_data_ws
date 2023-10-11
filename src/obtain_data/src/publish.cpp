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
// #include "obtain_data/globals.hpp"
#include "crane_x7_comm.cpp"

using namespace std::chrono_literals;

// Set up parameters
const std::string JOINTSTATE_TOPIC = "/joint_state";

// Frequency wave
float Fc_1[] = {0.1166, 0.1263, 0.1451, 0.1602, 0.1654, 0.1689, 0.1748};
float Fc_2[] = {0.2084, 0.2152, 0.2229, 0.2538, 0.2826, 0.2913, 0.2996};
float Fc_3[] = {0.3005, 0.3289, 0.3396, 0.3725, 0.4169, 0.5341, 0.5826};

float A_1[] = {65, 8, 65, 50, 65, 35, 65};
float A_2[] = {-65, -8, -65, -50, -65, -35, -65};
float A_3[JOINT_NUM];

int i;
int j;
int count = 1;

int Fs = 1;
float dt = 1 / Fs;
float StopTime = 0.25;
double t[MAX_DATA];

double th[JOINT_NUM][MAX_DATA];
double th_d[JOINT_NUM][MAX_DATA];
double th_dd[JOINT_NUM][MAX_DATA];

double th_run[JOINT_NUM];

class DynamixelStatePublisher : public rclcpp::Node
{

public:
    DynamixelStatePublisher()
        : Node("joint_state_publisher")
    {
        // Setup pub/sub
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(JOINTSTATE_TOPIC, rclcpp::SystemDefaultsQoS());
        timer_ = this->create_wall_timer(
            1ms, std::bind(&DynamixelStatePublisher::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    // JointState jointState;

    int Waveflag = 0;
    int Readflag = 0;
    int Initflag = 0;

    double present_theta[JOINT_NUM] = {0};
    double present_angvel[JOINT_NUM] = {0};
    double present_current[JOINT_NUM] = {0};

    void timer_callback()
    {

        if (Waveflag == 0)
        {
            // Parameters Signal
            for (i = 0; i < 7; i++)
            {
                A_3[i] = (-A_1[i] * Fc_1[i] - A_2[i] * Fc_2[i]) / Fc_3[i];
                // std::cout << A_3[i] << syd::endl;
            }

            t[0] = 0;
            for (i = 1; i <= MAX_DATA; i++)
            {
                t[i] = t[i - 1] + 0.1;
                // std::cout << t[i] << std::endl;
            }

            // Generate wave
            for (i = 0; i < 7; i++)
            {
                th[i][0] = 0;
                th_d[i][0] = 0;
                th_d[i][0] = 0;
            }

            for (j = 0; j < 7; j++)
            {
                for (i = 1; i <= MAX_DATA; i++)
                {
                    th[j][i] = A_1[j] * sin_deg(2 * M_PI * Fc_1[j] * t[i]) + A_2[j] * sin_deg(2 * M_PI * Fc_2[j] * t[i]) + A_3[j] * sin_deg(2 * M_PI * Fc_3[j] * t[i]);
                    th_d[j][i] = A_1[j] * Fc_1[j] * cos_deg(2 * M_PI * Fc_1[j] * t[i]) + A_2[j] * Fc_2[j] * cos_deg(2 * M_PI * Fc_2[j] * t[i]) + A_3[j] * Fc_3[j] * cos_deg(2 * M_PI * Fc_3[j] * t[i]);
                    th_d[j][i] = -(A_1[j] * pow(Fc_1[j], 2) * sin_deg(2 * M_PI * Fc_1[j] * t[i]) + A_2[j] * pow(Fc_2[j], 2) * sin_deg(2 * M_PI * Fc_2[j] * t[i]) + A_3[j] * pow(Fc_3[j], 2) * sin_deg(2 * M_PI * Fc_3[j] * t[i]));
                    // std::cout << th[j][i] << std::endl;
                }
            }

            Waveflag = 1;
        }

        if (Initflag == 0)
        {
            uint8_t operating_mode[JOINT_NUM] = {OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION};

            initilizeCranex7(operating_mode);
            setCranex7TorqueState(TORQUE_ENABLE);

            safe_start(20);

            for (j = 0; j < MAX_DATA; j++)
            {
                for (i = 0; i < 7; i++)
                {
                    th_run[i] = th[i][j];
                }

                // usleep(50000);
                setCranex7Angle(th_run);
                // getCranex7JointState(present_theta, present_angvel, present_current);
                getCranex7Current(present_current);

                // std::cout << j << " " << present_theta[0] << " " << present_theta[1] << " " << present_theta[2] << " " << present_theta[3] << " " << present_theta[4] << " " << present_theta[5] << " " << present_theta[6] << " " << present_theta[7] << std::endl;
                std::cout << j << " " << present_current[0] << " " << present_current[1] << " " << present_current[2] << " " << present_current[3] << " " << present_current[4] << " " << present_current[5] << " " << present_current[6] << " " << present_current[7] << std::endl;

                // Create the messages we might publish
                auto joint_msg = std::make_unique<sensor_msgs::msg::JointState>();

                joint_msg->name.resize(JOINT_NUM);
                joint_msg->position.resize(JOINT_NUM);
                joint_msg->velocity.resize(JOINT_NUM);
                joint_msg->effort.resize(JOINT_NUM);

                joint_msg->header.frame_id = "CraneX7";

                for (int i = 0; i < JOINT_NUM; i++)
                {
                    joint_msg->name[i] = "cranex7_j" + std::to_string(i);

                    joint_msg->position[i] = present_theta[i];
                    joint_msg->velocity[i] = present_angvel[i];
                    joint_msg->effort[i] = present_current[i];
                }

                joint_msg->header.stamp = now();
                joint_pub_->publish(std::move(joint_msg));
            }

            safe_start(20);

            Initflag = 1;
        }
    }
};

int main(int argc, char *argv[])
{
    // rclcpp::init(argc, argv);

    // // Create a multi-threaded executor
    // rclcpp::executors::MultiThreadedExecutor executor;

    // auto node = std::make_shared<DynamixelStatePublisher>();
    // executor.add_node(node);

    // // Spin the node with the executor
    // executor.spin();

    // rclcpp::shutdown();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelStatePublisher>());
    rclcpp::shutdown();
    return 0;

    return 0;
}