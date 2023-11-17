#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>
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
#include "obtain_data/dynamics.hpp"
// #include "obtain_data/globals.hpp"
#include "crane_x7_comm.cpp"
#include "dynamics.cpp"

using namespace std::chrono_literals;

// Set up parameters
const std::string JOINTSTATE_TOPIC = "/joint_state";
const std::string ESTJOINTSTATE_TOPIC = "/est_joint_state";

// Frequency wave
float Fc_1[] = {0.1166, 0.1263, 0.1451, 0.1602, 0.1654, 0.1689, 0.1748};
float Fc_2[] = {0.2084, 0.2152, 0.2229, 0.2538, 0.2826, 0.2913, 0.2996};
float Fc_3[] = {0.3005, 0.3289, 0.3396, 0.3725, 0.4169, 0.5341, 0.5826};

float A_1[] = {55, 8, 55, 25, 55, 35, 55};
float A_2[] = {-55, -8, -55, -25, -55, -35, -55};
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
double th_rad[JOINT_NUM];

class DynamixelStatePublisher : public rclcpp::Node
{

public:
    DynamixelStatePublisher()
        : Node("joint_state_publisher")
    {
        // Setup pub/sub
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(JOINTSTATE_TOPIC, rclcpp::SystemDefaultsQoS());
        est_joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(ESTJOINTSTATE_TOPIC, rclcpp::SystemDefaultsQoS());
        timer_ = this->create_wall_timer(
            1ms, std::bind(&DynamixelStatePublisher::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr est_joint_pub_;

    // JointState jointState;

    int Waveflag = 0;
    int Readflag = 0;
    int Initflag = 0;

    double present_theta[JOINT_NUM] = {0};
    double present_angvel[JOINT_NUM] = {0};
    double present_torque[JOINT_NUM] = {0};
    double estimated_torque[JOINT_NUM] = {0};
    double error_torque[JOINT_NUM] = {0};

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
            // while(1)
            {
                for (i = 0; i < 7; i++)
                {
                    th_run[i] = th[i][j];
                    th_rad[i] = th[i][j] * (M_PI / 180);
                }

                // th_run[0] = 0
                // th_run[1] = 10;
                // th_run[2] = 0;
                // th_run[3] = 57.5;
                // th_run[4] = 0;
                // th_run[5] = 0;
                // th_run[6] = 0;

                // usleep(50000);
                setCranex7Angle(th_run);
                // getCranex7JointState(present_theta, present_angvel, present_torque);
                getCranex7Velocity(present_angvel);
                getCranex7Torque(present_torque);

                // present_torque[0] = 0;
                // present_torque[1] = 0;
                // present_torque[2] = 0;
                // present_torque[3] = 0;
                // present_torque[4] = 0;
                // present_torque[5] = 0;
                // present_torque[6] = 0;

                // for (int i = 0; i < JOINT_NUM; ++i)
                // {
                //     std::cout << "mot0[" << i << "] = " << mot0[i] << std::endl;
                // }

                // std::cout << j << " " << th_run[0] << " " << th_run[1] << " " << th_run[2] << " " << th_run[3] << " " << th_run[4] << " " << th_run[5] << " " << th_run[6] << " " << th_run[7] << std::endl;
                // std::cout << j << " " << present_angvel[0] << " " << present_angvel[1] << " " << present_angvel[2] << " " << present_angvel[3] << " " << present_angvel[4] << " " << present_angvel[5] << " " << present_angvel[6] << " " << present_angvel[7] << std::endl;
                // std::cout << "Feedback || " << j << " " << present_torque[0] << " " << present_torque[1] << " " << present_torque[2] << " " << present_torque[3] << " " << present_torque[4] << " " << present_torque[5] << " " << present_torque[6] << " " << present_torque[7] << std::endl;

                getCranex7EstimatedTorque(th_rad, present_angvel, present_torque, estimated_torque);

                // std::cout << j << " " << present_theta[0] << " " << present_theta[1] << " " << present_theta[2] << " " << present_theta[3] << " " << present_theta[4] << " " << present_theta[5] << " " << present_theta[6] << " " << present_theta[7] << std::endl;
                // std::cout << j << " " << present_torque[0] << " " << present_torque[1] << " " << present_torque[2] << " " << present_torque[3] << " " << present_torque[4] << " " << present_torque[5] << " " << present_torque[6] << " " << present_torque[7] << std::endl;

                // std::cout << "Estimated || " << j << " " << estimated_torque[0] << " " << estimated_torque[1] << " " << estimated_torque[2] << " " << estimated_torque[3] << " " << estimated_torque[4] << " " << estimated_torque[5] << " " << estimated_torque[6] << " " << estimated_torque[7] << std::endl;

                // Calculate Joint torque error between estimated and real
                for (int i = 0; i < 7; i++)
                {
                    error_torque[i] = abs(estimated_torque[i] - present_torque[i]);
                }

                std::cout << "Error || " << j << " " << error_torque[0] << " " << error_torque[1] << " " << error_torque[2] << " " << error_torque[3] << " " << error_torque[4] << " " << error_torque[5] << " " << error_torque[6] << " " << error_torque[7] << std::endl;

                // if (error_torque[0] >= 1.00 || error_torque[1] >= 1.00 || error_torque[2] >= 1.00 || error_torque[3] >= 1.00 || error_torque[4] >= 1.00 || error_torque[5] >= 1.00 || error_torque[6] >= 1.00 || error_torque[7] >= 1.00)
                // {
                //     break;
                // }
                if (*std::max_element(error_torque, error_torque + 8) >= 1.20)
                {
                    // Break if any element is greater than or equal to 1.00
                    std::cout << "Collision!!!!!!" << std::endl;
                    break;
                }

                // Create the messages we might publish Joint state data
                auto joint_msg = std::make_unique<sensor_msgs::msg::JointState>();
                auto est_joint_msg = std::make_unique<sensor_msgs::msg::JointState>();

                joint_msg->name.resize(JOINT_NUM);
                joint_msg->position.resize(JOINT_NUM);
                joint_msg->velocity.resize(JOINT_NUM);
                joint_msg->effort.resize(JOINT_NUM);

                est_joint_msg->name.resize(JOINT_NUM);
                est_joint_msg->position.resize(JOINT_NUM);
                est_joint_msg->velocity.resize(JOINT_NUM);
                est_joint_msg->effort.resize(JOINT_NUM);

                joint_msg->header.frame_id = "CraneX7";
                est_joint_msg->header.frame_id = "Estimated_CraneX7";

                for (int i = 0; i < JOINT_NUM; i++)
                {
                    joint_msg->name[i] = "cranex7_j" + std::to_string(i);

                    // joint_msg->position[i] = present_theta[i];
                    joint_msg->velocity[i] = present_angvel[i];
                    joint_msg->effort[i] = present_torque[i];
                }

                for (int i = 0; i < JOINT_NUM; i++)
                {
                    est_joint_msg->name[i] = "est_cranex7_j" + std::to_string(i);

                    // joint_msg->position[i] = present_theta[i];
                    // joint_msg->velocity[i] = present_angvel[i];
                    est_joint_msg->effort[i] = estimated_torque[i];
                }

                joint_msg->header.stamp = now();
                joint_pub_->publish(std::move(joint_msg));

                est_joint_msg->header.stamp = now();
                est_joint_pub_->publish(std::move(est_joint_msg));

                // usleep(1000);
                // usleep(100000);
            }

            //safe_start(20);

            Initflag = 1;
            closeCranex7Port();
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