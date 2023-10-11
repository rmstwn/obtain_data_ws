#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <iostream>

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
//#include "obtain_data/globals.hpp"
#include "crane_x7_comm.cpp"

using namespace std::chrono_literals;

// Frequency wave
float Fc_1[] = {0.1166, 0.1263, 0.1451, 0.1602, 0.1654, 0.1689, 0.1748};
float Fc_2[] = {0.2084, 0.2152, 0.2229, 0.2538, 0.2826, 0.2913, 0.2996};
float Fc_3[] = {0.3005, 0.3289, 0.3396, 0.3725, 0.4169, 0.5341, 0.5826};

float A_1[] = {65, 8, 65, 55, 65, 35, 65};
float A_2[] = {-65, -8, -65, -55, -65, -35, -65};
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

int main(int argc, char *argv[])
{
    std::cout << "Press any key to start (or press q to quit)\n";

    if (getchar() == ('q'))
        return 0;

    JointState jointState;

    double present_position[JOINT_NUM] = {0};
    double present_velocity[JOINT_NUM] = {0};
    double present_current[JOINT_NUM] = {0};

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

    uint8_t operating_mode[JOINT_NUM] = {OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION};

    if (initilizeCranex7(operating_mode))
    {
        return 1;
    }
    setCranex7TorqueState(TORQUE_ENABLE);
    // sleeps(5);

    safe_start(20);

    return 0;
}