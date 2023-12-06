#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <array>

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

using namespace std;

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
double th_rad[JOINT_NUM - 1];

double forces[6];

int main(int argc, char *argv[])
{
    // std::cout << "Press any key to start (or press q to quit)\n";

    // if (getchar() == ('q'))
    //     return 0;

    // Feedback variables
    double present_theta[JOINT_NUM] = {0};
    double present_angvel[JOINT_NUM] = {0};
    double present_torque[JOINT_NUM - 1] = {0};
    double estimated_torque[JOINT_NUM] = {0};
    double error_torque[JOINT_NUM - 1] = {0};

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // // Create log file
    // std::ofstream data;
    // data.open("data.csv");

    // // Parameters Signal
    // for (i = 0; i < 7; i++)
    // {
    //     A_3[i] = (-A_1[i] * Fc_1[i] - A_2[i] * Fc_2[i]) / Fc_3[i];
    //     // std::cout << A_3[i] << syd::endl;
    // }

    // t[0] = 0;
    // for (i = 1; i <= MAX_DATA; i++)
    // {
    //     t[i] = t[i - 1] + 0.1;
    //     // std::cout << t[i] << std::endl;
    // }

    // // Generate wave
    // for (i = 0; i < 7; i++)
    // {
    //     th[i][0] = 0;
    //     th_d[i][0] = 0;
    //     th_d[i][0] = 0;
    // }

    // for (j = 0; j < 7; j++)
    // {
    //     for (i = 1; i <= MAX_DATA; i++)
    //     {
    //         th[j][i] = A_1[j] * sin_deg(2 * M_PI * Fc_1[j] * t[i]) + A_2[j] * sin_deg(2 * M_PI * Fc_2[j] * t[i]) + A_3[j] * sin_deg(2 * M_PI * Fc_3[j] * t[i]);
    //         th_d[j][i] = A_1[j] * Fc_1[j] * cos_deg(2 * M_PI * Fc_1[j] * t[i]) + A_2[j] * Fc_2[j] * cos_deg(2 * M_PI * Fc_2[j] * t[i]) + A_3[j] * Fc_3[j] * cos_deg(2 * M_PI * Fc_3[j] * t[i]);
    //         th_d[j][i] = -(A_1[j] * pow(Fc_1[j], 2) * sin_deg(2 * M_PI * Fc_1[j] * t[i]) + A_2[j] * pow(Fc_2[j], 2) * sin_deg(2 * M_PI * Fc_2[j] * t[i]) + A_3[j] * pow(Fc_3[j], 2) * sin_deg(2 * M_PI * Fc_3[j] * t[i]));
    //         // std::cout << th[j][i] << std::endl;
    //     }
    // }

    // for (i = 1; i <= MAX_DATA; i++)
    // {
    //     th[7][i] = 0;
    //     th_d[7][i] = 0;
    //     th_d[7][i] = 0;
    //     // std::cout << th[j][i] << std::endl;
    // }

    // uint8_t operating_mode[JOINT_NUM] = {OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION};

    // if (initilizeCranex7(operating_mode))
    // {
    //     return 1;
    // }
    // setCranex7TorqueState(TORQUE_ENABLE);
    // // sleeps(5);

    // safe_start(20);

    // auto start = std::chrono::high_resolution_clock::now();

    // for (j = 0; j < MAX_DATA; j++)
    // {
    //     for (i = 0; i < 7; i++)
    //     {
    //         // need to convert joint 4 57.5 to -57.5 to 0 115
    //         if (i == 3)
    //         {
    //             th_run[i] = map_range(th[i][j], 57.5, -57.5, 0, -115);
    //             th_rad[i] = th_run[i] * (M_PI / 180);
    //         }
    //         else
    //         {
    //             th_run[i] = th[i][j];
    //             th_rad[i] = th_run[i] * (M_PI / 180);
    //         }
    //     }

    //     // std::cout << j << " " << th_run[0] << " " << th_run[1] << " " << th_run[2] << " " << th_run[3] << " " << th_run[4] << " " << th_run[5] << " " << th_run[6] << " " << th_run[7] << std::endl;
    //     std::cout << j << " " << th_run[3] << std::endl;

    //     // usleep(50000);
    //     setCranex7Angle(th_run);
    //     // getCranex7JointState(present_position, present_velocity, present_torque);
    //     getCranex7Velocity(present_angvel);
    //     getCranex7Torque(present_torque);

    //     // getCranex7EstimatedTorque(th_rad, present_angvel, present_torque, estimated_torque);
    //     // std::cout << "Estimated Torque || " << j << " " << estimated_torque[0] << " " << estimated_torque[1] << " " << estimated_torque[2] << " " << estimated_torque[3] << " " << estimated_torque[4] << " " << estimated_torque[5] << " " << estimated_torque[6] << " " << estimated_torque[7] << std::endl;

    //     // Calculate Joint torque error between estimated and real for (int i = 0; i < 7; i++)
    //     // for (int i = 0; i < 7; i++)
    //     // {
    //     //     error_torque[i] = abs(estimated_torque[i] - present_torque[i]);
    //     // }

    //     // getCranex7EstimatedExtForces(th_rad, error_torque, forces);
    //     // std::cout << "Estimated forces || " << j << " " << forces[0] << " " << forces[1] << " " << forces[2] << " " << forces[3] << " " << forces[4] << " " << forces[5] << std::endl;

    //     auto end = std::chrono::high_resolution_clock::now();

    //     double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    //     time_taken *= 1e-9;

    //     // data << time_taken << ","
    //     //      << th[0][j] * (M_PI / 180) << ","
    //     //      << th[1][j] * (M_PI / 180) << ","
    //     //      << th[2][j] * (M_PI / 180) << ","
    //     //      << map_range(th[3][j], 57.5, -57.5, 0, 115) * (M_PI / 180) << ","
    //     //      << th[4][j] * (M_PI / 180) << ","
    //     //      << th[5][j] * (M_PI / 180) << ","
    //     //      << th[6][j] * (M_PI / 180) << ","
    //     //      << (double)present_angvel[0] << ","
    //     //      << (double)present_angvel[1] << ","
    //     //      << (double)present_angvel[2] << ","
    //     //      << (double)present_angvel[3] << ","
    //     //      << (double)present_angvel[4] << ","
    //     //      << (double)present_angvel[5] << ","
    //     //      << (double)present_angvel[6] << ","
    //     //      << (double)present_torque[0] << ","
    //     //      << (double)present_torque[1] << ","
    //     //      << (double)present_torque[2] << ","
    //     //      << (double)present_torque[3] << ","
    //     //      << (double)present_torque[4] << ","
    //     //      << (double)present_torque[5] << ","
    //     //      << (double)present_torque[6] << std::endl;

    //     data << time_taken << ","
    //          << th_rad[0] << ","
    //          << th_rad[1] << ","
    //          << th_rad[2] << ","
    //          << th_rad[3] << ","
    //          << th_rad[4] << ","
    //          << th_rad[5] << ","
    //          << th_rad[6] << ","
    //          << (double)present_angvel[0] << ","
    //          << (double)present_angvel[1] << ","
    //          << (double)present_angvel[2] << ","
    //          << (double)present_angvel[3] << ","
    //          << (double)present_angvel[4] << ","
    //          << (double)present_angvel[5] << ","
    //          << (double)present_angvel[6] << ","
    //          << (double)present_torque[0] << ","
    //          << (double)present_torque[1] << ","
    //          << (double)present_torque[2] << ","
    //          << (double)present_torque[3] << ","
    //          << (double)present_torque[4] << ","
    //          << (double)present_torque[5] << ","
    //          << (double)present_torque[6] << std::endl;

    //     // std::cout << j << " " << present_torque[0] << " " << present_torque[1] << " " << present_torque[2] << " " << present_torque[3] << " " << present_torque[4] << " " << present_torque[5] << " " << present_torque[6] << " " << present_torque[7] << std::endl;
    //     // //  std::cout << j << " " << present_torque[0] << " " << present_torque[1] << " " << present_torque[2] << " " << present_torque[3] << " " << present_torque[4] << " " << present_torque[5] << " " << present_torque[6] << " " << present_torque[7] << std::endl;

    //     //usleep(1000);
    // }

    // data.close();

    // safe_start(20);
    // setCranex7TorqueState(TORQUE_DISABLE);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // uint8_t operating_mode[JOINT_NUM] = {OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION, OPERATING_MODE_POSITION};

    // if (initilizeCranex7(operating_mode))
    // {
    //     return 1;
    // }
    // setCranex7TorqueState(TORQUE_ENABLE);

    // safe_start(20);

    // for (j = 0; j < MAX_DATA; j++)
    // {
    //     for (i = 0; i < 7; i++)
    //     {
    //         th_run[i] = 0;
    //         th_rad[i] = th_run[i] * (M_PI / 180);
    //     }

    //     // std::cout << j << " " << th_run[0] << " " << th_run[1] << " " << th_run[2] << " " << th_run[3] << " " << th_run[4] << " " << th_run[5] << " " << th_run[6] << " " << th_run[7] << std::endl;
    //     // std::cout << j << " " << th_run[3] << std::endl;

    //     // usleep(50000);
    //     setCranex7Angle(th_run);
    //     // getCranex7JointState(present_position, present_velocity, present_torque);
    //     getCranex7Velocity(present_angvel);
    //     getCranex7Torque(present_torque);

    //     getCranex7EstimatedTorque(th_rad, present_angvel, present_torque, estimated_torque);
    //     // std::cout << j << " Estimated Torque || " << " " << estimated_torque[0] << " " << estimated_torque[1] << " " << estimated_torque[2] << " " << estimated_torque[3] << " " << estimated_torque[4] << " " << estimated_torque[5] << " " << estimated_torque[6] << " " << estimated_torque[7] << std::endl;
    //     std::cout << j << " Present Torque || "
    //               << " " << present_torque[0] << " " << present_torque[1] << " " << present_torque[2] << " " << present_torque[3] << " " << present_torque[4] << " " << present_torque[5] << " " << present_torque[6] << " " << present_torque[7] << std::endl;

    //     // Calculate Joint torque error between estimated and real
    //     for (int i = 0; i < 7; i++)
    //     {
    //         error_torque[i] = abs(estimated_torque[i] - present_torque[i]);
    //     }

    //     //getCranex7EstimatedExtForces(th_rad, present_torque, forces);
    //     // std::cout << j << "Estimated forces || " << " " << forces[0] << " " << forces[1] << " " << forces[2] << " " << forces[3] << " " << forces[4] << " " << forces[5] << std::endl;
    //     std::cout << j << " Estimated forces || "
    //               << " Fx : " << forces[0] << " || Fy : " << forces[1] << " || Fz : " << forces[2] << std::endl;

    //     // std::cout << j << " " << present_torque[0] << " " << present_torque[1] << " " << present_torque[2] << " " << present_torque[3] << " " << present_torque[4] << " " << present_torque[5] << " " << present_torque[6] << " " << present_torque[7] << std::endl;
    //     // //  std::cout << j << " " << present_torque[0] << " " << present_torque[1] << " " << present_torque[2] << " " << present_torque[3] << " " << present_torque[4] << " " << present_torque[5] << " " << present_torque[6] << " " << present_torque[7] << std::endl;

    //     // usleep(1000);
    // }

    // safe_start(20);
    // setCranex7TorqueState(TORQUE_DISABLE);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    double th_radd[7] = {0.785398, 0.785398, 0.349066, 0.349066, 0.349066, 0.349066, 0.349066};
    double present_torquee[7] = {0, -0.0842427, 0, 0, 0, 0.0997624, 0};

    getCranex7EstimatedExtForces(th_radd, present_torquee, forces);

    return 0;
}