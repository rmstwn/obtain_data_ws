#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_velocity.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_velocity.hpp"

#include "obtain_data/dynamixel_node.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

// class MinimalPublisher : public rclcpp::Node
// {
//   public:
//     MinimalPublisher()
//     : Node("minimal_publisher"), count_(0)
//     {
//       publisher_ = this->create_publisher<std_msgs::msg::String>("toM_PIc", 10);
//       timer_ = this->create_wall_timer(
//       500ms, std::bind(&MinimalPublisher::timer_callback, this));
//     }

//   private:
//     void timer_callback()
//     {
//       auto message = std_msgs::msg::String();
//       message.data = "Hello, world! " + std::to_string(count_++);
//       RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//       publisher_->publish(message);
//     }
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//     size_t count_;
// };

#// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_CURRENT 126

#define OPERATING_MODE_CURRENT 0
#define OPERATING_MODE_VELOCITY 1
#define OPERATING_MODE_POSITION 3

// Data Byte Length
#define LENGTH_GOAL_POSITION 4
#define LENGTH_PRESENT_POSITION 4
#define LENGTH_PRESENT_CURRENT 4

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 3000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"
#define JN 7
#define MAX_DATA 30000

#define ESC_ASCII_VALUE 0x1b

dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LENGTH_GOAL_POSITION);
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, LENGTH_PRESENT_CURRENT);

uint8_t dxl_id[7] = {2, 3, 4, 5, 6, 7, 8};
uint8_t dxl_error = 0;
uint32_t dxl_goal_position = 0; // Goal position
int dxl_comm_result = COMM_TX_FAIL;
bool dxl_addparam_result = false; // addParam result
bool dxl_getdata_result = false;  // GetParam result

const int ms = 5;

// Position Value of X series is 4 byte data.
// For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
uint32_t goal_position[JN][MAX_DATA];
uint8_t param_goal_position[JN][4];
uint8_t param_goal_position1[4];

int present_position[JN][MAX_DATA];
int present_current[JN][MAX_DATA];

// Frequency wave
float Fc_1[] = {0.1166, 0.1263, 0.1451, 0.1602, 0.1654, 0.1689, 0.1748};
float Fc_2[] = {0.2084, 0.2152, 0.2229, 0.2538, 0.2826, 0.2913, 0.2996};
float Fc_3[] = {0.3005, 0.3289, 0.3396, 0.3725, 0.4169, 0.5341, 0.5826};
// float Fc_1[] = {0.1166, 0.2263, 0.1451, 0.2602, 0.1654, 0.1689, 0.1748};
// float Fc_2[] = {0.2084, 0.3152, 0.2229, 0.3538, 0.2826, 0.2913, 0.2996};
// float Fc_3[] = {0.3005, 0.4289, 0.3396, 0.5725, 0.4169, 0.5341, 0.5826};

// Amplitude
// float A_1[] = {65, 8, 65, 55, 65, 35, 65};
// float A_2[] = {-65, -8, -65, -55, -65, -35, -65};
// float A_1[] = { 45,  8,  45,  45,  45,  35,  45};
// float A_2[] = {-45, -8, -45, -45, -45, -35, -45};
float A_1[] = {65, 8, 65, 55, 65, 35, 65};
float A_2[] = {-65, -8, -65, -55, -65, -35, -65};

float A_3[JN];

int i;
int j;
int count = 1;

int Fs = 1;
float dt = 1 / Fs;
float StopTime = 0.25;
double t[MAX_DATA];

double th[JN][MAX_DATA];
double th_d[JN][MAX_DATA];
double th_dd[JN][MAX_DATA];

double deg_to_rad(double deg)
{
    return deg * M_PI / 180.0;
}

double sin_deg(double deg)
{
    double rad = deg_to_rad(deg);
    return sin(rad);
}

double cos_deg(double deg)
{
    double rad = deg_to_rad(deg);
    return cos(rad);
}

void setupDynamixel(uint8_t dxl_id)
{
    // Use Position Control Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_OPERATING_MODE,
        OPERATING_MODE_POSITION,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to set Position Control Mode.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to set Position Control Mode.");
    }

    // Enable Torque of DYNAMIXEL
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to enable torque.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to enable torque.");
    }
}

double map_range(double value, double in_min, double in_max, double out_min, double out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void safe_start(int velocity, double init_pos[JN][MAX_DATA])
{
    // Set profile Velocity of DYNAMIXEL
    packetHandler->write4ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_PROFILE_VELOCITY,
        velocity,
        &dxl_error);

    int j = 0;

    for (i = 0; i < 7; i++)
    {
        if (i == 3)
        {
            // Allocate goal position value into byte array
            param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(init_pos[i][j], -180, 180, 768, 2048)));
            param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(init_pos[i][j], -180, 180, 768, 2048)));
            param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(init_pos[i][j], -180, 180, 768, 2048)));
            param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(init_pos[i][j], -180, 180, 768, 2048)));
        }
        else
        {
            // Allocate goal position value into byte array
            param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(init_pos[i][j], -180, 180, 0, 4096)));
            param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(init_pos[i][j], -180, 180, 0, 4096)));
            param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(init_pos[i][j], -180, 180, 0, 4096)));
            param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(init_pos[i][j], -180, 180, 0, 4096)));
        }

        // std::cout << param_goal_position[i][0] << std::endl;
    }

    for (i = 0; i < 7; i++)
    {
        dxl_addparam_result = groupSyncWrite.addParam(dxl_id[i], param_goal_position[i]);

        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncWrite addparam failed", dxl_id[i]);
        }

        // std::cout << param_goal_position[i] << std::endl;
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        packetHandler->getTxRxResult(dxl_comm_result);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "%s", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Set [I: %d]", j);
    }

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    usleep(7000000);

    // Set profile Velocity of DYNAMIXEL
    packetHandler->write4ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_PROFILE_VELOCITY,
        (int)0,
        &dxl_error);
}

int main(int argc, char *argv[])
{
    // Create log file
    std::ofstream data;

    data.open("data.csv");

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

    // Open Serial Port
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false)
    {
        RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to open the port!");
        return -1;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false)
    {
        RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to set the baudrate!");
        return -1;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to set the baudrate.");
    }

    setupDynamixel(BROADCAST_ID);

    for (j = 0; j < 7; j++)
    {
        dxl_addparam_result = groupSyncRead.addParam(dxl_id[j]);
        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead addparam failed", dxl_id[j]);
            return 0;
        }
    }

    safe_start(20, th);
    auto start = std::chrono::high_resolution_clock::now();

    for (j = 0; j < MAX_DATA; j++)
    {

        for (i = 0; i < 7; i++)
        {
            if (i == 3)
            {
                // Allocate goal position value into byte array
                param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(th[i][j], -180, 180, 768, 2048)));
                param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(th[i][j], -180, 180, 768, 2048)));
                param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(th[i][j], -180, 180, 768, 2048)));
                param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(th[i][j], -180, 180, 768, 2048)));
            }
            else
            {
                // Allocate goal position value into byte array
                param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(th[i][j], -180, 180, 0, 4096)));
                param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(th[i][j], -180, 180, 0, 4096)));
                param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(th[i][j], -180, 180, 0, 4096)));
                param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(th[i][j], -180, 180, 0, 4096)));
            }

            // std::cout << param_goal_position[i][0] << std::endl;
        }

        for (i = 0; i < 7; i++)
        {
            dxl_addparam_result = groupSyncWrite.addParam(dxl_id[i], param_goal_position[i]);

            if (dxl_addparam_result != true)
            {
                RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncWrite addparam failed", dxl_id[i]);
            }

            // std::cout << param_goal_position[i] << std::endl;
        }

        // Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS)
            packetHandler->getTxRxResult(dxl_comm_result);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "%s", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "%s", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            // RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Set [I: %d]", j);
        }

        // Clear syncwrite parameter storage
        groupSyncWrite.clearParam();

        // Read Data
        dxl_comm_result = groupSyncRead.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS)
            packetHandler->getTxRxResult(dxl_comm_result);

        for (i = 0; i < 7; i++)
        {
            // Check if groupsyncread data of Dynamixel is available
            dxl_getdata_result = groupSyncRead.isAvailable(dxl_id[i], ADDR_PRESENT_CURRENT, LENGTH_PRESENT_CURRENT);
            if (dxl_getdata_result != true)
            {
                RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead getdata failed", dxl_id[i]);
                return 0;
            }
        }

        for (i = 0; i < 7; i++)
        {
            present_current[i][j] = groupSyncRead.getData(dxl_id[i], ADDR_PRESENT_CURRENT, LENGTH_PRESENT_CURRENT);
        }

        auto end = std::chrono::high_resolution_clock::now();

        double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        time_taken *= 1e-9;

        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[Time: %.05lf] [I: %d] [Target Pos Joint 0: %d deg] [Current Joint 0: %.03lf mA]", time_taken, j, (int16_t)th[0][j], (int16_t)present_current[1][j] * 2.69);

        //data << time_taken << "," << (int16_t)present_current[0][j] * 2.69 << "," << (int16_t)present_current[1][j] * 2.69 << "," << (int16_t)present_current[2][j] * 2.69 << "," << (int16_t)present_current[3][j] * 2.69 << "," << (int16_t)present_current[4][j] * 2.69 << "," << (int16_t)present_current[5][j] * 2.69 << "," << (int16_t)present_current[6][j] * 2.69 << "," << (double)map_range((int16_t)present_current[0][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[1][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[2][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[3][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[4][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[5][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[6][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << std::endl;

        data << time_taken << "," << (int16_t)th[0][j] << "," << (int16_t)th[1][j] << "," << (int16_t)th[2][j] << "," << (int16_t)th[3][j] << "," << (int16_t)th[4][j] << "," << (int16_t)th[5][j] << "," << (int16_t)th[6][j] << "," << (double)map_range((int16_t)present_current[0][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[1][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[2][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[3][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[4][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[5][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << "," << (double)map_range((int16_t)present_current[6][j] * 2.69, -2300, 2300, -4100, 4100) * 2.69 << std::endl;
        
        // myfileC << (int16_t)present_current[i] * 2.69 << std::endl;
        // myfileT << (double)map_range((int16_t)present_current[i] * 2.69, -2300, 2300, -4100, 4100) << std::endl;

        //usleep(5000);
        //  break;
    }

    data.close();

    safe_start(20, th);

    // std::ofstream myfileC;
    // std::ofstream myfileT;

    // // myfile1.open("input.txt");
    // // myfile2.open("output.txt");

    // myfileC.open("current.txt");
    // myfileT.open("torque.txt");

    // int i;
    // int count = 1;

    // int Fs = 1;
    // float dt = 1 / Fs;
    // float StopTime = 0.25;
    // double t[10000];

    // double th[10000];
    // double th_d[10000];
    // double th_dd[10000];

    // // Frequency wave
    // float Fc_1 = 0.145;
    // float Fc_2 = 0.295;
    // float Fc_3 = 0.355;

    // float A_1 = 65;
    // float A_2 = -65;

    // float A_3 = (-A_1 * Fc_1 - A_2 * Fc_2) / Fc_3;

    // t[0] = 0;
    // for (int i = 1; i <= 10000; i++)
    // {
    //     t[i] = t[i - 1] + 0.1;
    //     // std::cout << t[i] << std::endl;
    // }

    // // Generate wave
    // th[0] = 0;
    // th_d[0] = 0;
    // th_d[0] = 0;
    // for (int i = 1; i <= 10000; i++)
    // {
    //     th[i] = A_1 * sin_deg(2 * M_PI * Fc_1 * t[i]) + A_2 * sin_deg(2 * M_PI * Fc_2 * t[i]) + A_3 * sin_deg(2 * M_PI * Fc_3 * t[i]);
    //     th_d[i] = A_1 * Fc_1 * cos_deg(2 * M_PI * Fc_1 * t[i]) + A_2 * Fc_2 * cos_deg(2 * M_PI * Fc_2 * t[i]) + A_3 * Fc_3 * cos_deg(2 * M_PI * Fc_3 * t[i]);
    //     th_d[i] = -(A_1 * pow(Fc_1, 2) * sin_deg(2 * M_PI * Fc_1 * t[i]) + A_2 * pow(Fc_2, 2) * sin_deg(2 * M_PI * Fc_2 * t[i]) + A_3 * pow(Fc_3, 2) * sin_deg(2 * M_PI * Fc_3 * t[i]));
    // }

    // for (i = 0; i < 10000; i++)
    // {
    //     // Write Goal Position (length : 4 bytes)
    //     // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
    //     dxl_comm_result =
    //         packetHandler->write4ByteTxRx(
    //             portHandler,
    //             dxl_id[4],
    //             ADDR_GOAL_POSITION,
    //             (int)map_range(th[i], -180, 180, 0, 4096),
    //             &dxl_error);

    //     if (dxl_comm_result != COMM_SUCCESS)
    //     {
    //         RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    //         break;
    //     }
    //     else if (dxl_error != 0)
    //     {
    //         RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "%s", packetHandler->getRxPacketError(dxl_error));
    //         break;
    //     }
    //     // else
    //     // {
    //     //     RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[I: %d] Set [ID: %d] [Goal Position: %d]", i, dxl_id[0], (int)map_range(th[i], -180, 180, 0, 4096));
    //     // }

    //     // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
    //     // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
    //     dxl_comm_result = packetHandler->read4ByteTxRx(
    //         portHandler,
    //         dxl_id[4],
    //         ADDR_PRESENT_CURRENT,
    //         reinterpret_cast<uint32_t *>(&present_current[i]),
    //         &dxl_error);

    //     RCLCPP_INFO(
    //         rclcpp::get_logger("obtain_data_node"),
    //         "[I: %d] Get [ID: %d] [Present Current: %.03lf mA] [Torque: %lf mN.m]",
    //         i,
    //         dxl_id[4],
    //         (int16_t)present_current[i] * 2.69,
    //         (double)map_range((int16_t)present_current[i] * 2.69, -2300, 2300, -4100, 4100));

    //     // usleep(50000);

    //     // std::cout << th[i] << std::endl;
    //     // myfile1 << (int)th[i] << std::endl;
    //     // myfile2 << (int)map_range(present_position[i], 0, 4096, -180, 180) << std::endl;]
    //     myfileC << (int16_t)present_current[i] * 2.69 << std::endl;
    //     myfileT << (double)map_range((int16_t)present_current[i] * 2.69, -2300, 2300, -4100, 4100) << std::endl;
    // }

    // Generate wave
    // std::std::chrono::steady_clock::time_point begin = std::std::chrono::steady_clock::now();

    // for (;;)
    // {
    //     std::std::chrono::steady_clock::time_point end = std::std::chrono::steady_clock::now();

    //     // delay for 5 ms
    //     // for(i = 0 ; i < ms ; i++) {
    //     std::cout << std::std::chrono::duration_cast<std::std::chrono::milliseconds>(end - begin).count() << "[ms] " << count++ << std::endl;

    //     // std::cout << std::std::chrono::duration_cast<std::std::chrono::milliseconds> (end - begin).count()  << count++ << std::endl;

    //     usleep(10000);
    //     //}
    //     // print
    //     // RCLCPP_INFO("Publishing: '%d'", count++);
    //     // printf("%d\n", count++);

    //     if (count > 1200)
    //         break;
    // }

    // rclcpp::init(argc, argv);
    // rclcpp::sM_PIn();
    // rclcpp::shutdown();

    // // Disable Torque of DYNAMIXEL
    // packetHandler->write1ByteTxRx(
    //     portHandler,
    //     BROADCAST_ID,
    //     ADDR_TORQUE_ENABLE,
    //     0,
    //     &dxl_error);

    return 0;
}