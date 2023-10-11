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
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_CURRENT 126

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 3000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB1" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

uint8_t dxl_id[7] = {2,3,4,5,6,7,8};
uint8_t dxl_error = 0;
uint32_t dxl_goal_position = 400; // Goal position
int dxl_comm_result = COMM_TX_FAIL;
bool dxl_addparam_result = false; // addParam result
bool dxl_getdata_result = false;  // GetParam result

const int ms = 5;

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
        3,
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

int main(int argc, char *argv[])
{
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

    uint8_t dxl_error = 0;

    // Position Value of X series is 4 byte data.
    // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
    uint32_t goal_position = 400;

    int present_position[1000];
    int present_current[1000];

    // for (;;)
    // {
    //     // Write Goal Position (length : 4 bytes)
    //     // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
    //     dxl_comm_result =
    //         packetHandler->write4ByteTxRx(
    //             portHandler,
    //             8,
    //             ADDR_GOAL_POSITION,
    //             goal_position,
    //             &dxl_error);

    //     if (dxl_comm_result != COMM_SUCCESS)
    //     {
    //         RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    //     }
    //     else if (dxl_error != 0)
    //     {
    //         RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "%s", packetHandler->getRxPacketError(dxl_error));
    //     }
    //     else
    //     {
    //         RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Set [ID: %d] [Goal Position: %d]", dxl_id[0], goal_position);
    //     }

    //     usleep(100000);
    // }

    // std::ofstream myfile1;
    // std::ofstream myfile2;

    std::ofstream myfileC;
    std::ofstream myfileT;

    // myfile1.open("input.txt");
    // myfile2.open("output.txt");

    myfileC.open("current.txt");
    myfileT.open("torque.txt");

    int i;
    int count = 1;

    int Fs = 10;
    float dt = 1 / Fs;
    float StopTime = 0.25;
    double t[10000];

    double th[10000];
    double th_d[10000];
    double th_dd[10000];

    // Frequency wave
    float Fc_1 = 0.145;
    float Fc_2 = 0.295;
    float Fc_3 = 0.355;

    float A_1 = 65;
    float A_2 = -65;

    float A_3 = (-A_1 * Fc_1 - A_2 * Fc_2) / Fc_3;

    t[0] = 0;
    for (int i = 1; i <= 10000; i++)
    {
        t[i] = t[i - 1] + 0.1;
        // std::cout << t[i] << std::endl;
    }

    // Generate wave
    th[0] = 0;
    th_d[0] = 0;
    th_d[0] = 0;
    for (int i = 1; i <= 10000; i++)
    {
        th[i] = A_1 * sin_deg(2 * M_PI * Fc_1 * t[i]) + A_2 * sin_deg(2 * M_PI * Fc_2 * t[i]) + A_3 * sin_deg(2 * M_PI * Fc_3 * t[i]);
        th_d[i] = A_1 * Fc_1 * cos_deg(2 * M_PI * Fc_1 * t[i]) + A_2 * Fc_2 * cos_deg(2 * M_PI * Fc_2 * t[i]) + A_3 * Fc_3 * cos_deg(2 * M_PI * Fc_3 * t[i]);
        th_d[i] = -(A_1 * pow(Fc_1, 2) * sin_deg(2 * M_PI * Fc_1 * t[i]) + A_2 * pow(Fc_2, 2) * sin_deg(2 * M_PI * Fc_2 * t[i]) + A_3 * pow(Fc_3, 2) * sin_deg(2 * M_PI * Fc_3 * t[i]));
    }

    for (i = 0; i < 10000; i++)
    {
        // Write Goal Position (length : 4 bytes)
        // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
        dxl_comm_result =
            packetHandler->write4ByteTxRx(
                portHandler,
                dxl_id[4],
                ADDR_GOAL_POSITION,
                (int)map_range(th[i], -180, 180, 0, 4096),
                &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            break;
        }
        else if (dxl_error != 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "%s", packetHandler->getRxPacketError(dxl_error));
            break;
        }
        // else
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[I: %d] Set [ID: %d] [Goal Position: %d]", i, dxl_id[0], (int)map_range(th[i], -180, 180, 0, 4096));
        // }

        // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
        // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler,
            dxl_id[4],
            ADDR_PRESENT_CURRENT,
            reinterpret_cast<uint32_t *>(&present_current[i]),
            &dxl_error);

        RCLCPP_INFO(
            rclcpp::get_logger("obtain_data_node"),
            "[I: %d] Get [ID: %d] [Present Current: %.03lf mA] [Torque: %lf mN.m]",
            i,
            dxl_id[4],
            (int16_t)present_current[i] * 2.69,
            (double)map_range((int16_t)present_current[i] * 2.69, -2300, 2300, -4100, 4100));

        // usleep(50000);

        // std::cout << th[i] << std::endl;
        // myfile1 << (int)th[i] << std::endl;
        // myfile2 << (int)map_range(present_position[i], 0, 4096, -180, 180) << std::endl;]
        myfileC << (int16_t)present_current[i] * 2.69 << std::endl;
        myfileT << (double)map_range((int16_t)present_current[i] * 2.69, -2300, 2300, -4100, 4100) << std::endl;
    }

    // Generate wave
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // for (;;)
    // {
    //     std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    //     // delay for 5 ms
    //     // for(i = 0 ; i < ms ; i++) {
    //     std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms] " << count++ << std::endl;

    //     // std::cout << std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count()  << count++ << std::endl;

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

    // Disable Torque of DYNAMIXEL
    packetHandler->write1ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error);

    return 0;
}