#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "obtain_data/crane_x7_comm.hpp"

#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_velocity.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_velocity.hpp"

//// Port Handler ////
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(SERIAL_PORT);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LENGTH_GOAL_POSITION);
dynamixel::GroupSyncRead groupSyncReadPosition(portHandler, packetHandler, ADDR_PRESENT_POSITION, LENGTH_PRESENT_POSITION);
dynamixel::GroupSyncRead groupSyncReadVelocity(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LENGTH_PRESENT_VELOCITY);
dynamixel::GroupSyncRead groupSyncReadCurrent(portHandler, packetHandler, ADDR_PRESENT_CURRENT, LENGTH_PRESENT_CURRENT);

//// Unique set values of each servo motor ////
static const uint8_t id_array[JOINT_NUM] = {2, 3, 4, 5, 6, 7, 8, 9};                                  // ID (a unique value to identify each servo motor)
static const uint32_t min_angle_array[JOINT_NUM] = {262, 1024, 262, 228, 262, 1024, 148, 1991};       // Min angle (expressed as raw value of the dynamixel motor)
static const uint32_t max_angle_array[JOINT_NUM] = {3834, 3072, 3834, 2048, 3834, 3072, 3928, 3072};  // Max angle (expressed as raw value of the dynamixel motor)
static const uint32_t home_angle_array[JOINT_NUM] = {2048, 1024, 2048, 2048, 2048, 2048, 2048, 2048}; // Values at 0 radian posture (expressed as raw value of the dynamixel motor)

//// Variable for DynamixelSDK ////
static int port_num = 0;       // PortHandler Structs number
static int groupwrite_num = 0; // Groupbulkwrite Struct number
static int groupread_num = 0;  // Groupbulkread Struct number
static int dxl_comm_result = COMM_TX_FAIL;
static bool dxl_addparam_result = false;   // addParam result
static bool dxl_getdata_result = false;    // GetParam result
static uint8_t dxl_error;                  // Dynamixel error
uint8_t param_goal_position[JOINT_NUM][4]; // SyncWrite Param

//// Unit convertion functions for dynamixel ////
/**
 * @fn static double rad2dxlvalue(double)
 * @brief Angle unit conversion function from rad to dynamixel value
 * @param[in] rad :angle[rad/s]
 * @return value :angle[dynamixel value]
 */
static double rad2dxlvalue(double rad)
{
    double value = rad / (DXL_VALUE_TO_RADIAN);
    return value;
}

/**
 * @fn static double dxlvalue2rad(double)
 * @brief Angle unit conversion function from dynamixel value to rad
 * @param[in] value :angle[dynamixel value]
 * @return rad :angle[rad/s]
 */
static double dxlvalue2rad(double value)
{
    double rad = value * (DXL_VALUE_TO_RADIAN);
    return rad;
}

/**
 * @fn static double angularvel2dxlvalue(double)
 * @brief Anglular velocity unit conversion function from rad/s to dynamixel value
 * @param[in] angular_velocity :anglular velocity[rad/s]
 * @return value :anglular velocity[dynamixel value]
 */
static double angularvel2dxlvalue(double angular_velocity)
{
    double value = angular_velocity / (DXL_VALUE_TO_ANGULARVEL);
    return value;
}

/**
 * @fn static double dxlvalue2angularvel(double)
 * @brief Anglular velocity unit conversion function from dynamixel value to rad/s
 * @param[in] value :anglular velocity[dynamixel value]
 * @return angular_velocity :anglular velocity[rad/s]
 */
static double dxlvalue2angularvel(double value)
{
    double angular_velocity = value * (DXL_VALUE_TO_ANGULARVEL);
    return angular_velocity;
}

/**
 * @fn static double current2dxlvalue(double)
 * @brief Current unit conversion function from A to dynamixel value
 * @param[in] current :current[A]
 * @return value :current[dynamixel value]
 */
static double current2dxlvalue(double current)
{
    double value = current / (DXL_VALUE_TO_CURRENT);
    return value;
}

/**
 * @fn static double dxlvalue2current(double)
 * @brief Current unit conversion function from dynamixel value to A
 * @param[in] value :current[dynamixel value]
 * @return current :current[A]
 */
static double dxlvalue2current(double value)
{
    double current = value * (DXL_VALUE_TO_CURRENT);
    return current;
}

/**
 * @fn static double current2torqueXM430W350(double)
 * @brief Conversion function from current[A] to torque[Nm] for XM430W350
 * @param[in] current :current[A]
 * @return torque :torque[Nm]
 */
static double current2torqueXM430W350(double current)
{
    double torque = current * (CURRENT_TO_TORQUE_XM430W350);
    return torque;
}

/**
 * @fn static double current2torqueXM540W270(double)
 * @brief Conversion function from current[A] to torque[Nm] for XM540W270
 * @param[in] current :current[A]
 * @return torque :torque[Nm]
 */
static double current2torqueXM540W270(double current)
{
    double torque = current * (CURRENT_TO_TORQUE_XM540W270);
    return torque;
}

/**
 * @fn static double torque2currentXM430W350(double)
 * @brief Conversion function from torque[Nm] to current[A] for XM430W350
 * @param[in] torque :torque[Nm]
 * @return current :current[A]
 */
static double torque2currentXM430W350(double torque)
{
    double current = torque / (CURRENT_TO_TORQUE_XM430W350);
    return current;
}

/**
 * @fn static double torque2currentXM540W270(double)
 * @brief Conversion function from torque[Nm] to current[A] for XM540W270
 * @param[in] torque :torque[Nm]
 * @return current :current[A]
 */
static double torque2currentXM540W270(double torque)
{
    double current = torque / (CURRENT_TO_TORQUE_XM540W270);
    return current;
}

/**
 * @fn double deg_to_rad(double)
 * @brief Conversion function from Degree to Radian
 * @param[in] deg :degree
 * @return radian :radian
 */
double deg_to_rad(double deg)
{
    return deg * M_PI / 180.0;
}

/**
 * @fn double sin_deg(double)
 * @brief Calculate sin from Degree value
 * @param[in] deg :degree
 * @return sin :sin
 */
double sin_deg(double deg)
{
    double rad = deg_to_rad(deg);
    return sin(rad);
}

/**
 * @fn double cos_deg(double)
 * @brief Calculate cos from Degree value
 * @param[in] deg :degree
 * @return cos :cos
 */
double cos_deg(double deg)
{
    double rad = deg_to_rad(deg);
    return cos(rad);
}

/**
 * @fn double map_range(double)
 * @brief Convert value from a range to another range
 * @param[in] value, in_min, in_max, in_max, out_min, out_max
 * @return value :value
 */
double map_range(double value, double in_min, double in_max, double out_min, double out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @fn void sleeps(uint8_t)
 * @brief Delay in Seconds
 */
void sleeps(uint8_t second)
{
    usleep(second * 1000000);
}

//// Communication functions for CRANE-X7 ////

/**
 * @fn int initilizeCranex7(uint8_t *)
 * @brief Initilizetion function of CRANE-X7
 * @param[in] *operationg_mode An array containing the operating modes of each servo motor.
 * @return Success or failure of initilizetion.
 */
int initilizeCranex7(uint8_t *operating_mode_array)
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

    // Turn off the torque to change operating mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_TORQUE_ENABLE,
        TORQUE_DISABLE,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to disable torque.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to disable torque.");
    }

    // Set operating mode
    for (int i = 0; i < JOINT_NUM; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler,
            id_array[i],
            ADDR_OPERATING_MODE,
            (uint8_t)operating_mode_array[i],
            &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to set Position Control Mode.");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to set Position Control Mode.");
        }
    }

    // Set position p gain to the defalut value
    // (PortHandler *port, uint8_t id, uint16_t address, uint16_t data, uint8_t *error)
    // dxl_comm_result = packetHandler->write2ByteTxRx();
    dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_POSITION_P_GAIN,
        (uint16_t)DEFAULT_POSITION_P_GAIN,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to set Position P gain to the default value.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to set Position P gain to the default value.");
    }

    // Set position i gain to the defalut value
    dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_POSITION_I_GAIN,
        (uint16_t)DEFAULT_POSITION_I_GAIN,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to set Position I gain to the default value.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to set Position I gain to the default value.");
    }

    // Set position d gain to the defalut value
    dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_POSITION_D_GAIN,
        (uint16_t)DEFAULT_POSITION_D_GAIN,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to set Position I gain to the default value.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to set Position I gain to the default value.");
    }

    // Set velocity p gain to the defalut value
    dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_VELOCITY_P_GAIN,
        (uint16_t)DEFAULT_VELOCITY_P_GAIN,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to set Position I gain to the default value.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to set Position I gain to the default value.");
    }

    // Set velocity i gain to the defalut value
    dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_VELOCITY_I_GAIN,
        (uint16_t)DEFAULT_VELOCITY_I_GAIN,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to set Position I gain to the default value.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to set Position I gain to the default value.");
    }

    // Set velocity profile
    packetHandler->write4ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_PROFILE_VELOCITY,
        (uint32_t)PROFILE_VELOCITY,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to set Position I gain to the default value.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to set Position I gain to the default value.");
    }

    // Add param for joint state read
    for (int i = 0; i < JOINT_NUM; i++)
    {
        dxl_addparam_result = groupSyncReadPosition.addParam(id_array[i]);
        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead addparam failed", id_array[i]);
            return 0;
        }
        dxl_addparam_result = groupSyncReadVelocity.addParam(id_array[i]);
        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead addparam failed", id_array[i]);
            return 0;
        }

        dxl_addparam_result = groupSyncReadCurrent.addParam(id_array[i]);
        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead addparam failed", id_array[i]);
            return 0;
        }
    }

    return 0;
}

int AddSyncReadParam(void)
{

    // Add param for joint state read
    for (int i = 0; i < JOINT_NUM; i++)
    {
        dxl_addparam_result = groupSyncReadPosition.addParam(id_array[i]);
        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead addparam failed", id_array[i]);
            return 0;
        }
        dxl_addparam_result = groupSyncReadVelocity.addParam(id_array[i]);
        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead addparam failed", id_array[i]);
            return 0;
        }

        dxl_addparam_result = groupSyncReadCurrent.addParam(id_array[i]);
        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead addparam failed", id_array[i]);
            return 0;
        }
    }

    return 0;
}

/**
 * @fn int setCranex7Torque(int)
 * @brief Function to set command torque
 * @param[in] torque_array[] command torque array
 * @return Success or failure.
 */
int setCranex7TorqueState(int torque_enable)
{
    if (torque_enable == 1)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler,
            BROADCAST_ID,
            ADDR_TORQUE_ENABLE,
            TORQUE_ENABLE,
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
    else if (torque_enable == 0)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler,
            BROADCAST_ID,
            ADDR_TORQUE_ENABLE,
            TORQUE_DISABLE,
            &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("obtain_data_node"), "Failed to disable torque.");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Succeeded to disable torque.");
        }
    }

    return 0;
}

/**
 * @fn void safe_start(int velocity)
 * @brief Function to execute safe motion to home pos
 * @param[in] velocity profile command
 */
void safe_start(int velocity)
{
    // Set profile Velocity of DYNAMIXEL
    packetHandler->write4ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_PROFILE_VELOCITY,
        velocity,
        &dxl_error);

    int i;

    for (i = 0; i < 7; i++)
    {
        // if (i == 3)
        // {
        //     // Allocate goal position value into byte array
        //     param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(0, -57.5, 57.5, 768, 2048)));
        //     param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(0, -57.5, 57.5, 768, 2048)));
        //     param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(0, -57.5, 57.5, 768, 2048)));
        //     param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(0, -57.5, 57.5, 768, 2048)));
        // }
        // else
        // {
        //     // Allocate goal position value into byte array
        //     param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(0, -180, 180, 0, 4096)));
        //     param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(0, -180, 180, 0, 4096)));
        //     param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(0, -180, 180, 0, 4096)));
        //     param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(0, -180, 180, 0, 4096)));
        // }

        // if (i == 3)
        // {
        //     // Allocate goal position value into byte array
        //     param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(-57.5, 180, -180, 4096, 0)));
        //     param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(-57.5, 180, -180, 4096, 0)));
        //     param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(-57.5, 180, -180, 4096, 0)));
        //     param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(-57.5, 180, -180, 4096, 0)));
        // }
        // else
        // {
        //     // Allocate goal position value into byte array
        //     param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(0, 180, -180, 4096, 0)));
        //     param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(0, 180, -180, 4096, 0)));
        //     param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(0, 180, -180, 4096, 0)));
        //     param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(0, 180, -180, 4096, 0)));
        // }

        // Allocate goal position value into byte array
        param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(0, 180, -180, 4096, 0)));
        param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(0, 180, -180, 4096, 0)));
        param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(0, 180, -180, 4096, 0)));
        param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(0, 180, -180, 4096, 0))); 

        // std::cout << param_goal_position[i][0] << std::endl;
    }

    for (i = 0; i < 7; i++)
    {
        dxl_addparam_result = groupSyncWrite.addParam(id_array[i], param_goal_position[i]);

        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncWrite addparam failed", id_array[i]);
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

    usleep(7000000);

    // Set profile Velocity of DYNAMIXEL
    packetHandler->write4ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_PROFILE_VELOCITY,
        (int)0,
        &dxl_error);
}

/**
 * @fn int setCranex7Angle(double)
 * @brief Function to set command angle
 * @param[in] angle_array[] command angle array
 * @return Success or failure.
 */
int setCranex7Angle(double *angle_array)
{
    // for (j = 0; j < MAX_DATA; j++)
    // {
    for (int i = 0; i < JOINT_NUM; i++)
    {
        // if (i == 3)
        // {
        //     // Allocate goal position value into byte array
        //     param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(angle_array[i], -57.5, 57.5, 768, 2048)));
        //     param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(angle_array[i], -57.5, 57.5, 768, 2048)));
        //     param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(angle_array[i], -57.5, 57.5, 768, 2048)));
        //     param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(angle_array[i], -57.5, 57.5, 768, 2048)));
        // }
        // else
        // {
        //     // Allocate goal position value into byte array
        //     param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(angle_array[i], -180, 180, 0, 4096)));
        //     param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(angle_array[i], -180, 180, 0, 4096)));
        //     param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(angle_array[i], -180, 180, 0, 4096)));
        //     param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(angle_array[i], -180, 180, 0, 4096)));
        // }

        // Allocate goal position value into byte array
        param_goal_position[i][0] = DXL_LOBYTE(DXL_LOWORD((int)map_range(angle_array[i], 180, -180, 4096, 0)));
        param_goal_position[i][1] = DXL_HIBYTE(DXL_LOWORD((int)map_range(angle_array[i], 180, -180, 4096, 0)));
        param_goal_position[i][2] = DXL_LOBYTE(DXL_HIWORD((int)map_range(angle_array[i], 180, -180, 4096, 0)));
        param_goal_position[i][3] = DXL_HIBYTE(DXL_HIWORD((int)map_range(angle_array[i], 180, -180, 4096, 0)));

        // std::cout << param_goal_position[i][0] << std::endl;
    }

    for (int i = 0; i < JOINT_NUM; i++)
    {
        dxl_addparam_result = groupSyncWrite.addParam(id_array[i], param_goal_position[i]);

        if (dxl_addparam_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncWrite addparam failed", id_array[i]);
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
        // RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Set Goal Pos");
    }

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    return 0;
}

/**
 * @fn int getCranex7JointState(double *, double *, double *)
 * @brief Function to get joint state
 * @param[out] angle_array[] present angle array
 * @param[out] angular_velocity_array[] present angular velocity array
 * @param[out] torque_array[] present torque array
 * @return Success or failure.
 */
int getCranex7JointState(double *angle_array, double *angular_velocity_array, double *torque_array)
{
    int32_t present_position[JOINT_NUM] = {0};
    int16_t present_velocity[JOINT_NUM] = {0};
    int16_t present_current[JOINT_NUM] = {0};

    // Read Position Data
    dxl_comm_result = groupSyncReadPosition.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        packetHandler->getTxRxResult(dxl_comm_result);

    for (int i = 0; i < JOINT_NUM; i++)
    {
        // Check if groupsyncread data of Dynamixel is available
        dxl_getdata_result = groupSyncReadPosition.isAvailable(id_array[i], ADDR_PRESENT_POSITION, LENGTH_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead getdata position failed", id_array[i]);
            return 0;
        }
    }

    for (int i = 0; i < JOINT_NUM; i++)
    {
        present_position[i] = groupSyncReadPosition.getData(id_array[i], ADDR_PRESENT_POSITION, LENGTH_PRESENT_POSITION);
        angle_array[i] = dxlvalue2rad((double)(present_position[i] - (int32_t)home_angle_array[i]));
    }

    // Read Velocity Data
    dxl_comm_result = groupSyncReadVelocity.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        packetHandler->getTxRxResult(dxl_comm_result);

    for (int i = 0; i < JOINT_NUM; i++)
    {
        // Check if groupsyncread data of Dynamixel is available
        dxl_getdata_result = groupSyncReadVelocity.isAvailable(id_array[i], ADDR_PRESENT_VELOCITY, LENGTH_PRESENT_VELOCITY);
        if (dxl_getdata_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead getdata velocity failed", id_array[i]);
            return 0;
        }
    }

    for (int i = 0; i < JOINT_NUM; i++)
    {
        present_velocity[i] = groupSyncReadVelocity.getData(id_array[i], ADDR_PRESENT_VELOCITY, LENGTH_PRESENT_VELOCITY);
        angular_velocity_array[i] = dxlvalue2angularvel((double)present_velocity[i]);
    }

    // Read Current Data
    dxl_comm_result = groupSyncReadCurrent.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        packetHandler->getTxRxResult(dxl_comm_result);

    for (int i = 0; i < JOINT_NUM; i++)
    {
        // Check if groupsyncread data of Dynamixel is available
        dxl_getdata_result = groupSyncReadCurrent.isAvailable(id_array[i], ADDR_PRESENT_CURRENT, LENGTH_PRESENT_CURRENT);
        if (dxl_getdata_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead getdata current failed", id_array[i]);
            return 0;
        }
    }

    for (int i = 0; i < JOINT_NUM; i++)
    {
        present_current[i] = groupSyncReadCurrent.getData(id_array[i], ADDR_PRESENT_CURRENT, LENGTH_PRESENT_CURRENT);

        if (i == XM540_W270_JOINT)
        {
            torque_array[i] = current2torqueXM540W270(dxlvalue2current((double)present_current[i]));
        }
        else
        {
            torque_array[i] = current2torqueXM430W350(dxlvalue2current((double)present_current[i]));
        }
    }
}

/**
 * @fn int getCranex7Position(double *)
 * @brief Function to get joint current
 * @param[out] angle_array[] present angle array
 * @return Success or failure.
 */
int getCranex7Position(double *angle_array)
{
    int32_t present_position[JOINT_NUM] = {0};

    // Read Position Data
    dxl_comm_result = groupSyncReadPosition.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        packetHandler->getTxRxResult(dxl_comm_result);

    for (int i = 0; i < JOINT_NUM; i++)
    {
        // Check if groupsyncread data of Dynamixel is available
        dxl_getdata_result = groupSyncReadPosition.isAvailable(id_array[i], ADDR_PRESENT_POSITION, LENGTH_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead getdata position failed", id_array[i]);
            return 0;
        }
    }

    for (int i = 0; i < JOINT_NUM; i++)
    {
        present_position[i] = groupSyncReadPosition.getData(id_array[i], ADDR_PRESENT_POSITION, LENGTH_PRESENT_POSITION);
        angle_array[i] = dxlvalue2rad((double)(present_position[i] - (int32_t)home_angle_array[i]));
    }
}

/**
 * @fn int getCranex7Velocity(double *)
 * @brief Function to get joint current
 * @param[out] angular_velocity_array[] present angular velocity array
 * @return Success or failure.
 */
int getCranex7Velocity(double *angular_velocity_array)
{
    int16_t present_velocity[JOINT_NUM] = {0};

    // Read Velocity Data
    dxl_comm_result = groupSyncReadVelocity.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        packetHandler->getTxRxResult(dxl_comm_result);

    for (int i = 0; i < JOINT_NUM; i++)
    {
        // Check if groupsyncread data of Dynamixel is available
        dxl_getdata_result = groupSyncReadVelocity.isAvailable(id_array[i], ADDR_PRESENT_VELOCITY, LENGTH_PRESENT_VELOCITY);
        if (dxl_getdata_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead getdata velocity failed", id_array[i]);
            return 0;
        }
    }

    for (int i = 0; i < JOINT_NUM; i++)
    {
        present_velocity[i] = groupSyncReadVelocity.getData(id_array[i], ADDR_PRESENT_VELOCITY, LENGTH_PRESENT_VELOCITY);
        angular_velocity_array[i] = dxlvalue2angularvel((double)present_velocity[i]);
    }
}

/**
 * @fn int getCranex7Torque(double *)
 * @brief Function to get joint current
 * @param[out] torque_array[] present torque array
 * @return Success or failure.
 */
int getCranex7Torque(double *torque_array)
{
    int16_t present_current[JOINT_NUM] = {0};

    // Read Current Data
    dxl_comm_result = groupSyncReadCurrent.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        packetHandler->getTxRxResult(dxl_comm_result);

    for (int i = 0; i < JOINT_NUM; i++)
    {
        // Check if groupsyncread data of Dynamixel is available
        dxl_getdata_result = groupSyncReadCurrent.isAvailable(id_array[i], ADDR_PRESENT_CURRENT, LENGTH_PRESENT_CURRENT);
        if (dxl_getdata_result != true)
        {
            RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "[ID:%03d] groupSyncRead getdata current failed", id_array[i]);
            return 0;
        }
    }

    for (int i = 0; i < JOINT_NUM; i++)
    {
        present_current[i] = groupSyncReadCurrent.getData(id_array[i], ADDR_PRESENT_CURRENT, LENGTH_PRESENT_CURRENT);

        if (i == XM540_W270_JOINT)
        {
            torque_array[i] = current2torqueXM540W270(dxlvalue2current((double)present_current[i]));
        }
        else
        {
            torque_array[i] = current2torqueXM430W350(dxlvalue2current((double)present_current[i]));
        }
    }
}

/**
 * @fn void closeCranex7Port(void)
 * @brief Close port
 */
void closeCranex7Port(void)
{
    // Close port
    // Open Serial Port
    portHandler->closePort();
    RCLCPP_INFO(rclcpp::get_logger("obtain_data_node"), "Port closed.");
}
