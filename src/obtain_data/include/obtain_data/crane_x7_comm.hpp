#ifndef CRANE_X7_CONTROL_HPP_
#define CRANE_X7_CONTROL_HPP_

#include <stdint.h>

//// Definition of dynamixel ////

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE     11
#define ADDR_TORQUE_ENABLE      64
#define ADDR_VELOCITY_I_GAIN    76
#define ADDR_VELOCITY_P_GAIN    78
#define ADDR_POSITION_D_GAIN    80
#define ADDR_POSITION_I_GAIN    82
#define ADDR_POSITION_P_GAIN    84
#define ADDR_BUS_WATCHDOG       98
#define ADDR_GOAL_CURRENT       102
#define ADDR_GOAL_VELOCITY      104
#define ADDR_PROFILE_VELOCITY   112
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_CURRENT    126
#define ADDR_PRESENT_VELOCITY   128
#define ADDR_PRESENT_POSITION   132

// Data Byte Length

#define LENGTH_BUS_WATCHDOG     1
#define LENGTH_GOAL_POSITION    4
#define LENGTH_PRESENT_POSITION 4
#define LENGTH_GOAL_VELOCITY    4
#define LENGTH_PRESENT_VELOCITY 4
#define LENGTH_GOAL_CURRENT     2
#define LENGTH_PRESENT_CURRENT  2
#define LENGTH_PRESENT_VALUE    10
#define LENGTH_PROFILE_VELOCITY 4

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Control value
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define OPERATING_MODE_CURRENT  0
#define OPERATING_MODE_VELOCITY 1
#define OPERATING_MODE_POSITION 3
#define DEFAULT_POSITION_P_GAIN 800
#define DEFAULT_POSITION_I_GAIN 100
#define DEFAULT_POSITION_D_GAIN 300
#define DEFAULT_VELOCITY_P_GAIN 100
#define DEFAULT_VELOCITY_I_GAIN 1920
#define PROFILE_VELOCITY        60

// Default setting
#define BAUDRATE 3000000           // Default Baudrate of DYNAMIXEL X series
#define SERIAL_PORT "/dev/ttyUSB0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

//// Definition of crane-x7 ////
#define XM540_W270_JOINT 1 // only 2nd joint servo motor is XM540_W270 (other XM430_W350)
#ifndef JOINT_NUM
#define JOINT_NUM 8
#endif
#ifndef PI
#define PI 3.14159265
#endif

#define MAX_DATA 10000

#define ESC_ASCII_VALUE 0x1b

// Unit conversion
#define DXL_VALUE_TO_RADIAN ((2 * PI) / 4096)
#define DXL_VALUE_TO_ANGULARVEL ((0.229 * 2 * PI) / 60)
#define DXL_VALUE_TO_CURRENT (0.00269)
#define TORQUE_CORRECTION_FACTOR (1.3)
#define CURRENT_TO_TORQUE_XM430W350 (1.783 * TORQUE_CORRECTION_FACTOR)
#define CURRENT_TO_TORQUE_XM540W270 (2.409 * TORQUE_CORRECTION_FACTOR)

//// Prototype declaration ////
int initilizeCranex7(uint8_t *);
int setCranex7TorqueState(double *);
int setCranex7Angle(double *);
int setCranex7AngularVelocity(double *);
int setCranex7Torque(double *);
int getCranex7JointState(double *, double *, double *);
int getCranex7Position(double *);
int getCranex7Velocity(double *);
int getCranex7Torque(double *);
void brakeCranex7Joint(void);
void closeCranex7Port(void);
void safe_start(int *);
int AddSyncReadParam(void);

#endif