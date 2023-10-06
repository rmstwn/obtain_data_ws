#ifndef OBTAIN_DATA_HPP_
#define OBTAIN_DATA_HPP_

#include <stdio.h>
#include "obtain_data/crane_x7_comm.hpp"

extern double present_theta2[JOINT_NUM];
extern double present_angvel2[JOINT_NUM];
extern double present_current2[JOINT_NUM];

class JointState
{
public:
    // JointState()
    // {
    //     for (int i = 0; i < JOINT_NUM; ++i)
    //     {
    //         present_theta[i] = 1.0;
    //         present_angvel[i] = 1.0;
    //         present_current[i] = 1.0;
    //     }
    // }
    double present_theta[JOINT_NUM];
    double present_angvel[JOINT_NUM];
    double present_current[JOINT_NUM];

    // Getters and setters for joint data
    double getTheta(int jointIndex) const
    {
        return present_theta[jointIndex];
    }

    void setTheta(int jointIndex, double value)
    {
        present_theta[jointIndex] = value;
    }

    double getAngVel(int jointIndex) const
    {
        return present_angvel[jointIndex];
    }

    void setAngVel(int jointIndex, double value)
    {
        present_angvel[jointIndex] = value;
    }

    double getCurrent(int jointIndex) const
    {
        return present_current[jointIndex];
    }

    void setCurrent(int jointIndex, double value)
    {
        present_current[jointIndex] = value;
    }

    // private:
    //     double present_theta[JOINT_NUM];
    //     double present_angvel[JOINT_NUM];
    //     double present_current[JOINT_NUM];
};

extern JointState jointState;

#endif // OBTAIN_DATA_HPP_