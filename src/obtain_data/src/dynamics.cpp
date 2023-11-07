#include <vector>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include "obtain_data/crane_x7_comm.hpp"
#include "obtain_data/dynamics.hpp"

/**
 * @fn int getCranex7EstimatedTorque(double *)
 * @brief Function to get joint torque estimation
 * @param[in] angle_array[] input angle array
 * @param[in] vel_array[] input feedback velocity array
 * @param[in] torque_array[] input feedback torque array
 * @param[out] est_torque_array[] output estimated torque array
 * @return Success or failure.
 */

int getCranex7EstimatedTorque(double *angle_array, double *vel_array, double *torque_array, double *est_torque_array)
{
    // int len = 10; // Replace with your desired length

    // Define th0 to th7 and omg0 to omg7 as double
    double th0 = angle_array[0];
    double th1 = angle_array[1];
    double th2 = angle_array[2];
    double th3 = angle_array[3];
    double th4 = angle_array[4];
    double th5 = angle_array[5];
    double th6 = angle_array[6];
    // double th7 = 0;

    double omg0 = vel_array[0];
    double omg1 = vel_array[1];
    double omg2 = vel_array[2];
    double omg3 = vel_array[3];
    double omg4 = vel_array[4];
    double omg5 = vel_array[5];
    double omg6 = vel_array[6];
    // double omg7 = 0;

    double trq0 = torque_array[0];
    double trq1 = torque_array[1];
    double trq2 = torque_array[2];
    double trq3 = torque_array[3];
    double trq4 = torque_array[4];
    double trq5 = torque_array[5];
    double trq6 = torque_array[6];
    // double trq7 = 0;

    // Assuming you have populated th0 to th6 and omg0 to omg6 variables
    Eigen::VectorXd mot0(25);
    Eigen::VectorXd mot1(25);
    Eigen::VectorXd mot2(25);
    Eigen::VectorXd mot3(25);
    Eigen::VectorXd mot4(25);
    Eigen::VectorXd mot5(25);
    Eigen::VectorXd mot6(25);
    // std::vector<double> mot7(26, 0.0);

    // std::cout << "mot0.size() = " << mot0.size() << std::endl;

    // Assign values to mot0
    mot0[0] = std::copysign(1.0, omg0);                              // 0st axis friction term 1
    mot0[1] = std::copysign(1.0, omg0) * std::sqrt(std::fabs(omg0)); // 0st axis friction term 2
    mot0[14] = 1.0;                                                  // Positive constant 1 (Note: MATLAB index 15 corresponds to C++ index 14)

    // // Print the values of mot0 for verification
    // for (int i = 0; i < mot0.size(); ++i)
    // {
    //     std::cout << "mot0[" << i << "] = " << mot0[i] << std::endl;
    // }

    // Assign values to mot1
    mot1[2] = std::copysign(1.0, omg1);                                                                                                                                                                                                                                                                        // 1st axis friction term 3 Vel 2
    mot1[3] = std::copysign(1.0, omg1) * std::sqrt(std::fabs(omg1));                                                                                                                                                                                                                                           // 1st axis friction term 4 Vel 2                                                                                                                                                                                                                                  // 1st axis friction term 4 Vel 2
    mot1[15] = 1.0;                                                                                                                                                                                                                                                                                            // Positive constant 2
    mot1[21] = -std::sin(th1);                                                                                                                                                                                                                                                                                 // Param Mot 1
    mot1[22] = std::cos(th1) * std::cos(th2) * std::cos(th3) - std::sin(th1) * std::sin(th3);                                                                                                                                                                                                                  // Param Mot 2
    mot1[23] = -std::cos(th3) * std::sin(th1) - std::cos(th1) * std::cos(th2) * std::sin(th3);                                                                                                                                                                                                                 // Param Mot 3
    mot1[24] = -std::cos(th3) * std::cos(th5) * std::sin(th1) - std::cos(th2) * std::cos(th5) * std::sin(th3) + std::cos(th1) * std::sin(th2) * std::sin(th4) * std::sin(th5) + std::cos(th4) * std::sin(th1) * std::sin(th3) * std::sin(th5) - std::cos(th2) * std::cos(th3) * std::cos(th4) * std::sin(th5); // Param Mot 4

    // // Print the values of mot1 for verification
    // for (int i = 0; i < mot1.size(); ++i)
    // {
    //     std::cout << "mot1[" << i << "] = " << mot1[i] << std::endl;
    // }

    // Assign values to mot2
    mot2[4] = std::copysign(1.0, omg2);                                                                                                                                                                                       // 1st axis friction term 3 Vel 2
    mot2[5] = std::copysign(1.0, omg2) * std::sqrt(std::fabs(omg2));                                                                                                                                                          // 2nd axis friction term 6 Vel 3
    mot2[16] = 1.0;                                                                                                                                                                                                           // Positive constant 3 (1.0 for the 3rd constant)
    mot2[22] = -std::cos(th3) * std::sin(th1) * std::sin(th2);                                                                                                                                                                // Param Mot 2
    mot2[23] = std::sin(th1) * std::sin(th2) * std::sin(th3);                                                                                                                                                                 // Param Mot 3
    mot2[24] = std::cos(th5) * std::sin(th1) * std::sin(th2) * std::sin(th3) + std::cos(th2) * std::sin(th1) * std::sin(th4) * std::sin(th5) + std::cos(th3) * std::cos(th4) * std::sin(th1) * std::sin(th2) * std::sin(th5); // Param Mot 4

    // // Print the values of mot1 for verification
    // for (int i = 0; i < mot2.size(); ++i)
    // {
    //     std::cout << "mot2[" << i << "] = " << mot2[i] << std::endl;
    // }

    // Assign values to mot3
    mot3[6] = std::copysign(1.0, omg3);                                                                                                                                                                                                                                        // 1st axis friction term 7 Vel 4
    mot3[7] = std::copysign(1.0, omg3) * std::sqrt(std::fabs(omg3));                                                                                                                                                                                                           // 3rd axis friction term 8 Vel 4
    mot3[17] = 1.0;                                                                                                                                                                                                                                                            // Positive constant 4 (1.0 for the 4th constant)
    mot3[22] = std::cos(th1) * std::cos(th3) - std::cos(th2) * std::sin(th1) * std::sin(th3);                                                                                                                                                                                  // Param Mot 2
    mot3[23] = -std::cos(th1) * std::sin(th3) - std::cos(th2) * std::cos(th3) * std::sin(th1);                                                                                                                                                                                 // Param Mot 3
    mot3[24] = -std::cos(th1) * std::cos(th5) * std::sin(th3) - std::cos(th2) * std::cos(th3) * std::cos(th5) * std::sin(th1) - std::cos(th1) * std::cos(th3) * std::cos(th4) * std::sin(th5) + std::cos(th2) * std::cos(th4) * std::sin(th1) * std::sin(th3) * std::sin(th5); // Param Mot 4

    // // Print the values of mot3 for verification
    // for (int i = 0; i < mot3.size(); ++i)
    // {
    //     std::cout << "mot3[" << i << "] = " << mot3[i] << std::endl;
    // }

    // Assign values to mot4
    mot4[8] = std::copysign(1.0, omg4);                                                                                                                                                                                       // 3rd axis friction term 9 Vel 5
    mot4[9] = std::copysign(1.0, omg4) * std::sqrt(std::fabs(omg4));                                                                                                                                                          // 3rd axis friction term 10 Vel 5
    mot4[19] = 1.0;                                                                                                                                                                                                           // Positive constant 5 (1.0 for the 5th constant)
    mot4[24] = std::cos(th4) * std::sin(th1) * std::sin(th2) * std::sin(th5) + std::cos(th1) * std::sin(th3) * std::sin(th4) * std::sin(th5) + std::cos(th2) * std::cos(th3) * std::sin(th1) * std::sin(th4) * std::sin(th5); // Param Mot 4

    // // Print the values of mot4 for verification
    // for (int i = 0; i < mot4.size(); ++i)
    // {
    //     std::cout << "mot4[" << i << "] = " << mot4[i] << std::endl;
    // }

    // Assign values to mot5
    mot5[10] = std::copysign(1.0, omg5);                                                                                                                                                                                                                                                                                                      // 3rd axis friction term 11 Vel 6
    mot5[11] = std::copysign(1.0, omg5) * std::sqrt(std::fabs(omg5));                                                                                                                                                                                                                                                                         // 3rd axis friction term 12 Vel 6
    mot5[20] = 1.0;                                                                                                                                                                                                                                                                                                                           // Positive constant 6 (1.0 for the 6th constant)
    mot5[24] = std::cos(th2) * std::sin(th1) * std::sin(th3) * std::sin(th5) - std::cos(th1) * std::cos(th3) * std::sin(th5) - std::cos(th1) * std::cos(th4) * std::cos(th5) * std::sin(th3) + std::cos(th5) * std::sin(th1) * std::sin(th2) * std::sin(th4) - std::cos(th2) * std::cos(th3) * std::cos(th4) * std::cos(th5) * std::sin(th1); // Param Mot 4

    // // Print the values of mot5 for verification
    // for (int i = 0; i < mot5.size(); ++i)
    // {
    //     std::cout << "mot5[" << i << "] = " << mot5[i] << std::endl;
    // }

    // Assign values to mot6
    mot6[12] = std::copysign(1.0, omg6);                              // 3rd axis friction term 13 Vel 5
    mot6[13] = std::copysign(1.0, omg6) * std::sqrt(std::fabs(omg6)); // 3rd axis friction term 14 Vel 5
    mot6[20] = 1.0;

    // // Print the values of mot6 for verification
    // for (int i = 0; i < mot6.size(); ++i)
    // {
    //     std::cout << "mot6[" << i << "] = " << mot6[i] << std::endl;
    // }
    // std::cout << "mot6.size() = " << mot6.size() << std::endl;

    // Define trqS as a vector (populate with actual values)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> trqS(7, 1);

    trqS.row(0) << trq0;
    trqS.row(1) << trq1;
    trqS.row(2) << trq2;
    trqS.row(3) << trq3;
    trqS.row(4) << trq4;
    trqS.row(5) << trq5;
    trqS.row(6) << trq6;

    // std::cout << "trqS" << std::endl
    //           << trqS << std::endl;

    // Eigen::Matrix<double, 1, Eigen::Dynamic> v2(25);

    // Eigen::VectorXd v1(25);
    // v1 << 1, 0, 0, 0, 3, 4, 5, 6, 7, 8, 9, 2, 4, 5, 1, 3, 4, 5, 6, 7, 1, 4, 5, 3, 4;

    // Declare the motS matrices vertically
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> motS(7, 25);

    motS.row(0) = mot0.transpose();
    motS.row(1) = mot1.transpose();
    motS.row(2) = mot2.transpose();
    motS.row(3) = mot3.transpose();
    motS.row(4) = mot4.transpose();
    motS.row(5) = mot5.transpose();
    motS.row(6) = mot6.transpose();

    // std::cout << "motS.size() = " << motS.size() << std::endl;
    // std::cout << "motS.rows() = " << motS.rows() << std::endl;
    // std::cout << "motS.cols() = " << motS.cols() << std::endl;

    // std::cout << "motS" << std::endl
    //           << motS << std::endl;

    // Create Pseudo Inverse of motS_matrix
    Eigen::MatrixXd motS_pinv = motS.completeOrthogonalDecomposition().pseudoInverse();
    // std::cout << "motS_pinv" << std::endl
    //           << motS_pinv << std::endl;

    // Vector parameters calculation
    Eigen::MatrixXd param = motS_pinv * trqS;
    // std::cout << "param" << std::endl
    //           << param << std::endl;

    // Calculate trqT (Estimated gravity torque)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> trqT(7, 1);

    trqT = motS * param;
    std::cout << "trqT" << std::endl
              << trqT << std::endl;

    // Split trqT in
    // trqT6 = ...to est_torque_array based on the division
    for (int i = 0; i < JOINT_NUM; i++)
    {
        est_torque_array[i] = trqT(i);
    }

    return 0;
}
