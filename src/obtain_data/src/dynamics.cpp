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

#define EIGEN_INITIALIZE_MATRICES_BY_NAN

double computeFrictionTerm(double omg)
{
    // Check if omg is 0, return 0.0 in that case, otherwise, use std::copysign
    return (omg == 0.0) ? 0.0 : std::copysign(1.0, omg);
}

/**
 * @fn int getCranex7EstimatedTorque(double *, double *, double *, double *)
 * @brief Function to get joint torque estimation
 * @param[in] angle_array[] input angle array
 * @param[in] vel_array[] input feedback velocity array
 * @param[in] torque_array[] input feedback torque array
 * @param[out] est_torque_array[] output estimated torque array
 * @return Success or failure.
 */
int getCranex7EstimatedTorque(double *angle_array, double *vel_array, double *torque_array, double *est_torque_array)
{
    // Define th0 to th7 and omg0 to omg7 as double
    double th0 = angle_array[0];
    double th1 = angle_array[1];
    double th2 = angle_array[2];
    double th3 = angle_array[3];
    double th4 = angle_array[4];
    double th5 = angle_array[5];
    double th6 = angle_array[6];

    // std::cout << "th " << th0 << " " << th1 << " " << th2 << " " << th3 << " " << th4 << " " << th5 << " " << th6 << std::endl;

    double omg0 = vel_array[0];
    double omg1 = vel_array[1];
    double omg2 = vel_array[2];
    double omg3 = vel_array[3];
    double omg4 = vel_array[4];
    double omg5 = vel_array[5];
    double omg6 = vel_array[6];

    // std::cout << "omg " << omg0 << " " << omg1 << " " << omg2 << " " << omg3 << " " << omg4 << " " << omg5 << " " << omg6 << std::endl;

    double trq0 = torque_array[0];
    double trq1 = torque_array[1];
    double trq2 = torque_array[2];
    double trq3 = torque_array[3];
    double trq4 = torque_array[4];
    double trq5 = torque_array[5];
    double trq6 = torque_array[6];

    // std::cout << "trq " << trq0 << " " << trq1 << " " << trq2 << " " << trq3 << " " << trq4 << " " << trq5 << " " << trq6 << std::endl;

    // Assuming you have populated th0 to th6 and omg0 to omg6 variables
    // Eigen::VectorXd mot0(25);
    // Eigen::VectorXd mot1(25);
    // Eigen::VectorXd mot2(25);
    // Eigen::VectorXd mot3(25);
    // Eigen::VectorXd mot4(25);
    // Eigen::VectorXd mot5(25);
    // Eigen::VectorXd mot6(25);

    Eigen::VectorXd mot0 = Eigen::VectorXd::Zero(25);
    Eigen::VectorXd mot1 = Eigen::VectorXd::Zero(25);
    Eigen::VectorXd mot2 = Eigen::VectorXd::Zero(25);
    Eigen::VectorXd mot3 = Eigen::VectorXd::Zero(25);
    Eigen::VectorXd mot4 = Eigen::VectorXd::Zero(25);
    Eigen::VectorXd mot5 = Eigen::VectorXd::Zero(25);
    Eigen::VectorXd mot6 = Eigen::VectorXd::Zero(25);

    // std::vector<double> mot7(26, 0.0);

    // std::cout << "mot0.size() = " << mot0.size() << std::endl;

    // Assign values to mot0
    mot0[0] = computeFrictionTerm(omg0);                              // 0st axis friction term 1
    mot0[1] = computeFrictionTerm(omg0) * std::sqrt(std::fabs(omg0)); // 0st axis friction term 2
    mot0[14] = 1.0;                                                   // Positive constant 1 (Note: MATLAB index 15 corresponds to C++ index 14)

    // Print the values of mot0 for verification
    // for (int i = 0; i < mot0.size(); ++i)
    // {
    //     std::cout << "mot0[" << i << "] = " << mot0[i] << std::endl;
    // }

    // Assign values to mot1
    mot1[2] = computeFrictionTerm(omg1);                                                                                                                                                                                                                                                                       // 1st axis friction term 3 Vel 2
    mot1[3] = computeFrictionTerm(omg1) * std::sqrt(std::fabs(omg1));                                                                                                                                                                                                                                          // 1st axis friction term 4 Vel 2                                                                                                                                                                                                                                  // 1st axis friction term 4 Vel 2
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
    mot2[4] = computeFrictionTerm(omg2);                                                                                                                                                                                      // 1st axis friction term 3 Vel 2
    mot2[5] = computeFrictionTerm(omg2) * std::sqrt(std::fabs(omg2));                                                                                                                                                         // 2nd axis friction term 6 Vel 3
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
    mot3[6] = computeFrictionTerm(omg3);                                                                                                                                                                                                                                       // 1st axis friction term 7 Vel 4
    mot3[7] = computeFrictionTerm(omg3) * std::sqrt(std::fabs(omg3));                                                                                                                                                                                                          // 3rd axis friction term 8 Vel 4
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
    mot4[8] = computeFrictionTerm(omg4);                                                                                                                                                                                      // 3rd axis friction term 9 Vel 5
    mot4[9] = computeFrictionTerm(omg4) * std::sqrt(std::fabs(omg4));                                                                                                                                                         // 3rd axis friction term 10 Vel 5
    mot4[18] = 1.0;                                                                                                                                                                                                           // Positive constant 5 (1.0 for the 5th constant)
    mot4[24] = std::cos(th4) * std::sin(th1) * std::sin(th2) * std::sin(th5) + std::cos(th1) * std::sin(th3) * std::sin(th4) * std::sin(th5) + std::cos(th2) * std::cos(th3) * std::sin(th1) * std::sin(th4) * std::sin(th5); // Param Mot 4

    // // Print the values of mot4 for verification
    // for (int i = 0; i < mot4.size(); ++i)
    // {
    //     std::cout << "mot4[" << i << "] = " << mot4[i] << std::endl;
    // }

    // Assign values to mot5
    mot5[10] = computeFrictionTerm(omg5);                                                                                                                                                                                                                                                                                                     // 3rd axis friction term 11 Vel 6
    mot5[11] = computeFrictionTerm(omg5) * std::sqrt(std::fabs(omg5));                                                                                                                                                                                                                                                                        // 3rd axis friction term 12 Vel 6
    mot5[19] = 1.0;                                                                                                                                                                                                                                                                                                                           // Positive constant 6 (1.0 for the 6th constant)
    mot5[24] = std::cos(th2) * std::sin(th1) * std::sin(th3) * std::sin(th5) - std::cos(th1) * std::cos(th3) * std::sin(th5) - std::cos(th1) * std::cos(th4) * std::cos(th5) * std::sin(th3) + std::cos(th5) * std::sin(th1) * std::sin(th2) * std::sin(th4) - std::cos(th2) * std::cos(th3) * std::cos(th4) * std::cos(th5) * std::sin(th1); // Param Mot 4

    // // Print the values of mot5 for verification
    // for (int i = 0; i < mot5.size(); ++i)
    // {
    //     std::cout << "mot5[" << i << "] = " << mot5[i] << std::endl;
    // }

    // Assign values to mot6
    mot6[12] = computeFrictionTerm(omg6);                              // 3rd axis friction term 13 Vel 5
    mot6[13] = computeFrictionTerm(omg6) * std::sqrt(std::fabs(omg6)); // 3rd axis friction term 14 Vel 5
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

    motS.row(0) << mot0.transpose();
    motS.row(1) << mot1.transpose();
    motS.row(2) << mot2.transpose();
    motS.row(3) << mot3.transpose();
    motS.row(4) << mot4.transpose();
    motS.row(5) << mot5.transpose();
    motS.row(6) << mot6.transpose();

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
    // Eigen::MatrixXd param = motS_pinv * trqS;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> param(25, 1);

    // param.row(0) << 0.0137;
    // param.row(1) << 0.0103;
    // param.row(2) << 0.0033;
    // param.row(3) << 0.7529;
    // param.row(4) << 0.0190;
    // param.row(5) << 0.0450;
    // param.row(6) << 0.0825;
    // param.row(7) << 0.2702;
    // param.row(8) << 0.0090;
    // param.row(9) << 0.0165;
    // param.row(10) << 0.1579;
    // param.row(11) << 0.2953;
    // param.row(12) << 0.0146;
    // param.row(13) << 0.0101;
    // param.row(14) << -0.0127;
    // param.row(15) << -0.1573;
    // param.row(16) << -0.0095;
    // param.row(17) << -0.1681;
    // param.row(18) << -0.0152;
    // param.row(19) << 0.0374;
    // param.row(20) << -0.0105;
    // param.row(21) << 2.5918;
    // param.row(22) << 1.2529;
    // param.row(23) << 0.7630;
    // param.row(24) << -0.1139;

    // param.row(0) << 0.0266;
    // param.row(1) << 0.0373;
    // param.row(2) << 0.2411;
    // param.row(3) << 0.2011;
    // param.row(4) << 0.0086;
    // param.row(5) << 0.0634;
    // param.row(6) << 0.1223;
    // param.row(7) << 0.2519;
    // param.row(8) << 0.0044;
    // param.row(9) << 0.0213;
    // param.row(10) << 0.2356;
    // param.row(11) << 0.2301;
    // param.row(12) << 0.0094;
    // param.row(13) << 0.0117;
    // param.row(14) << -0.0186;
    // param.row(15) << -0.1304;
    // param.row(16) << -0.0150;
    // param.row(17) << -0.0415;
    // param.row(18) << -0.0157;
    // param.row(19) << 0.0284;
    // param.row(20) << -0.0069;
    // param.row(21) << 1.1098;
    // param.row(22) << 1.1377;
    // param.row(23) << 0.4773;
    // param.row(24) << 0.0026;

    // param.row(0) << 0.0048;
    // param.row(1) << 0.0182;
    // param.row(2) << 0.3378;
    // param.row(3) << -0.2209;
    // param.row(4) << 0.0417;
    // param.row(5) << -0.0051;
    // param.row(6) << 0.3409;
    // param.row(7) << -0.1068;
    // param.row(8) << 0.0036;
    // param.row(9) << 0.0185;
    // param.row(10) << 0.4601;
    // param.row(11) << -0.0341;
    // param.row(12) << 0.0050;
    // param.row(13) << 0.0142;
    // param.row(14) << -0.0108;
    // param.row(15) << -0.0673;
    // param.row(16) << -0.0148;
    // param.row(17) << 0.0547;
    // param.row(18) << -0.0114;
    // param.row(19) << 0.0368;
    // param.row(20) << -0.0046;
    // param.row(21) << 2.5224;
    // param.row(22) << 1.0153;
    // param.row(23) << 0.7370;
    // param.row(24) << -0.0314;

    param.row(0) << 0.0156;
    param.row(1) << -0.0101;
    param.row(2) << 0.3447;
    param.row(3) << -0.1063;
    param.row(4) << 0.0466;
    param.row(5) << 0.0019;
    param.row(6) << 0.3531;
    param.row(7) << -0.0747;
    param.row(8) << 0.0160;
    param.row(9) << 0.0011;
    param.row(10) << 0.4605;
    param.row(11) << -0.0513;
    param.row(12) << 0.0109;
    param.row(13) << 0.0064;
    param.row(14) << -0.0022;
    param.row(15) << -0.1126;
    param.row(16) << -0.0101;
    param.row(17) << -0.0317;
    param.row(18) << -0.0030;
    param.row(19) << 0.0243;
    param.row(20) << 0.0024;
    param.row(21) << 2.9188;
    param.row(22) << -0.0051;
    param.row(23) << 1.3449;
    param.row(24) << 0.0123;

    // std::cout << "param" << std::endl
    //           << param << std::endl;

    // Calculate trqT (Estimated gravity torque)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> trqT(7, 1);

    trqT = motS * param;

    // std::cout << "trqT" << std::endl
    //           << trqT << std::endl;

    // Split trqT in
    // trqT6 = ...to est_torque_array based on the division
    for (int i = 0; i < 7; i++)
    {
        est_torque_array[i] = trqT(i);
    }

    motS = Eigen::MatrixXd::Zero(7, 25);
    trqT = Eigen::MatrixXd::Zero(7, 1);

    return 0;
}

/**
 * @fn int getCranex7EstimatedExtForces(double *)
 * @brief Function to get external force estimation from error torque
 * @param[in] angle_array[] input angle array
 * @param[in] torque_array[] input torque array
 * @param[out] est_force_array[] output estimated force array
 * @return Success or failure.
 */
int getCranex7EstimatedExtForces(double *angle_array, double *torque_array, double *est_force_array)
{
    // Link array declaration
    Eigen::VectorXd link(8);

    // link << 41, 105, 170, 355, 476, 605, 624;
    // link << 41, 64, 65, 185, 121, 129, 19, 84;
    // link << 4.1, 6.4, 6.5, 18.5, 12.1, 12.9, 1.9, 8.4;

    link << 0.041, 0.064, 0.065, 0.185, 0.121, 0.129, 0.019, 0.084;

    // Homogeneous Transformation Matrix
    Eigen::MatrixXd H0_0(4, 4);
    Eigen::MatrixXd H0_1(4, 4);
    Eigen::MatrixXd H1_2(4, 4);
    Eigen::MatrixXd H2_3(4, 4);
    Eigen::MatrixXd H3_4(4, 4);
    Eigen::MatrixXd H4_5(4, 4);
    Eigen::MatrixXd H5_6(4, 4);
    Eigen::MatrixXd H6_7(4, 4);

    H0_0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, link[0],
        0, 0, 0, 1;

    H0_1 << cos(angle_array[0]), 0, sin(angle_array[0]), 0,
        sin(angle_array[0]), 0, -cos(angle_array[0]), 0,
        0, 1, 0, link[1],
        0, 0, 0, 1;

    H1_2 << cos(angle_array[1]), 0, -sin(angle_array[1]), link[2] * cos(angle_array[1]),
        sin(angle_array[1]), 0, cos(angle_array[1]), link[2] * sin(angle_array[1]),
        0, -1, 0, 0,
        0, 0, 0, 1;

    H2_3 << cos(angle_array[2]), 0, sin(angle_array[2]), 0,
        sin(angle_array[2]), 0, -cos(angle_array[2]), 0,
        0, 1, 1, link[3],
        0, 0, 0, 1;

    H3_4 << cos(angle_array[3]), 0, -sin(angle_array[3]), link[4] * cos(angle_array[3]),
        sin(angle_array[3]), 0, cos(angle_array[3]), link[4] * sin(angle_array[3]),
        0, -1, 0, 0,
        0, 0, 0, 1;

    H4_5 << cos(angle_array[4]), 0, sin(angle_array[4]), 0,
        sin(angle_array[4]), 0, -cos(angle_array[4]), 0,
        0, 1, 0, link[5],
        0, 0, 0, 1;

    H5_6 << cos(angle_array[5]), 0, -sin(angle_array[5]), link[6] * cos(angle_array[5]),
        sin(angle_array[5]), 0, cos(angle_array[5]), link[6] * sin(angle_array[5]),
        0, -1, 0, 0,
        0, 0, 0, 1;

    H6_7 << cos(angle_array[6]), -sin(angle_array[6]), 0, 0,
        sin(angle_array[6]), cos(angle_array[6]), 0, 0,
        0, 0, 1, link[7],
        0, 0, 0, 1;

    Eigen::MatrixXd JH0_0(4, 4);
    Eigen::MatrixXd JH0_1(4, 4);
    Eigen::MatrixXd JH0_2(4, 4);
    Eigen::MatrixXd JH0_3(4, 4);
    Eigen::MatrixXd JH0_4(4, 4);
    Eigen::MatrixXd JH0_5(4, 4);
    Eigen::MatrixXd JH0_6(4, 4);
    Eigen::MatrixXd JH0_7(4, 4);

    JH0_0 = H0_0;
    JH0_1 = H0_0 * H0_1;
    JH0_2 = H0_0 * H0_1 * H1_2;
    JH0_3 = H0_0 * H0_1 * H1_2 * H2_3;
    JH0_4 = H0_0 * H0_1 * H1_2 * H2_3 * H3_4;
    JH0_5 = H0_0 * H0_1 * H1_2 * H2_3 * H3_4 * H4_5;
    JH0_6 = H0_0 * H0_1 * H1_2 * H2_3 * H3_4 * H4_5 * H5_6;
    JH0_7 = H0_0 * H0_1 * H1_2 * H2_3 * H3_4 * H4_5 * H5_6 * H6_7;

    // Create the Displacement Matrix for Jacobian Matrix
    Eigen::MatrixXd d0_0(3, 1);
    Eigen::MatrixXd d0_1(3, 1);
    Eigen::MatrixXd d0_2(3, 1);
    Eigen::MatrixXd d0_3(3, 1);
    Eigen::MatrixXd d0_4(3, 1);
    Eigen::MatrixXd d0_5(3, 1);
    Eigen::MatrixXd d0_6(3, 1);
    Eigen::MatrixXd d0_7(3, 1);

    d0_0 << JH0_0(0, 3),
        JH0_0(1, 3),
        JH0_0(2, 3);

    d0_1 << JH0_1(0, 3),
        JH0_1(1, 3),
        JH0_1(2, 3);

    d0_2 << JH0_2(0, 3),
        JH0_2(1, 3),
        JH0_2(2, 3);

    d0_3 << JH0_3(0, 3),
        JH0_3(1, 3),
        JH0_3(2, 3);

    d0_4 << JH0_4(0, 3),
        JH0_4(1, 3),
        JH0_4(2, 3);

    d0_5 << JH0_5(0, 3),
        JH0_5(1, 3),
        JH0_5(2, 3);

    d0_6 << JH0_6(0, 3),
        JH0_6(1, 3),
        JH0_6(2, 3);

    d0_7 << JH0_7(0, 3),
        JH0_7(1, 3),
        JH0_7(2, 3);

    Eigen::MatrixXd Jd0_0(3, 1);
    Eigen::MatrixXd Jd0_1(3, 1);
    Eigen::MatrixXd Jd0_2(3, 1);
    Eigen::MatrixXd Jd0_3(3, 1);
    Eigen::MatrixXd Jd0_4(3, 1);
    Eigen::MatrixXd Jd0_5(3, 1);
    Eigen::MatrixXd Jd0_6(3, 1);

    Jd0_0 = d0_7 - d0_0;
    Jd0_1 = d0_7 - d0_1;
    Jd0_2 = d0_7 - d0_2;
    Jd0_3 = d0_7 - d0_3;
    Jd0_4 = d0_7 - d0_4;
    Jd0_5 = d0_7 - d0_5;
    Jd0_6 = d0_7 - d0_6;

    // Create the Rotation Matrix for Jacobian Matrix
    Eigen::MatrixXd JR0_0(3, 1);
    Eigen::MatrixXd JR0_1(3, 1);
    Eigen::MatrixXd JR0_2(3, 1);
    Eigen::MatrixXd JR0_3(3, 1);
    Eigen::MatrixXd JR0_4(3, 1);
    Eigen::MatrixXd JR0_5(3, 1);
    Eigen::MatrixXd JR0_6(3, 1);

    JR0_0 << JH0_0(0, 2),
        JH0_0(1, 2),
        JH0_0(2, 2);

    JR0_1 << JH0_1(0, 2),
        JH0_1(1, 2),
        JH0_1(2, 2);

    JR0_2 << JH0_2(0, 2),
        JH0_2(1, 2),
        JH0_2(2, 2);

    JR0_3 << JH0_3(0, 2),
        JH0_3(1, 2),
        JH0_3(2, 2);

    JR0_4 << JH0_4(0, 2),
        JH0_4(1, 2),
        JH0_4(2, 2);

    JR0_5 << JH0_5(0, 2),
        JH0_5(1, 2),
        JH0_5(2, 2);

    JR0_6 << JH0_6(0, 2),
        JH0_6(1, 2),
        JH0_6(2, 2);

    // Cross Product
    // std::cout << "Cross Product" << std::endl;

    Eigen::Vector3d JR0_0_cross(JR0_0(0, 0), JR0_0(1, 0), JR0_0(2, 0));
    Eigen::Vector3d JR0_1_cross(JR0_1(0, 0), JR0_1(1, 0), JR0_1(2, 0));
    Eigen::Vector3d JR0_2_cross(JR0_2(0, 0), JR0_2(1, 0), JR0_2(2, 0));
    Eigen::Vector3d JR0_3_cross(JR0_3(0, 0), JR0_3(1, 0), JR0_3(2, 0));
    Eigen::Vector3d JR0_4_cross(JR0_4(0, 0), JR0_4(1, 0), JR0_4(2, 0));
    Eigen::Vector3d JR0_5_cross(JR0_5(0, 0), JR0_5(1, 0), JR0_5(2, 0));
    Eigen::Vector3d JR0_6_cross(JR0_6(0, 0), JR0_6(1, 0), JR0_6(2, 0));

    Eigen::Vector3d Jd0_0_cross(Jd0_0(0, 0), Jd0_0(1, 0), Jd0_0(2, 0));
    Eigen::Vector3d Jd0_1_cross(Jd0_1(0, 0), Jd0_1(1, 0), Jd0_1(2, 0));
    Eigen::Vector3d Jd0_2_cross(Jd0_2(0, 0), Jd0_2(1, 0), Jd0_2(2, 0));
    Eigen::Vector3d Jd0_3_cross(Jd0_3(0, 0), Jd0_3(1, 0), Jd0_3(2, 0));
    Eigen::Vector3d Jd0_4_cross(Jd0_4(0, 0), Jd0_4(1, 0), Jd0_4(2, 0));
    Eigen::Vector3d Jd0_5_cross(Jd0_5(0, 0), Jd0_5(1, 0), Jd0_5(2, 0));
    Eigen::Vector3d Jd0_6_cross(Jd0_6(0, 0), Jd0_6(1, 0), Jd0_6(2, 0));

    Eigen::Vector3d J0 = JR0_0_cross.cross(Jd0_0_cross);
    Eigen::Vector3d J1 = JR0_1_cross.cross(Jd0_1_cross);
    Eigen::Vector3d J2 = JR0_2_cross.cross(Jd0_2_cross);
    Eigen::Vector3d J3 = JR0_3_cross.cross(Jd0_3_cross);
    Eigen::Vector3d J4 = JR0_4_cross.cross(Jd0_4_cross);
    Eigen::Vector3d J5 = JR0_5_cross.cross(Jd0_5_cross);
    Eigen::Vector3d J6 = JR0_6_cross.cross(Jd0_6_cross);

    // std::cout << "J5" << std::endl
    //           << J5 << std::endl;

    // return 0;

    // Jacobian matrix
    // std::cout << "Jacobian matrix" << std::endl;

    Eigen::MatrixXd J(6, 7);

    J << J0(0, 0), J1(0, 0), J2(0, 0), J3(0, 0), J4(0, 0), J5(0, 0), J6(0, 0),
        J0(1, 0), J1(1, 0), J2(1, 0), J3(1, 0), J4(1, 0), J5(1, 0), J6(1, 0),
        J0(2, 0), J1(2, 0), J2(2, 0), J3(2, 0), J4(2, 0), J5(2, 0), J6(2, 0),
        JR0_0(0, 0), JR0_1(0, 0), JR0_2(0, 0), JR0_3(0, 0), JR0_4(0, 0), JR0_5(0, 0), JR0_6(0, 0),
        JR0_0(1, 0), JR0_1(1, 0), JR0_2(1, 0), JR0_3(1, 0), JR0_4(1, 0), JR0_5(1, 0), JR0_6(1, 0),
        JR0_0(2, 0), JR0_1(2, 0), JR0_2(2, 0), JR0_3(2, 0), JR0_4(2, 0), JR0_5(2, 0), JR0_6(2, 0);
    ;

    // J << J0(0, 0), J1(0, 0), J2(0, 0), J3(0, 0), J4(0, 0), J5(0, 0),
    //     J0(1, 0), J1(1, 0), J2(1, 0), J3(1, 0), J4(1, 0), J5(1, 0),
    //     J0(2, 0), J1(2, 0), J2(2, 0), J3(2, 0), J4(2, 0), J5(2, 0),
    //     JR0_0(0, 0), JR0_1(0, 0), JR0_2(0, 0), JR0_3(0, 0), JR0_4(0, 0), JR0_5(0, 0),
    //     JR0_0(1, 0), JR0_1(1, 0), JR0_2(1, 0), JR0_3(1, 0), JR0_4(1, 0), JR0_5(1, 0),
    //     JR0_0(2, 0), JR0_1(2, 0), JR0_2(2, 0), JR0_3(2, 0), JR0_4(2, 0), JR0_5(2, 0);
    //;

    // std::cout << "J.size() = " << J.size() << std::endl;
    // std::cout << "J.rows() = " << J.rows() << std::endl;
    // std::cout << "J.cols() = " << J.cols() << std::endl;

    // std::cout << "J" << std::endl
    //           << J << std::endl;

    // std::cout << " EoE Pos || "
    //           << " Xeoe : " << JH0_7(0, 3) << " || Yeoe : " << JH0_7(1, 3) << " || Zeoe : " << JH0_7(2, 3) << std::endl;

    Eigen::VectorXd tau(7);

    tau << torque_array[0], torque_array[1], torque_array[2], torque_array[3], torque_array[4], torque_array[5], torque_array[6];

    // std::cout << "tau" << std::endl
    //           << tau << std::endl;

    //////////////// Create Pseudo Inverse of Jacobian matrix ////////////////
    // std::cout << "Pseudo Inverse Jacobian matrix" << std::endl;

    // Eigen::MatrixXd J_PseudoInv = J.completeOrthogonalDecomposition().pseudoInverse();
    // Eigen::MatrixXd J_PseudoInv = J.transpose() * (J * J.transpose()).inverse();

    // std::cout << "The determinant of J * J.transpose() is " << (J * J.transpose()).determinant() << std::endl;

    // std::cout << "J_PseudoInv.size() = " << J_PseudoInv.size() << std::endl;
    // std::cout << "J_PseudoInv.rows() = " << J_PseudoInv.rows() << std::endl;
    // std::cout << "J_PseudoInv.cols() = " << J_PseudoInv.cols() << std::endl;

    // std::cout << "J_PseudoInv" << std::endl
    //           << J_PseudoInv << std::endl;

    // std::cout << "Jacobian Matrix (J) dimensions: " << J.rows() << " x " << J.cols() << std::endl;
    // std::cout << "Joint Torques (tau) dimensions: " << tau.rows() << " x " << tau.cols() << std::endl;

    // Eigen::VectorXd F = J_PseudoInv.transpose().completeOrthogonalDecomposition().solve(Torque);

    //////////////// Calculate end-effector forces using DLS ////////////////
    double damping_factor = 0.01; // Need to adjust this based on your system

    Eigen::MatrixXd J_pseudo_inv = (J.transpose() * J + damping_factor * Eigen::MatrixXd::Identity(7, 7)).inverse() * J.transpose();
    // Eigen::VectorXd F = J_pseudo_inv * Torque;
    Eigen::VectorXd F = J_pseudo_inv.completeOrthogonalDecomposition().solve(tau);

    for (int i = 0; i < 6; i++)
    {
        est_force_array[i] = F(i);
    }
}
