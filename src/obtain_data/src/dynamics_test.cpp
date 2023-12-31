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

int main()
{
    // Define th0 to th7 and omg0 to omg7 as double
    // double th0 = 0.7741;
    // double th1 = 0.1530;
    // double th2 = 1.2308;
    // double th3 = -0.0543;
    // double th4 = -0.0747;
    // double th5 = 0.1403;
    // double th6 = 0.5819;

    // double omg0 = -0.1935;
    // double omg1 = 0.0055;
    // double omg2 = -0.7800;
    // double omg3 = -0.4418;
    // double omg4 = 0.2352;
    // double omg5 = 0.1645;
    // double omg6 = 0.8525;

    // double trq0 = -0.0935;
    // double trq1 = 1.4068;
    // double trq2 = -0.1559;
    // double trq3 = 0.8979;
    // double trq4 = 0.0811;
    // double trq5 = 0.5362;
    // double trq6 = 0.0374;

    /*
    double th0 = -0.670192;
    double th1 = -0.0178416;
    double th2 = 0.840471;
    double th3 = 0.653604;
    double th4 = 0.723583;
    double th5 = 0.163549;
    double th6 = 0.395575;
    std::cout << "th " << th0 << " " << th1 << " " << th2 << " " << th3 << " " << th4 << " " << th5 << " " << th6 << std::endl;

    double omg0 = 1.1271;
    double omg1 = 0.143885;
    double omg2 = 0.935252;
    double omg3 = 0;
    double omg4 = -0.335732;
    double omg5 = -0.431655;
    double omg6 = -0.383693;
    std::cout << "omg " << omg0 << " " << omg1 << " " << omg2 << " " << omg3 << " " << omg4 << " " << omg5 << " " << omg6 << std::endl;

    double trq0 = 0.0498812;
    double trq1 = 0.454911;
    double trq2 = -0.0374109;
    double trq3 = 0.324228;
    double trq4 = -0.0311758;
    double trq5 = -0.392815;
    double trq6 = -0.0249406;
    std::cout << "trq " << trq0 << " " << trq1 << " " << trq2 << " " << trq3 << " " << trq4 << " " << trq5 << " " << trq6 << std::endl;

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

    // Print the values of mot0 for verification
    for (int i = 0; i < mot0.size(); ++i)
    {
        std::cout << "mot0[" << i << "] = " << mot0[i] << std::endl;
    }

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
    mot4[18] = 1.0;                                                                                                                                                                                                           // Positive constant 5 (1.0 for the 5th constant)
    mot4[24] = std::cos(th4) * std::sin(th1) * std::sin(th2) * std::sin(th5) + std::cos(th1) * std::sin(th3) * std::sin(th4) * std::sin(th5) + std::cos(th2) * std::cos(th3) * std::sin(th1) * std::sin(th4) * std::sin(th5); // Param Mot 4

    // // Print the values of mot4 for verification
    // for (int i = 0; i < mot4.size(); ++i)
    // {
    //     std::cout << "mot4[" << i << "] = " << mot4[i] << std::endl;
    // }

    // Assign values to mot5
    mot5[10] = std::copysign(1.0, omg5);                                                                                                                                                                                                                                                                                                      // 3rd axis friction term 11 Vel 6
    mot5[11] = std::copysign(1.0, omg5) * std::sqrt(std::fabs(omg5));                                                                                                                                                                                                                                                                         // 3rd axis friction term 12 Vel 6
    mot5[19] = 1.0;                                                                                                                                                                                                                                                                                                                           // Positive constant 6 (1.0 for the 6th constant)
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

    std::cout << "trqS" << std::endl
              << trqS << std::endl;

    // Eigen::Matrix<double, 1, Eigen::Dynamic> v2(25);

    // Eigen::VectorXd v1(25);
    // v1 << 1, 0, 0, 0, 3, 4, 5, 6, 7, 8, 9, 2, 4, 5, 1, 3, 4, 5, 6, 7, 1, 4, 5, 3, 4;

    // Declare the motS matrices vertically
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> motS(7, 25);

    // motS.row(0) << mot0.transpose();
    // motS.row(1) << mot1.transpose();
    // motS.row(2) << mot2.transpose();
    // motS.row(3) << mot3.transpose();
    // motS.row(4) << mot4.transpose();
    // motS.row(5) << mot5.transpose();
    // motS.row(6) << mot6.transpose();

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

    std::cout << "motS" << std::endl
              << motS << std::endl;

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

    param.row(0) << 0.0048;
    param.row(1) << 0.0182;
    param.row(2) << 0.3378;
    param.row(3) << -0.2209;
    param.row(4) << 0.0417;
    param.row(5) << -0.0051;
    param.row(6) << 0.3409;
    param.row(7) << -0.1068;
    param.row(8) << 0.0036;
    param.row(9) << 0.0185;
    param.row(10) << 0.4601;
    param.row(11) << -0.0341;
    param.row(12) << 0.0050;
    param.row(13) << 0.0142;
    param.row(14) << -0.0108;
    param.row(15) << -0.0673;
    param.row(16) << -0.0148;
    param.row(17) << 0.0547;
    param.row(18) << -0.0114;
    param.row(19) << 0.0368;
    param.row(20) << -0.0046;
    param.row(21) << 2.5224;
    param.row(22) << 1.0153;
    param.row(23) << 0.7370;
    param.row(24) << -0.0314;

    // std::cout << "param" << std::endl
    //           << param << std::endl;

    // Calculate trqT (Estimated gravity torque)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> trqT(7, 1);

    trqT = motS * param;
    std::cout << "trqT" << std::endl
              << trqT << std::endl;

    */

    // Split trqT in
    // trqT6 = ...to est_torque_array based on the division
    // for (int i = 0; i < 7; i++)
    // {
    //     est_torque_array[i] = trqT(i);
    // }


    double err_torque(7);
    
    return 0;
}
