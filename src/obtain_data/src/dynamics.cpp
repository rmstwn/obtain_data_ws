#include <vector>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>

int main()
{
    // int len = 10; // Replace with your desired length

    // Define th0 to th7 and omg0 to omg7 as double
    double th0 = 0.4537;
    double th1 = 0.0681;
    double th2 = 0.1722;
    double th3 = -0.4079;
    double th4 = -0.0130;
    double th5 = 0.4341;
    double th6 = 1.6687;
    // double th7 = 0;

    double omg0 = 0.2755;
    double omg1 = 0.0883;
    double omg2 = 0.5520;
    double omg3 = -0.3350;
    double omg4 = 2.7995;
    double omg5 = 1.0280;
    double omg6 = 1.9500;
    // double omg7 = 0;

    double trq0 = 0;
    double trq1 = 1.2636;
    double trq2 = -0.0187;
    double trq3 = 0.8979;
    double trq4 = 0.0499;
    double trq5 = 0.5799;
    double trq6 = 0.0249;
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

    // mot0 = Eigen::VectorXd::Zero(0, mot0.cols());
    // mot1 = Eigen::VectorXd::Zero(0, mot1.cols());
    // mot2 = Eigen::VectorXd::Zero(0, mot2.cols());
    // mot3 = Eigen::VectorXd::Zero(0, mot3.cols());
    // mot4 = Eigen::VectorXd::Zero(0, mot4.cols());
    // mot5 = Eigen::VectorXd::Zero(0, mot5.cols());
    // mot6 = Eigen::VectorXd::Zero(0, mot6.cols());

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

    // Eigen::MatrixXd motS; // Declare a matrix to store the concatenated result

    // // Concatenate the matrices vertically
    // // motS = Eigen::MatrixXd::Zero(0, mot0.cols()); // Initialize an empty matrix to match the dimensions of mot0
    // motS.conservativeResize(mot0.cols() + mot1.cols() + mot2.cols() + mot3.cols() + mot4.cols() + mot5.cols() + mot6.cols(), mot0.rows());

    // std::cout << "motS.size() = " << motS.size() << std::endl;
    // std::cout << "motS.rows() = " << motS.rows() << std::endl;
    // std::cout << "motS.cols() = " << motS.cols() << std::endl;

    // Eigen::Matrix<double, 1, Eigen::Dynamic> v2(25);

    // Eigen::VectorXd v1(25);
    // v1 << 1, 0, 0, 0, 3, 4, 5, 6, 7, 8, 9, 2, 4, 5, 1, 3, 4, 5, 6, 7, 1, 4, 5, 3, 4;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> motS(7, 25);

    motS.row(0) = mot0.transpose();
    motS.row(1) = mot1.transpose();
    motS.row(2) = mot2.transpose();
    motS.row(3) = mot3.transpose();
    motS.row(4) = mot4.transpose();
    motS.row(5) = mot5.transpose();
    motS.row(6) = mot6.transpose();

    // std::cout << "motS" << std::endl
    //           << motS << std::endl;

    // Create Pseudo Inverse of motS_matrix
    Eigen::MatrixXd motS_pinv = motS.completeOrthogonalDecomposition().pseudoInverse();
    // std::cout << "motS_pinv" << std::endl
    //           << motS_pinv << std::endl;

    Eigen::MatrixXd param = motS_pinv * trqS;
    std::cout << "param" << std::endl
              << param << std::endl;

    // // std::cout << "motS.size() = " << motS.size() << std::endl;

    // Eigen::MatrixXd motS;

    // motS = Eigen::MatrixXd::Zero(0, mot0.cols()); // Initialize an empty matrix to match the dimensions of mot0

    // // Calculate para
    // Eigen::MatrixXd motS_matrix(motS.size(), 7);
    // Eigen::MatrixXd trqS_matrix(trqS.size(), 7);

    // // std::cout << "mot0.size() = " << mot0.size() << std::endl;
    // // std::cout << "mot1.size() = " << mot1.size() << std::endl;
    // // std::cout << "mot2.size() = " << mot2.size() << std::endl;
    // // std::cout << "mot3.size() = " << mot3.size() << std::endl;
    // // std::cout << "mot4.size() = " << mot4.size() << std::endl;
    // // std::cout << "mot5.size() = " << mot5.size() << std::endl;
    // // std::cout << "mot6.size() = " << mot6.size() << std::endl;
    // // std::cout << "motS.size() = " << motS.size() << std::endl;
    // // std::cout << "trqS.size() = " << trqS.size() << std::endl;

    // // for (int i = 0; i < motS.size(); ++i)
    // // {
    // //     std::cout << "motS[" << i << "] = " << motS[i] << std::endl;
    // // }

    // for (int i = 0; i < motS.size(); i++)
    // {
    //     motS_matrix(i, 0) = motS[i];
    // }

    // for (int i = 0; i < trqS.size(); i++)
    // {
    //     trqS_matrix(i, 0) = trqS[i];
    // }

    // std::cout << "motS_matrix.size() = " << motS_matrix.size() << std::endl;
    // std::cout << "motS_matrix.rows() = " << motS_matrix.rows() << std::endl;
    // std::cout << "motS_matrix.cols() = " << motS_matrix.cols() << std::endl;

    // // Create Pseudo Inverse of motS_matrix
    // // Eigen::MatrixXd para_matrix = motS_matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(trqS_matrix);
    // Eigen::MatrixXd motS_pinv = motS_matrix.completeOrthogonalDecomposition().pseudoInverse();

    // std::cout << "motS_pinv.size() = " << motS_pinv.size() << std::endl;
    // std::cout << "motS_pinv.rows() = " << motS_pinv.rows() << std::endl;
    // std::cout << "motS_pinv.cols() = " << motS_pinv.cols() << std::endl;

    // Eigen::MatrixXd para_matrix =  motS_pinv * trqS_matrix;

    // Convert para_matrix to a vector
    // std::vector<double> para(para_matrix.data(), para_matrix.data() + para_matrix.size());

    // std::cout << "para.size() = " << para.size() << std::endl;

    // for (int i = 0; i < para.size(); ++i)
    // {
    //     std::cout << "para[" << i << "] = " << para[i] << std::endl;
    // }

    // // Calculate trqT
    // Eigen::MatrixXd para_matrix_reversed = para_matrix.transpose();
    // Eigen::MatrixXd motS_matrix_reversed = motS_matrix.transpose();
    // Eigen::MatrixXd trqT_matrix = motS_matrix_reversed * para_matrix_reversed;
    // std::vector<double> trqT = std::vector<double>(trqT_matrix.data(), trqT_matrix.data() + trqT_matrix.rows() * trqT_matrix.cols());

    // std::cout << "param = " << para_matrix_reversed << std::endl;

    // Split trqT into trqT0, trqT1, trqT2, trqT3, trqT4, trqT5, and trqT6 based on the division

    // trqT0 = ...
    // trqT1 = ...
    // trqT2 = ...
    // trqT3 = ...
    // trqT4 = ...
    // trqT5 = ...
    // trqT6 = ...

    return 0;
}
