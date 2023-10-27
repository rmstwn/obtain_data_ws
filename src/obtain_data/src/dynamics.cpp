#include <iostream>
#include <cmath>
#include <vector>

int main()
{
    int len = 10; // Replace with your desired length

    // Define th0 to th7 and omg0 to omg7 as double
    double th0;
    double th1;
    double th2;
    double th3;
    double th4;
    double th5;
    double th6;
    double th7;

    double omg0 = -1.3;
    double omg1;
    double omg2;
    double omg3;
    double omg4;
    double omg5;
    double omg6;
    double omg7;

    // Assuming you have populated th0 to th7 and omg0 to omg7 variables
    std::vector<double> mot0(26, 0.0);
    std::vector<double> mot1(26, 0.0);
    std::vector<double> mot2(26, 0.0);
    std::vector<double> mot3(26, 0.0);
    std::vector<double> mot4(26, 0.0);
    std::vector<double> mot5(26, 0.0);
    std::vector<double> mot6(26, 0.0);
    std::vector<double> mot7(26, 0.0);

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
    mot1[22] = -std::sin(th1);                                                                                                                                                                                                                                                                                 // Param Mot 1
    mot1[23] = std::cos(th1) * std::cos(th2) * std::cos(th3) - std::sin(th1) * std::sin(th3);                                                                                                                                                                                                                  // Param Mot 2
    mot1[24] = -std::cos(th3) * std::sin(th1) - std::cos(th1) * std::cos(th2) * std::sin(th3);                                                                                                                                                                                                                 // Param Mot 3
    mot1[25] = -std::cos(th3) * std::cos(th5) * std::sin(th1) - std::cos(th2) * std::cos(th5) * std::sin(th3) + std::cos(th1) * std::sin(th2) * std::sin(th4) * std::sin(th5) + std::cos(th4) * std::sin(th1) * std::sin(th3) * std::sin(th5) - std::cos(th2) * std::cos(th3) * std::cos(th4) * std::sin(th5); // Param Mot 4

    // Print the values of mot1 for verification
    for (int i = 0; i < mot1.size(); ++i)
    {
        std::cout << "mot1[" << i << "] = " << mot1[i] << std::endl;
    }

    // Assign values to mot2
    mot2[4] = std::copysign(1.0, omg2);                                                                                                                                                                                       // 1st axis friction term 3 Vel 2
    mot2[5] = std::copysign(1.0, omg2) * std::sqrt(std::fabs(omg2));                                                                                                                                                          // 2nd axis friction term 6 Vel 3
    mot2[16] = 1.0;                                                                                                                                                                                                           // Positive constant 3 (1.0 for the 3rd constant)
    mot2[22] = -std::cos(th3) * std::sin(th1) * std::sin(th2);                                                                                                                                                                // Param Mot 2
    mot2[23] = std::sin(th1) * std::sin(th2) * std::sin(th3);                                                                                                                                                                 // Param Mot 3
    mot2[24] = std::cos(th5) * std::sin(th1) * std::sin(th2) * std::sin(th3) + std::cos(th2) * std::sin(th1) * std::sin(th4) * std::sin(th5) + std::cos(th3) * std::cos(th4) * std::sin(th1) * std::sin(th2) * std::sin(th5); // Param Mot 4

    // Print the values of mot1 for verification
    for (int i = 0; i < mot2.size(); ++i)
    {
        std::cout << "mot2[" << i << "] = " << mot2[i] << std::endl;
    }

    // Assign values to mot3
    mot3[4] = std::copysign(1.0, omg3);                                                                                                                                                                                                                                        // 1st axis friction term 7 Vel 4
    mot3[5] = std::copysign(1.0, omg3) * std::sqrt(std::fabs(omg3));                                                                                                                                                                                                           // 3rd axis friction term 8 Vel 4
    mot3[17] = 1.0;                                                                                                                                                                                                                                                            // Positive constant 4 (1.0 for the 4th constant)
    mot3[23] = std::cos(th1) * std::cos(th3) - std::cos(th2) * std::sin(th1) * std::sin(th3);                                                                                                                                                                                  // Param Mot 2
    mot3[24] = -std::cos(th1) * std::sin(th3) - std::cos(th2) * std::cos(th3) * std::sin(th1);                                                                                                                                                                                 // Param Mot 3
    mot3[25] = -std::cos(th1) * std::cos(th5) * std::sin(th3) - std::cos(th2) * std::cos(th3) * std::cos(th5) * std::sin(th1) - std::cos(th1) * std::cos(th3) * std::cos(th4) * std::sin(th5) + std::cos(th2) * std::cos(th4) * std::sin(th1) * std::sin(th3) * std::sin(th5); // Param Mot 4

    // Print the values of mot3 for verification
    for (int i = 0; i < mot3.size(); ++i)
    {
        std::cout << "mot3[" << i << "] = " << mot3[i] << std::endl;
    }

    // Assign values to mot4
    mot4[12] = std::copysign(1.0, omg4);                                                                                                                                                                                      // 3rd axis friction term 9 Vel 5
    mot4[13] = std::copysign(1.0, omg4) * std::sqrt(std::fabs(omg4));                                                                                                                                                         // 3rd axis friction term 10 Vel 5
    mot4[19] = 1.0;                                                                                                                                                                                                           // Positive constant 5 (1.0 for the 5th constant)
    mot4[24] = std::cos(th4) * std::sin(th1) * std::sin(th2) * std::sin(th5) + std::cos(th1) * std::sin(th3) * std::sin(th4) * std::sin(th5) + std::cos(th2) * std::cos(th3) * std::sin(th1) * std::sin(th4) * std::sin(th5); // Param Mot 4

    // Print the values of mot4 for verification
    for (int i = 0; i < mot4.size(); ++i)
    {
        std::cout << "mot4[" << i << "] = " << mot4[i] << std::endl;
    }

    // Assign values to mot5
    mot5[16] = std::copysign(1.0, omg5);                                                                                                                                                                                                                                                                                                      // 3rd axis friction term 11 Vel 6
    mot5[17] = std::copysign(1.0, omg5) * std::sqrt(std::fabs(omg5));                                                                                                                                                                                                                                                                         // 3rd axis friction term 12 Vel 6
    mot5[20] = 1.0;                                                                                                                                                                                                                                                                                                                           // Positive constant 6 (1.0 for the 6th constant)
    mot5[25] = std::cos(th2) * std::sin(th1) * std::sin(th3) * std::sin(th5) - std::cos(th1) * std::cos(th3) * std::sin(th5) - std::cos(th1) * std::cos(th4) * std::cos(th5) * std::sin(th3) + std::cos(th5) * std::sin(th1) * std::sin(th2) * std::sin(th4) - std::cos(th2) * std::cos(th3) * std::cos(th4) * std::cos(th5) * std::sin(th1); // Param Mot 4

    // Assign values to mot6
    mot6[12] = std::copysign(1.0, omg6);                              // 3rd axis friction term 13 Vel 5
    mot6[13] = std::copysign(1.0, omg6) * std::sqrt(std::fabs(omg6)); // 3rd axis friction term 14 Vel 5
    mot6[21] = 1.0;

    return 0;
}