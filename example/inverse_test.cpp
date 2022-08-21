#include "../include/UR5Kinematic.hpp"

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>


int main(int argc, char* argv[]) {

    UR5Kinematic robot;

    Eigen::MatrixXd theta(6,1);
    Eigen::MatrixXd error(3,1);

    Eigen::MatrixXd pre_forward_target(3,1);
    Eigen::MatrixXd after_forward_target(3,1);

    Eigen::MatrixXd inverse_kin_theta(6,1);

    // check if all needed arguments are present, if not tell user what the input needs to be and exit program
    if (argc != 7) {
        std::cout << std::endl;
        std::cout << "please start the code with: ./inverse_test  [theta_1] [theta_2] [theta_3] [theta_4] [theta_5] [theta_6]" << std::endl;
        std::cout << "[theta_1] angle of the first robotic joint" << std::endl;
        std::cout << "[theta_2] angle of the second robotic joint" << std::endl;
        std::cout << "[theta_3] angle of the third robotic joint" << std::endl;
        std::cout << "[theta_4] angle of the fourth robotic joint" << std::endl;
        std::cout << "[theta_5] angle of the fifth robotic joint" << std::endl;
        std::cout << "[theta_6] angle of the sixth robotic joint" << std::endl;
        std::cout << std::endl;
        return -1;
    } else {
        // write the given arguments into the theta variable
        for (int i = 0; i < 6; i++) {
            theta(i, 0) = strtod(argv[i+1], NULL);
        }
    }
    std::cout << std::endl << "Executing Test: Inverse Kinematic!" << std::endl << std::endl;

    // execute the forward kinematic, so that the inverse kinematic can be filled with position target values
    pre_forward_target = robot.calculateForwardKinematic(theta(0,0), theta(1,0), theta(2,0), theta(3,0), theta(4,0), theta(5,0));

    // starting the runtime measurment
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    // compute the inverse kinematic, use the previous calculated target positions
    inverse_kin_theta = robot.computeInverseKinematic(pre_forward_target(0, 0), pre_forward_target(1, 0), pre_forward_target(2, 0));
    // ending the runtime measurment
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    
    // evaluate inverse kinematic
    after_forward_target = robot.calculateForwardKinematic(inverse_kin_theta(0, 0), inverse_kin_theta(1, 0), inverse_kin_theta(2, 0), inverse_kin_theta(3, 0), inverse_kin_theta(4, 0), inverse_kin_theta(5, 0));

    // calculate remaining error
    error = after_forward_target-pre_forward_target;

    // check if error is in allowed range, range is setable inside the class
    if ((std::abs(error(0,0)) < robot.getMaxAllowedError()) && (std::abs(error(1,0)) < robot.getMaxAllowedError()) && (std::abs(error(2,0)) < robot.getMaxAllowedError())) {
        std::cout << "Test Inverse kinematic -> successfull" << std::endl;
    } else {
        std::cout << "Test Inverse kinematic -> NOT successfull" << std::endl;
    }

    // create user readable output
    std::cout << std::endl;
    std::cout << "calculated theta, angles [rad]:" << std::endl;
    std::cout << std::setprecision(9) << inverse_kin_theta(0, 0) << ", " << inverse_kin_theta(1, 0) << ", " << inverse_kin_theta(2, 0) << ", " << inverse_kin_theta(3, 0) << ", " << inverse_kin_theta(4, 0) << ", " << inverse_kin_theta(5, 0) << std::endl;
    std::cout << std::endl;
    std::cout << "(pre) calculated forward kinematic" << std::endl << std::endl;
    std::cout << "x-axis:\t" << std::setprecision(9) << pre_forward_target(0, 0) << std::endl;
    std::cout << "y-axis:\t" << std::setprecision(9) << pre_forward_target(1, 0) << std::endl; 
    std::cout << "z-axis:\t" << std::setprecision(9) << pre_forward_target(2, 0) << std::endl; 
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "(after) calculated forward kinematic with error" << std::endl << std::endl;
    std::cout << "x-axis:\t" << std::setprecision(9) << after_forward_target(0, 0) << "\t+ (" << error(0, 0) << ")" << std::endl;
    std::cout << "y-axis:\t" << std::setprecision(9) << after_forward_target(1, 0) << "\t+ (" << error(1, 0) << ")" << std::endl; 
    std::cout << "z-axis:\t" << std::setprecision(9) << after_forward_target(2, 0) << "\t+ (" << error(2, 0) << ")" << std::endl; 
    std::cout << std::endl;
    std::cout << "needed runtime: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count() << " ns" << std::endl;
    std::cout << std::endl;
    return 0;
}