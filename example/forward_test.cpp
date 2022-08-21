#include "../include/UR5Kinematic.hpp"

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>

int main(int argc, char* argv[]) {

    UR5Kinematic robot;
    double allowed_error = 1e-4;

    Eigen::MatrixXd theta(6,1);
    Eigen::MatrixXd target(3,1);
    Eigen::MatrixXd error(3,1);

    // check if all paramerters are given, if not tell user what paramerters are need an exit program
    if (argc != 10) {
        std::cout << "please start the code with: ./forward_test  [theta_1] [theta_2] [theta_3] [theta_4] [theta_5] [theta_6] [x_value_ee] [y_value_ee] [z_value_ee]" << std::endl;
        std::cout << "[theta_1] angle of the first robotic joint" << std::endl;
        std::cout << "[theta_2] angle of the second robotic joint" << std::endl;
        std::cout << "[theta_3] angle of the third robotic joint" << std::endl;
        std::cout << "[theta_4] angle of the fourth robotic joint" << std::endl;
        std::cout << "[theta_5] angle of the fifth robotic joint" << std::endl;
        std::cout << "[theta_6] angle of the sixth robotic joint" << std::endl;
        std::cout << "[x_value_ee] the x value the end effector calculated be the excel sheet" << std::endl;
        std::cout << "[y_value_ee] the y value the end effector calculated be the excel sheet" << std::endl;
        std::cout << "[z_value_ee] the z value the end effector calculated be the excel sheet" << std::endl;
        return -1;
    } else {
        // configurate theta with the given parameters
        for (int i = 0; i < 6; i++) {
            theta(i, 0) = strtod(argv[i+1], NULL);
        }
        // configurate the should target with the given parameters
        for (int j = 0; j < 3; j++) {
            target(j, 0) = strtod(argv[j+7], NULL);
        }
    }
    std::cout << std::endl << "Executing Test: Forward Kinematic!" << std::endl << std::endl;

    // get start value for runtime calculation
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    // execute the forward kinematic of the UR5Kinematic class
    Eigen::MatrixXd forward_values = robot.calculateForwardKinematic(theta(0,0), theta(1,0), theta(2,0), theta(3,0), theta(4,0), theta(5,0));
    // get end value for the runtime calculation
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    
    // calculate error between given values and calculated values
    error = target-forward_values;

    // check if test was successfull
    if ((std::abs(error(0,0)) < allowed_error) && (std::abs(error(1,0)) < allowed_error) && (std::abs(error(2,0)) < allowed_error)) {
        std::cout << "Test Forward kinematic -> successfull" << std::endl;
    } else {
        std::cout << "Test Forward kinematic -> NOT successfull" << std::endl;
    }

    // create user readable output
    std::cout << "calcualted values with error" << std::endl << std::endl;
    std::cout << "x-axis:\t" << std::setprecision(9) << forward_values(0, 0) << "\t+ (" << error(0, 0) << ")" << std::endl;
    std::cout << "y-axis:\t" << std::setprecision(9) << forward_values(1, 0) << "\t+ (" << error(1, 0) << ")" << std::endl; 
    std::cout << "z-axis:\t" << std::setprecision(9) << forward_values(2, 0) << "\t+ (" << error(2, 0) << ")" << std::endl; 
    std::cout << std::endl;
    std::cout << "needed runtime: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count() << " ns" << std::endl;
    std::cout << std::endl;
    return 0;
}