#include "../include/UR5Kinematic.hpp"	

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>

#define _USE_MATH_DEFINES

int main(int argc, char* argv[]) {

    UR5Kinematic robot;

    Eigen::MatrixXd theta(6,1);
    
    Eigen::MatrixXd error(3,1);

    Eigen::MatrixXd pre_forward_target(3,1);
    Eigen::MatrixXd after_forward_target(3,1);

    Eigen::MatrixXd inverse_kin_theta(6,1);

    int n = 10;

    double avg_time = 0;
    double avg_error = 0;

    std::cout << "Executing Test: Inverse Kinematic! " << std::endl << n << " times" << std::endl << std::endl;

    for (int i= 0; i < n; i++) {
        theta  = Eigen::MatrixXd::Random(6,1) * M_PI;
        

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

        avg_error   += std::sqrt(std::pow(error(0,0),2) + std::pow(error(1, 0),2) + std::pow(error(2, 0),2));
        avg_time    += std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count()/1e6;
    }
    
    avg_time = avg_time / (double)n;
    avg_error = avg_error / (double)n;

    std::cout << "average error:\t\t\t" << avg_error << std::endl; 
    std::cout << "average calculation time:\t" << avg_time << " ms" << std::endl;
    return 0;
    
}