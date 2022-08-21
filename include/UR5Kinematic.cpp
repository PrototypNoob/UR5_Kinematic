#include "UR5Kinematic.hpp"

#include <cmath>

#define _USE_MATH_DEFINES

UR5Kinematic::UR5Kinematic() {
    // the denavit hartenberg parameter of the UR5 robot, should not be hard coded. 
    // another option is to use a config file, so that no rebuild is needed.
    this->denavit_param <<  0,  0,          0.089159,   (M_PI/2),
                            0,  -0.425,     0,          0,
                            0,  -0.39225,   0,          0,
                            0,  0,          0.10915,    (M_PI/2),
                            0,  0,          0.09465,    (-M_PI/2),
                            0,  0,          0.0823,     0;

    // set the inital angle position of the UR5 robot, for invers kinematic
    this->setCurrentTheta(M_PI/2, M_PI/3, M_PI/4, M_PI/5, M_PI/6, M_PI/7);
}

UR5Kinematic::~UR5Kinematic() {
}

void UR5Kinematic::setMaxAllowedError(double allowed_error) {
    this->max_allowed_error = allowed_error;
}

void UR5Kinematic::setCurrentTheta(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6) {
    this->current_theta(0, 0) = theta_1;
    this->current_theta(1, 0) = theta_2;
    this->current_theta(2, 0) = theta_3;
    this->current_theta(3, 0) = theta_4;
    this->current_theta(4, 0) = theta_5;
    this->current_theta(5, 0) = theta_6;
}

double UR5Kinematic::getMaxAllowedError() {
    return this->max_allowed_error;
}

Eigen::MatrixXd UR5Kinematic::getCurrentTheta() {
    return this->current_theta;
}

Eigen::MatrixXd UR5Kinematic::getPosError() {
    return this->pos_error;
}

Eigen::MatrixXd UR5Kinematic::getPosTarget() {
    return this->pos_target;
}

Eigen::MatrixXd UR5Kinematic::calculateForwardKinematic(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6) {
    this->calculateFullRobotDenavit(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6);

    // only return the direction matrix of the denavit hartenberg tranformation, for end effector between joint 0 and 6
    Eigen::MatrixXd forward_kinematik = this->joints06_denavit.block<3, 1>(0, 3);
    return forward_kinematik;
}

Eigen::MatrixXd UR5Kinematic::computeInverseKinematic(double x_target, double y_target, double z_target) {
    Eigen::MatrixXd i_1_theta(6, 1);
    Eigen::MatrixXd i_theta(6, 1);

    i_theta = this->getCurrentTheta();
    this->setPositionTarget(x_target, y_target, z_target);

    // calculate the forward kineamtic of the system
    Eigen::MatrixXd forward = this->calculateForwardKinematic(i_theta(0, 0), i_theta(1, 0), i_theta(2, 0), i_theta(3, 0),i_theta(4, 0), i_theta(5, 0));

    // calculate the psotion error, between current position and target
    this->pos_error = this->pos_target - forward;

    // use the IK solver to solve reach the required accuracity
    while((std::abs(this->pos_error(0,0)) > this->max_allowed_error) || (std::abs(this->pos_error(1,0)) > this->max_allowed_error) || (std::abs(this->pos_error(2,0)) > this->max_allowed_error)) {
        // calculate the inverse kineamtic with i_theta
        Eigen::MatrixXd inverse_jacobian = this->calculateInverseJacobian(i_theta(0, 0), i_theta(1, 0), i_theta(2, 0), i_theta(3, 0),i_theta(4, 0), i_theta(5, 0));

        // change the theta angles according to the position error and the inverse kinematic
        i_1_theta = i_theta + inverse_jacobian * this->pos_error;
        // calculate the forward kinematic again, to check the current position
        forward = this->calculateForwardKinematic(i_1_theta(0, 0), i_1_theta(1, 0), i_1_theta(2, 0), i_1_theta(3, 0),i_1_theta(4, 0), i_1_theta(5, 0));

        // recalculat the remaining error, to check if close enough to the target and for next adjustment
        this->pos_error = this->pos_target - forward;
        i_theta = i_1_theta;
    }

    return i_theta;
}

void UR5Kinematic::setPositionTarget(double x_target, double y_target, double z_target) {
    this->pos_target(0, 0) = x_target;
    this->pos_target(1, 0) = y_target;
    this->pos_target(2, 0) = z_target;
}

void UR5Kinematic::calculateFullRobotDenavit(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6) {
    // calculate the denavit hartenberg transformation
    Eigen::MatrixXd calc_joints01_denavit = this->denavitHartenbergTrans(theta_1, this->denavit_param(0,1), this->denavit_param(0,2), this->denavit_param(0,3));
    Eigen::MatrixXd calc_joints12_denavit = this->denavitHartenbergTrans(theta_2, this->denavit_param(1,1), this->denavit_param(1,2), this->denavit_param(1,3));
    Eigen::MatrixXd calc_joints23_denavit = this->denavitHartenbergTrans(theta_3, this->denavit_param(2,1), this->denavit_param(2,2), this->denavit_param(2,3));
    Eigen::MatrixXd calc_joints34_denavit = this->denavitHartenbergTrans(theta_4, this->denavit_param(3,1), this->denavit_param(3,2), this->denavit_param(3,3));
    Eigen::MatrixXd calc_joints45_denavit = this->denavitHartenbergTrans(theta_5, this->denavit_param(4,1), this->denavit_param(4,2), this->denavit_param(4,3));
    Eigen::MatrixXd calc_joints56_denavit = this->denavitHartenbergTrans(theta_6, this->denavit_param(5,1), this->denavit_param(5,2), this->denavit_param(5,3));

    // create the denavit hartenberg transformation based of the base (joint 0) of the robot 
    this->joints01_denavit = calc_joints01_denavit;
    this->joints02_denavit = calc_joints01_denavit * calc_joints12_denavit;
    this->joints03_denavit = calc_joints01_denavit * calc_joints12_denavit * calc_joints23_denavit;
    this->joints04_denavit = calc_joints01_denavit * calc_joints12_denavit * calc_joints23_denavit * calc_joints34_denavit;
    this->joints05_denavit = calc_joints01_denavit * calc_joints12_denavit * calc_joints23_denavit * calc_joints34_denavit * calc_joints45_denavit;
    this->joints06_denavit = calc_joints01_denavit * calc_joints12_denavit * calc_joints23_denavit * calc_joints34_denavit * calc_joints45_denavit * calc_joints56_denavit;
}


Eigen::MatrixXd UR5Kinematic::denavitHartenbergTrans(double theta, double a, double d, double alpha)
{
    // complet denavit hartenberg transformation
    Eigen::MatrixXd denavit_transforamtion(4,4);

    denavit_transforamtion(0,0) = std::cos(theta);
    denavit_transforamtion(0,1) = -1 * std::sin(theta) * std::cos(alpha);
    denavit_transforamtion(0,2) = std::sin(theta) * std::sin(alpha);
    denavit_transforamtion(0,3) = a * std::cos(theta);

    denavit_transforamtion(1,0) = std::sin(theta);
    denavit_transforamtion(1,1) = std::cos(theta) * std::cos(alpha);
    denavit_transforamtion(1,2) = -1 * std::cos(theta) * std::sin(alpha);
    denavit_transforamtion(1,3) = a * std::sin(theta);

    denavit_transforamtion(2,0) = 0;
    denavit_transforamtion(2,1) = std::sin(alpha);
    denavit_transforamtion(2,2) = std::cos(alpha);
    denavit_transforamtion(2,3) = d;

    denavit_transforamtion(3,0) = 0;
    denavit_transforamtion(3,1) = 0;
    denavit_transforamtion(3,2) = 0;
    denavit_transforamtion(3,3) = 1;

    return denavit_transforamtion;
}

Eigen::MatrixXd UR5Kinematic::calculateJacobian(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6){
    this->calculateFullRobotDenavit(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6);
    
    // get the rotation and direction matrix out of the denavit hartenberg transformation
    Eigen::MatrixXd rot_joint01_dh = this->joints01_denavit.block<3, 3>(0, 0);
    Eigen::MatrixXd dir_joint01_dh = this->joints01_denavit.block<3, 1>(0, 3);

    Eigen::MatrixXd rot_joint02_dh = this->joints02_denavit.block<3, 3>(0, 0);
    Eigen::MatrixXd dir_joint02_dh = this->joints02_denavit.block<3, 1>(0,3);

    Eigen::MatrixXd rot_joint03_dh = this->joints03_denavit.block<3, 3>(0, 0);
    Eigen::MatrixXd dir_joint03_dh = this->joints03_denavit.block<3, 1>(0,3);

    Eigen::MatrixXd rot_joint04_dh = this->joints04_denavit.block<3, 3>(0, 0);
    Eigen::MatrixXd dir_joint04_dh = this->joints04_denavit.block<3, 1>(0,3);

    Eigen::MatrixXd rot_joint05_dh = this->joints05_denavit.block<3, 3>(0, 0);
    Eigen::MatrixXd dir_joint05_dh = this->joints05_denavit.block<3, 1>(0,3);

    Eigen::MatrixXd rot_joint06_dh = this->joints06_denavit.block<3, 3>(0, 0);
    Eigen::MatrixXd dir_joint06_dh = this->joints06_denavit.block<3, 1>(0,3);

    // create "join" jacobian for the robotic system
    Eigen::Vector3d M1(0,0,1); // gleich zu R_0^0*M1

    Eigen::Vector3d rot_vector_01 = rot_joint01_dh * M1;
    Eigen::Vector3d rot_vector_02 = rot_joint02_dh * M1;
    Eigen::Vector3d rot_vector_03 = rot_joint03_dh * M1;
    Eigen::Vector3d rot_vector_04 = rot_joint04_dh * M1;
    Eigen::Vector3d rot_vector_05 = rot_joint05_dh * M1;
    Eigen::Vector3d rot_vector_06 = rot_joint06_dh * M1;

    Eigen::Vector3d dir_vector_01(Eigen::Map<Eigen::Vector3d>(dir_joint01_dh.data(), dir_joint01_dh.cols() * dir_joint01_dh.rows()));
    Eigen::Vector3d dir_vector_02(Eigen::Map<Eigen::Vector3d>(dir_joint02_dh.data(), dir_joint02_dh.cols() * dir_joint02_dh.rows()));
    Eigen::Vector3d dir_vector_03(Eigen::Map<Eigen::Vector3d>(dir_joint03_dh.data(), dir_joint03_dh.cols() * dir_joint03_dh.rows()));
    Eigen::Vector3d dir_vector_04(Eigen::Map<Eigen::Vector3d>(dir_joint04_dh.data(), dir_joint04_dh.cols() * dir_joint04_dh.rows()));
    Eigen::Vector3d dir_vector_05(Eigen::Map<Eigen::Vector3d>(dir_joint05_dh.data(), dir_joint05_dh.cols() * dir_joint05_dh.rows()));
    Eigen::Vector3d dir_vector_06(Eigen::Map<Eigen::Vector3d>(dir_joint06_dh.data(), dir_joint06_dh.cols() * dir_joint06_dh.rows()));

    Eigen::MatrixXd tmp_jacobian_1 = M1.cross(dir_vector_06);
    Eigen::MatrixXd tmp_jacobian_2 = rot_vector_01.cross(dir_vector_06-dir_vector_01);
    Eigen::MatrixXd tmp_jacobian_3 = rot_vector_02.cross(dir_vector_06-dir_vector_02);
    Eigen::MatrixXd tmp_jacobian_4 = rot_vector_03.cross(dir_vector_06-dir_vector_03);
    Eigen::MatrixXd tmp_jacobian_5 = rot_vector_04.cross(dir_vector_06-dir_vector_04);
    Eigen::MatrixXd tmp_jacobian_6 = rot_vector_05.cross(dir_vector_06-dir_vector_05);

    // create the current jacobian
    Eigen::MatrixXd jacobian(3,6); 

    jacobian(0,0) = tmp_jacobian_1(0,0);
    jacobian(1,0) = tmp_jacobian_1(1,0);
    jacobian(2,0) = tmp_jacobian_1(2,0);

    jacobian(0,1) = tmp_jacobian_2(0,0);
    jacobian(1,1) = tmp_jacobian_2(1,0);
    jacobian(2,1) = tmp_jacobian_2(2,0);

    jacobian(0,2) = tmp_jacobian_3(0,0);
    jacobian(1,2) = tmp_jacobian_3(1,0);
    jacobian(2,2) = tmp_jacobian_3(2,0);

    jacobian(0,3) = tmp_jacobian_4(0,0);
    jacobian(1,3) = tmp_jacobian_4(1,0);
    jacobian(2,3) = tmp_jacobian_4(2,0);

    jacobian(0,4) = tmp_jacobian_5(0,0);
    jacobian(1,4) = tmp_jacobian_5(1,0);
    jacobian(2,4) = tmp_jacobian_5(2,0);

    jacobian(0,5) = tmp_jacobian_6(0,0);
    jacobian(1,5) = tmp_jacobian_6(1,0);
    jacobian(2,5) = tmp_jacobian_6(2,0);

    return jacobian;
}

Eigen::MatrixXd UR5Kinematic::calculateInverseJacobian(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6) {
    Eigen::MatrixXd jacobian = this->calculateJacobian(theta_1,theta_2,theta_3,theta_4,theta_5,theta_6);
    Eigen::MatrixXd inverse_jacobian = jacobian.completeOrthogonalDecomposition().pseudoInverse();

    return inverse_jacobian;
}