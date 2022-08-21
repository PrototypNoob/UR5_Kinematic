#ifndef _UR5KINEMATIC_HPP_
#define _UR5KINEMATIC_HPP_

#include <eigen3/Eigen/Dense>

/**
 * @brief This class implements an forward and inverse kinematic of an ur5 robot. For the implementaiton
 * the Denavit Hartenberg transformation is used to create the forward kinematic and the jacobian. Plese note
 * that the denavit parameters are hard coded and should be changed if another kinematic should be evaluated.
 * To be able to use this class, the library Eigen needs to be installed on the system.
 * 
 * @author Vincent Mannheim
 * @date 21.08.2022
 */
class UR5Kinematic {
    public:
        /**
         * @brief Construct a new UR5Kinematic object.
         * In this constructor the denavit hartenberg parameters are initalised. Also the current angle position (theta)
         * is given, so that for the inverse kineamtic an start point is given. To use this on a real robotic system
         * it would be recommended to set the current parameter and change the denavit parameter acording to an config file.
         */
        UR5Kinematic();
        /**
         * @brief Destroy the UR5Kinematic object.
         */
        ~UR5Kinematic();

        /**
         * @brief Set the Max Allowed Error object, this value impacts the runtime of the inverse kinematic
         * if the allowed error is higher the inverse kinematic reaches a solution faster. Check the value 
         * if the runtime is important or increase it, if the position accuracity is important.
         * 
         * @param allowed_error the error that should be checked by the inverse kinematic, this value is used
         * by all axises.
         */
        void setMaxAllowedError(double allowed_error);
        /**
         * @brief Set the Current Theta object
         * Allows to change the inital position (angle) or set a new position (angle) this
         * is needed for the inverse kinematic, so that the robot knows where it is at the start.
         * 
         * @param theta_1 the theta angle of joint 1
         * @param theta_2 the theta angle of joint 2
         * @param theta_3 the theta angle of joint 3
         * @param theta_4 the theta angle of joint 4
         * @param theta_5 the theta angle of joint 5
         * @param theta_6 the theta angle of joint 6
         */
        void setCurrentTheta(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6);

        /**
         * @brief Get the Max Allowed Error object, this returns the allowed error. With this it can be checked
         * if the inverse kinematic reached an the position with a high enough accuracity. Here no accuracity per
         * axis is given.
         * 
         * @return double the maximum allowed error set for the inverse kinematic
         */
        double getMaxAllowedError();
        /**
         * @brief Get the Current Theta object, this returns the current position of all angles of the robotic system.
         * 
         * @return Eigen::MatrixXd(6,1) the current theta angles
         */
        Eigen::MatrixXd getCurrentTheta();
        /**
         * @brief Get the Pos Error object, returns the remaining position error.
         * 
         * @return Eigen::MatrixXd(3,1) the reamining position error
         */
        Eigen::MatrixXd getPosError();
        /**
         * @brief Get the Pos Target object, allows access to the position target that was given to the 
         * inverse kinematic.
         * 
         * @return Eigen::MatrixXd(3,1) the position that should be reached by the inverse kinematic
         */
        Eigen::MatrixXd getPosTarget();
        
        /**
         * @brief calculates the forward kinematic via the denavit hartenberg parameters. This only return the 
         * position of the end effector and not positions of the single joints.
         * 
         * @param theta_1 the given angle of joint 1
         * @param theta_2 the given angle of joint 2
         * @param theta_3 the given angle of joint 3
         * @param theta_4 the given angle of joint 4
         * @param theta_5 the given angle of joint 5
         * @param theta_6 the given angle of joint 6
         * @return Eigen::MatrixXd(3,1) the position of the UR5 robot. 
         */
        Eigen::MatrixXd calculateForwardKinematic(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6);
        /**
         * @brief computes the inverse kinematic of the UR5 robot, the used algoritem is the IK solver shown on the wikipedia page
         * 
         * @param x_target the target position of the end effector in x direction
         * @param y_target the target position of the end effector in y direction
         * @param z_target the target position of the end effector in z direction
         * @return Eigen::MatrixXd(6,1) the theta angles computed by the inverse kinematic
         */
        Eigen::MatrixXd computeInverseKinematic(double x_target, double y_target, double z_target);
    private:
        /// @brief maximum allowed position error, used in inverse kineatic
        double max_allowed_error = 0.000001;
        /// @brief the inital parameters of the denavit hartenberg matrix
        Eigen::MatrixXd denavit_param{6,4};
        /// @brief the current angle positions of the joints
        Eigen::MatrixXd current_theta{6,1};

        /// @brief the denavit hartenberg transformation between joint 0 and 1 
        Eigen::MatrixXd joints01_denavit{4,4};
        /// @brief the denavit hartenberg transformation between joint 0 and 2 
        Eigen::MatrixXd joints02_denavit{4,4};
        /// @brief the denavit hartenberg transformation between joint 0 and 3 
        Eigen::MatrixXd joints03_denavit{4,4};
        /// @brief the denavit hartenberg transformation between joint 0 and 4
        Eigen::MatrixXd joints04_denavit{4,4};
        /// @brief the denavit hartenberg transformation between joint 0 and 5 
        Eigen::MatrixXd joints05_denavit{4,4};
        /// @brief the denavit hartenberg transformation between joint 0 and 6
        Eigen::MatrixXd joints06_denavit{4,4};

        /// @brief the position error that remains after running the inverse kinematic
        Eigen::MatrixXd pos_error{3, 1};
        /// @brief the position target that should be reached by the inverse kinematic
        Eigen::MatrixXd pos_target{3, 1};
        /// @brief the position of the end effector after running a forward kinematic
        Eigen::MatrixXd forward_kinematic_ee {3, 1};

        /**
         * @brief Set the Position Target object, sets the target position for the inverse 
         * kinematic. Can not be used outside of UR5Kinematic
         * 
         * @param x_target the x-axis target position 
         * @param y_target the y-axis target position
         * @param z_target the z-axis target position
         */
        void setPositionTarget(double x_target, double y_target, double z_target);
        /**
         * @brief this method calculates the denavit hartenberg parameter between the base an all joints. This Method
         * is needed because the inverse kinematic and forward kinematic need to calculate the transforamtion
         * 
         * @param theta_1 the joint angle theta of joint 1
         * @param theta_2 the joint angle theta of joint 2
         * @param theta_3 the joint angle theta of joint 3
         * @param theta_4 the joint angle theta of joint 4
         * @param theta_5 the joint angle theta of joint 5 
         * @param theta_6 the joint angle theta of joint 6
         */
        void calculateFullRobotDenavit(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6);

        /**
         * @brief this method calculates the jacobian via the rotation and direction matrix of the denavit hartenberg transforamtion. 
         * The jacobian is needed for the IK algoritem.
         * 
         * @param theta_1 the joint angle theta of joint 1
         * @param theta_2 the joint angle theta of joint 2
         * @param theta_3 the joint angle theta of joint 3
         * @param theta_4 the joint angle theta of joint 4
         * @param theta_5 the joint angle theta of joint 5
         * @param theta_6 the joint angle theta of joint 6
         * @return Eigen::MatrixXd(3, 6) the jacobian of the UR5 robot (without orientation)
         */
        Eigen::MatrixXd calculateJacobian(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6);
        /**
         * @brief Implements the Denavit Hartenberg transforamtion, with the given parameters.
         * 
         * @param theta the theta angle of the UR5 robot
         * @param a the distance between joints in x-direction
         * @param d the distance between joints in z-direction
         * @param alpha the rotation around the x-axis
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd denavitHartenbergTrans(double theta, double a, double d, double alpha);
        /**
         * @brief this method calcualtes the inverse kineamtic, for this the functionallity of the Eigen Library is used.
         * 
         * @param theta_1 the joint angle theta of joint 1
         * @param theta_2 the joint angle theta of joint 2
         * @param theta_3 the joint angle theta of joint 3
         * @param theta_4 the joint angle theta of joint 4
         * @param theta_5 the joint angle theta of joint 5
         * @param theta_6 the joint angle theta of joint 6
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd calculateInverseJacobian(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6);
};

#endif /* _UR5KINEMATIC_HPP_ */