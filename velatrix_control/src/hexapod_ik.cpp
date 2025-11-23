/******************************************************************************
*  Filename:           hexapod_ik.cpp
*  Description:        Define the hexapod single leg inverse kinematics functions
*  Author:             Gouri Shankar
******************************************************************************/
#include <cmath>
#include "hexapod_kinematics/hexapod_config.hpp"
#include "hexapod_kinematics/hexapod_ik.hpp"

namespace hexapod_kinematics {

    HexapodIK::HexapodIK() {
        L1_ = hexapod_kinematics::LegDimensions::COXA_LENGTH;
        L2_ = hexapod_kinematics::LegDimensions::FEMUR_LENGTH;
        L3_ = hexapod_kinematics::LegDimensions::TIBIA_LENGTH;
    }

    Eigen::Vector3d HexapodIK::solverIK(int leg_id, const Eigen::Vector3d& foot_pos) {

        double x_pos = foot_pos.x();
        double y_pos = foot_pos.y();
        double z_pos = foot_pos.z();

        double theta1_ = computeCoxaAngle(x_pos, y_pos); // Coxa angle

        double r_2 = ((x_pos / std::cos(theta1_)) - L1_);
        double r_1 = std::sqrt(pow(z_pos, 2) + pow(r_2, 2));

        double theta2_ = computeFemurAngle(x_pos, z_pos, theta1_, r_1, r_2); // Femur angle

        double theta3_ = computeTibiaAngle(r_1);  // Tibia angle

        Eigen::Vector3d joint_angles;
        joint_angles.x() = theta1_;
        joint_angles.y() = theta2_;
        joint_angles.z() = theta3_;

        return joint_angles;

    };

    double HexapodIK::computeCoxaAngle(double x_pos, double y_pos) {
        return std::atan2(y_pos,x_pos);
    };

    double HexapodIK::computeFemurAngle(double x_pos, double z_pos, double theta1_, double r_1, double r_2) {
        double phi_1 = std::atan2(z_pos, r_2);
        double phi_2 = std::acos((pow(L3_, 2) - pow(L2_, 2) - pow(r_1, 2)) / (-2 * L2_ * r_1));
        return phi_1 + phi_2;
    };

    double HexapodIK::computeTibiaAngle(double r_1) {
        double phi_3 = std::acos((pow(r_1, 2) - pow(L2_, 2) - pow(L3_, 2)) / (-2 * L2_ * L3_));
        return -(M_PI - phi_3);
    };
}