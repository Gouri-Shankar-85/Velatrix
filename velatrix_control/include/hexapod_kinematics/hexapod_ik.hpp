/******************************************************************************
*  Filename:           hexapod_ik.hpp
*  Description:        Header defining hexapod single leg inverse kinematics
*  Author:             Gouri Shankar
******************************************************************************/

#pragma once

#include "hexapod_kinematics/hexapod_config.hpp"
#include "hexapod_kinematics/hexapod_body.hpp"

namespace hexapod_kinematics {

    class HexapodIK {

    public:
        HexapodIK();

        Eigen::Vector3d solverIK(int leg_id, const Eigen::Vector3d& foot_pos);    

    private:

        double L1_; //coxa length
        double L2_; //femur length
        double L3_; //tibia length

        double computeCoxaAngle(double x_pos, double y_pos);
        double computeFemurAngle(double x_pos, double z_pos, double theta1_,  double r_1, double r_2);
        double computeTibiaAngle(double r_1);
    };
}