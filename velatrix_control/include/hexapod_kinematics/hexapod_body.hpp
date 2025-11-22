/******************************************************************************
*  Filename:           hexapod_body.hpp
*  Description:        Header defining the transformations for the hexapod body frame and leg frames.
*  Author:             Gouri Shankar
******************************************************************************/

#pragma once

#include <eigen3/Eigen/Dense>
#include "hexapod_kinematics/hexapod_config.hpp"

namespace hexapod_kinematics {

    class HexapodBody {

    public:
        HexapodBody();

        // Transform point from body frame to leg frame
        Eigen::Vector3d transformBodyToLeg(int leg_id, const Eigen::Vector3d& point_in_body);

        // Transform point from leg frame to body frame
        Eigen::Vector3d transformLegToBody(int leg_id, const Eigen::Vector3d& point_in_leg);
    };
}