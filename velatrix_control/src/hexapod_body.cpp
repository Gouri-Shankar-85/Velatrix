/*********************************************************************************************************
*  Filename:           hexapod_body.cpp
*  Description:        Define the transformations functions for the hexapod body frame and leg frames.
*  Author:             Gouri Shankar
*********************************************************************************************************/

#include "hexapod_kinematics/hexapod_body.hpp"
#include <cmath>

namespace hexapod_kinematics {

    HexapodBody::HexapodBody() {
        // Constructor implementation 
    };

    Eigen::Vector3d HexapodBody::transformBodyToLeg(int leg_id, const Eigen::Vector3d& point_in_body) {

        const auto& mount = hexapod_kinematics::LegMountConfig::getLegMount(leg_id);

        // Translation to leg mount point
        Eigen::Vector3d translated;
        translated.x() = point_in_body.x() - mount.x;
        translated.y() = point_in_body.y() - mount.y;
        translated.z() = point_in_body.z() - mount.z;

        // Rotation - inverse yaw rotation
        double cos_angle = std::cos(-mount.angle);
        double sin_angle = std::sin(-mount.angle);

        // Final transformation
        Eigen::Vector3d point_in_leg;
        point_in_leg.x() = cos_angle * translated.x() - sin_angle * translated.y();
        point_in_leg.y() = sin_angle * translated.x() + cos_angle * translated.y();
        point_in_leg.z() = translated.z();

        return point_in_leg;
    };

    Eigen::Vector3d HexapodBody::transformLegToBody(int leg_id, const Eigen::Vector3d& point_in_leg) {

        const auto& mount = hexapod_kinematics::LegMountConfig::getLegMount(leg_id);

        // Translation - leg mount point
        Eigen::Vector3d translated;
        translated.x() = point_in_leg.x() + mount.x;
        translated.y() = point_in_leg.y() + mount.y;
        translated.z() = point_in_leg.z() + mount.z;

        // Rotation - forward yaw rotation
        double cos_angle = std::cos(mount.angle);  
        double sin_angle = std::sin(mount.angle);

        // Final transformation
        Eigen::Vector3d point_in_body;
        point_in_body.x() = cos_angle * translated.x() - sin_angle * translated.y();
        point_in_body.y() = sin_angle * translated.x() + cos_angle * translated.y();
        point_in_body.z() = translated.z();

        return point_in_body;
    };
        
}
