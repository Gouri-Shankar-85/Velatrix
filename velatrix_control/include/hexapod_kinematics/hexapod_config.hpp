/******************************************************************************
*  Filename:           hexapod_config.hpp
*  Description:        Header defining hexapod leg dimensions and mounting points for all legs (constants only, no objects)
*  Author:             Gouri Shankar
******************************************************************************/

#pragma once

#include <array>

namespace hexapod_kinematics {

/**
    * @class LegDimensions
    * @brief Defines the dimensions of a single hexapod leg.
    * 
    * Single leg link dimensions in metres
    * dimensions measured from joint center to joint center
*/

    class LegDimensions {

    public:
        static constexpr double COXA_LENGTH = 0.05435;
        static constexpr double FEMUR_LENGTH = 0.0700;
        static constexpr double TIBIA_LENGTH = 0.16314;

        // Workspace calculation (total leg reach)
        static constexpr double MAX_REACH = COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH;
        static constexpr double MIN_REACH = TIBIA_LENGTH - FEMUR_LENGTH;

    private:
        LegDimensions() = delete; // Prevent instantiation
        LegDimensions(const LegDimensions&) = delete;
        LegDimensions& operator=(const LegDimensions&) = delete;
    };

/**
    * @class LegMountPoint
    * @brief Represents the mounting position of a single leg on the robot body.
    *
    * Each leg mount point stores the x, y, z coordinates (in metres) 
    * relative to the robot's base frame and the yaw angle in radians 
    * defining the orientation of the leg's coxa joint.
*/

    class LegMountPoint {

    public:
        double x;
        double y;
        double z;
        double angle; //yaw angle in radians

        constexpr LegMountPoint(double x_, double y_, double z_, double angle_):
            x(x_), y(y_), z(z_), angle(angle_) {}
    };

/**
    * @class LegMountConfig
    * @brief Contains the static configuration of all leg mount points for the hexapod.
    *
    * Provides a compile-time constant array of all LegMountPoint objects 
    * representing the position and orientation of each leg on the robot body.
    * NUM_LEGS defines the total number of legs.
    *
    * This class is non-instantiable; it only serves as a container for constants.
 */

    class LegMountConfig {

    public:
        static constexpr int NUM_LEGS = 6;

        static inline constexpr std::array<LegMountPoint, 6> LEG_MOUNTS = {{

            // Each leg coxa mount position w.r.t base_link frame
            LegMountPoint(0.039287, 0.06031, 0.05285, 0.9751),            
            LegMountPoint(0.070023, -0.00036228, 0.05285, 0.028762),
            LegMountPoint(0.041807, -0.05834, 0.05285, -0.92041),
            LegMountPoint(-0.039241, -0.061034, 0.05285, -2.2067),
            LegMountPoint(-0.069977, -0.00036228, 0.05285, -3.1161),
            LegMountPoint(-0.038726, 0.060651, 0.05285, 2.1693),            
        }};

        static constexpr const LegMountPoint& getLegMount(int leg_id) {
            return LEG_MOUNTS[leg_id];
        }

    private:
        LegMountConfig() = delete; 

    };
}