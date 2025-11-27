/******************************************************************************
*  Filename:           hexapod_gait.hpp
*  Description:        Header defining hexapod gait planner - Tripod Gait
*  Author:             Gouri Shankar
******************************************************************************/

#pragma once

#include <array>
#include <eigen3/Eigen/Dense>

namespace hexapod_gait {

    class HexapodGait {

    public:
        HexapodGait();

        // Set gait parameters
        void setGaitParameters(double step_length, double step_height, double step_duration);
        
        // Set default stance position
        void setDefaultStanceParameters(const std::array<Eigen::Vector3d, 6>& stance_positions);
        
        //Start/Stop walking
        void startWalking(double direction_angle);
        void stopWalking();

        // Get current foot positions
        std::array<Eigen::Vector3d, 6> getFootPositions(double current_time);
    
    private:

        // Gait parametrs
        double step_length_;    // T - forward/backward distance
        double step_height_;    // A - lift height during swing
        double step_duration_;  // Time for one complete cycle (seconds)
        double ground_level_;   // S - default foot height from body

        // Walking state
        bool is_walking_;
        double gait_start_time_;
        double walking_direction_; // in radians

        // Tripod Groups
        static constexpr std::array<int, 3> TRIPOD_A = {0,2,4};
        static constexpr std::array<int, 3> TRIPOD_B = {1,3,5};

        // Default stance positions
        std::array<Eigen::Vector3d, 6> default_stance_position_;

        // Beizer curve calculations
        Eigen::Vector3d beizerCurve(double t,
                                    const Eigen::Vector3d& P1,                                        
                                    const Eigen::Vector3d& P2,
                                    const Eigen::Vector3d& P3 );

        // Get swing phase positions
        Eigen::Vector3d getSwingPosition(double t, int leg_id);

        // Get stance phase positions
        Eigen::Vector3d getStancePositions(double t, int leg_id);

        // Rotate foot positions based on walking direction
        Eigen::Vector3d rotateFootPosition(const Eigen::Vector3d& point, int leg_id);

        // Determine if leg is in swing phase
        bool isLegInSwingPhase(int leg_id, double phase_time);

        // Get phase time [0,1] from absolute time
        double getPhaseTime(double current_time);
    };
}