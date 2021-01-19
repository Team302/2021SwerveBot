#pragma once

#include <frc/AnalogGyro.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/math>

#include "DragonSwerveModule.h"

class SwerveChassis
{
    public:
        SwerveChassis() {m_gyro.Reset();}

        void Drive(units::velocity::meters_per_second_t xSpeed, units::velocity::meters_per_second_t ySpeed, units::angular_velocity::radians_per_second_t rot, bool fieldRelative);

        void UpdateOdometry();

        static constexpr auto MaxSpeed = 3.0_mps; 
        static constexpr units::angular_velocity::radians_per_second_t MaxAngularSpeed{wpi::math::pi};

    private:
        frc::Translation2d::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
};