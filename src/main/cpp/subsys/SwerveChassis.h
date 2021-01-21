#pragma once

#include <frc/AnalogGyro.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/math>
#include <hw/factories/PigeonFactory.h>
#include <hw/DragonPigeon.h>

#include "DragonSwerveModule.h"

class SwerveChassis
{
    public:
        SwerveChassis(DragonSwerveModule* frontleft, DragonSwerveModule* frontright, DragonSwerveModule* backleft, DragonSwerveModule* backright, units::velocity::meters_per_second_t maxSpeed); 
        

        void Drive(units::velocity::meters_per_second_t xSpeed, units::velocity::meters_per_second_t ySpeed, units::angular_velocity::radians_per_second_t rot, bool fieldRelative);

        void UpdateOdometry();

        static constexpr auto MaxSpeed = 3.0_mps; 
        static constexpr units::angular_velocity::radians_per_second_t MaxAngularSpeed{wpi::math::pi};

    private:
        frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
        frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
        frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
        frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

        DragonSwerveModule* m_frontLeft;
        DragonSwerveModule* m_frontRight;
        DragonSwerveModule* m_backLeft;
        DragonSwerveModule* m_backRight;

        units::velocity::meters_per_second_t m_maxSpeed;

        DragonPigeon* m_pigeon = PigeonFactory::GetFactory()->GetPigeon();
        //frc::AnalogGyro m_gyro{0};

        frc::SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation};

        // Gains are for example purposes only - must be determined for your own
        // robot!
        frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
        frc::Rotation2d(), frc::Pose2d(), m_kinematics,
        {0.1, 0.1, 0.1},   {0.05},        {0.1, 0.1, 0.1}};
};