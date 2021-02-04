
//====================================================================================================================================================
// Copyright 2020 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once
#include <memory>

#include <frc/AnalogGyro.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/math>

#include <hw/factories/PigeonFactory.h>
#include <hw/DragonPigeon.h>
#include <subsys/SwerveModule.h>

class SwerveChassis
{
    public:
        /// @brief Construct a swerve chassis
        /// @param [in] std::shared_ptr<SwerveModule>     frontleft:          front left swerve module
        /// @param [in] std::shared_ptr<SwerveModule>     frontright:         front right swerve module
        /// @param [in] std::shared_ptr<SwerveModule>     backleft:           back left swerve module
        /// @param [in] std::shared_ptr<SwerveModule>     backright:          back right swerve module
        /// @param [in] units::length::inch_t                   wheelBase:          distance between the front and rear wheels
        /// @param [in] units::length::inch_t                   track:              distance between the left and right wheels
        /// @param [in] units::velocity::meters_per_second_t    maxSpeed:           maximum linear speed of the chassis 
        /// @param [in] units::radians_per_second_t             maxAngularSpeed:    maximum rotation speed of the chassis 
        /// @param [in] double                                  maxAcceleration:    maximum acceleration in meters_per_second_squared
        SwerveChassis( std::shared_ptr<SwerveModule>  frontleft, 
                       std::shared_ptr<SwerveModule>  frontright, 
                       std::shared_ptr<SwerveModule>  backleft, 
                       std::shared_ptr<SwerveModule>  backright, 
                       units::length::inch_t                wheelBase,
                       units::length::inch_t                track,
                       units::velocity::meters_per_second_t maxSpeed,
                       units::radians_per_second_t          maxAngularSpeed,
                       double                               maxAcceleration ); 

        /// @brief Align all of the swerve modules to point forward
        void ZeroAlignSwerveModules();

        /// @brief Drive the chassis
        /// @param [in] units::velocity::meters_per_second_t            xSpeed:         forward/reverse speed (positive is forward)
        /// @param [in] units::velocity::meters_per_second_t            ySpeed:         left/right speed (positive is left)
        /// @param [in] units::angular_velocity::radians_per_second_t   rot:            Rotation speed around the vertical (Z) axis; (positive is counter clockwise)
        /// @param [in] bool                                            fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
        ///                                                                             false: direction is based on robot front/back
        /// @param [in] units::length::inch_t                   wheelBase:          distance between the front and rear wheels
        void Drive(units::velocity::meters_per_second_t xSpeed, units::velocity::meters_per_second_t ySpeed, units::angular_velocity::radians_per_second_t rot, bool fieldRelative);
        void Drive( double drivePercent, double steerPercent, double rotatePercent, bool fieldRelative );

        void UpdateOdometry();

        //static constexpr auto MaxSpeed = 3.0_mps; 
        //static constexpr units::angular_velocity::radians_per_second_t MaxAngularSpeed{wpi::math::pi};

        units::length::inch_t GetWheelBase() const {return m_wheelBase; }  
        units::length::inch_t GetTrack() const {return m_track;}
        units::velocity::meters_per_second_t GetMaxSpeed() const {return m_maxSpeed;}
        units::radians_per_second_t GetMaxAngularSpeed() const {return m_maxAngularSpeed;}
        std::shared_ptr<SwerveModule> GetFrontLeft() const { return m_frontLeft;}
        std::shared_ptr<SwerveModule> GetFrontRight() const { return m_frontRight;}
        std::shared_ptr<SwerveModule> GetBackLeft() const { return m_backLeft;}
        std::shared_ptr<SwerveModule> GetBackRight() const { return m_backRight;}
        double GetMaxAcceleration() const { return m_maxAcceleration; }
        frc::SwerveDrivePoseEstimator<4> GetPose() { return m_poseEstimator; }       

    private:

        std::shared_ptr<SwerveModule> m_frontLeft;
        std::shared_ptr<SwerveModule> m_frontRight;
        std::shared_ptr<SwerveModule> m_backLeft;
        std::shared_ptr<SwerveModule> m_backRight;

        units::length::inch_t                m_wheelBase;       
        units::length::inch_t                m_track;
        units::velocity::meters_per_second_t m_maxSpeed;
        units::radians_per_second_t          m_maxAngularSpeed;
        double                               m_maxAcceleration;

        DragonPigeon*                        m_pigeon;
        
        //TODO:  these need to be calculated from the track and wheelbase, gains should probably be passed in
        frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
        frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
        frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
        frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};
        frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};

        // Gains are for example purposes only - must be determined for your own robot!
        //Clean up to get clearer information
        frc::SwerveDrivePoseEstimator<4> m_poseEstimator{  frc::Rotation2d(), 
                                                           frc::Pose2d(), 
                                                           m_kinematics,
                                                           {0.1, 0.1, 0.1},   
                                                           {0.05},        
                                                           {0.1, 0.1, 0.1} };
};
