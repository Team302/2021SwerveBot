
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
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

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
        SwerveChassis
        ( 
			std::shared_ptr<SwerveModule>                               frontLeft, 
			std::shared_ptr<SwerveModule>                               frontRight,
			std::shared_ptr<SwerveModule>                               backLeft, 
			std::shared_ptr<SwerveModule>                               backRight, 
			units::length::inch_t                                       wheelBase,
			units::length::inch_t                                       track,
			units::velocity::meters_per_second_t                        maxSpeed,
			units::radians_per_second_t                                 maxAngularSpeed,
			units::acceleration::meters_per_second_squared_t            maxAcceleration,
			units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration
        );

        /// @brief Align all of the swerve modules to point forward
        void ZeroAlignSwerveModules();

        /// @brief Drive the chassis
        /// @param [in] units::velocity::meters_per_second_t            xSpeed:         forward/reverse speed (positive is forward)
        /// @param [in] units::velocity::meters_per_second_t            ySpeed:         left/right speed (positive is left)
        /// @param [in] units::angular_velocity::radians_per_second_t   rot:            Rotation speed around the vertical (Z) axis; (positive is counter clockwise)
        /// @param [in] bool                                            fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
        ///                                                                             false: direction is based on robot front/back
        void Drive(units::velocity::meters_per_second_t xSpeed, 
                   units::velocity::meters_per_second_t ySpeed, 
                   units::angular_velocity::radians_per_second_t rot, 
                   bool fieldRelative);

        /// @brief Drive the chassis
        /// @param [in] double  drivePercent:   forward/reverse percent output (positive is forward)
        /// @param [in] double  steerPercent:   left/right percent output (positive is left)
        /// @param [in] double  rotatePercent:  Rotation percent output around the vertical (Z) axis; (positive is counter clockwise)
        /// @param [in] bool    fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
        ///                                     false: direction is based on robot front/back
        void Drive(double drivePercent, double steerPercent, double rotatePercent, bool fieldRelative );

        /// @brief Drive the chassis
        /// @param [in] frc::ChassisSpeeds  speeds:         kinematics for how to move the chassis
        /// @param [in] bool                fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
        ///                                                 false: direction is based on robot front/back
        void Drive(frc::ChassisSpeeds speeds, bool fieldRelative);

        /// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
        void UpdateOdometry();

        /// @brief Provide the current chassis speed information
        frc::ChassisSpeeds GetChassisSpeeds() const;

        /// @brief Reset the current chassis pose based on the provided pose and rotation
        /// @param [in] const Pose2d&       pose        Current XY position
        /// @param [in] const Rotation2d&   angle       Current rotation angle
        void ResetPosition
        ( 
            const frc::Pose2d&       pose,
            const frc::Rotation2d&   angle
        );

        /// @brief Reset the current chassis pose based on the provided pose (the rotation comes from the Pigeon)
        /// @param [in] const Pose2d&       pose        Current XY position
        void ResetPosition
        ( 
            const frc::Pose2d&       pose
        );

        //static constexpr auto MaxSpeed = 3.0_mps; 
        //static constexpr units::angular_velocity::radians_per_second_t MaxAngularSpeed{wpi::math::pi};

        units::length::inch_t GetWheelBase() const {return m_wheelBase; }  
        units::length::inch_t GetTrack() const {return m_track;}
        units::velocity::meters_per_second_t GetMaxSpeed() const {return m_maxSpeed;}
        units::radians_per_second_t GetMaxAngularSpeed() const {return m_maxAngularSpeed;}
        units::acceleration::meters_per_second_squared_t GetMaxAcceleration() const { return m_maxAcceleration; }
        units::angular_acceleration::radians_per_second_squared_t GetMaxAngularAcceleration() const { return m_maxAngularAcceleration; }
        std::shared_ptr<SwerveModule> GetFrontLeft() const { return m_frontLeft;}
        std::shared_ptr<SwerveModule> GetFrontRight() const { return m_frontRight;}
        std::shared_ptr<SwerveModule> GetBackLeft() const { return m_backLeft;}
        std::shared_ptr<SwerveModule> GetBackRight() const { return m_backRight;}
        frc::SwerveDrivePoseEstimator<4> GetPose() const { return m_poseEstimator; }  

        void SetDriveScaleFactor( double scale ) { m_scale = scale;}

    private:

        std::shared_ptr<SwerveModule>                               m_frontLeft;
        std::shared_ptr<SwerveModule>                               m_frontRight;
        std::shared_ptr<SwerveModule>                               m_backLeft;
        std::shared_ptr<SwerveModule>                               m_backRight;

        units::length::inch_t                                       m_wheelBase;       
        units::length::inch_t                                       m_track;
        units::velocity::meters_per_second_t                        m_maxSpeed;
        units::radians_per_second_t                                 m_maxAngularSpeed;
        units::acceleration::meters_per_second_squared_t            m_maxAcceleration;
        units::angular_acceleration::radians_per_second_squared_t   m_maxAngularAcceleration;

        DragonPigeon*                                               m_pigeon;
        double                                                      m_scale;
        
        frc::Translation2d m_frontLeftLocation;
        frc::Translation2d m_frontRightLocation;
        frc::Translation2d m_backLeftLocation;
        frc::Translation2d m_backRightLocation;

        frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};

        // Gains are for example purposes only - must be determined for your own robot!
        //Clean up to get clearer information
        frc::SwerveDrivePoseEstimator<4> m_poseEstimator{  frc::Rotation2d(), 
                                                           frc::Pose2d(), 
                                                           m_kinematics,
                                                           {0.1, 0.1, 0.1},   // state standard deviations
                                                           {0.05},            // local measurement standard deviations
                                                           {0.1, 0.1, 0.1} }; // vision measurement standard deviations
};
