
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
#include <subsys/DragonSwerveModule.h>

class SwerveChassis
{
    public:
        SwerveChassis( std::shared_ptr<DragonSwerveModule> frontleft, 
                       std::shared_ptr<DragonSwerveModule> frontright, 
                       std::shared_ptr<DragonSwerveModule> backleft, 
                       std::shared_ptr<DragonSwerveModule> backright, 
                       units::length::inch_t wheelDiameter,
                       units::length::inch_t wheelBase,
                       units::length::inch_t track,
                       units::velocity::meters_per_second_t maxSpeed,
                       double maxAcceleration ); 
        
        void Drive(units::velocity::meters_per_second_t xSpeed, units::velocity::meters_per_second_t ySpeed, units::angular_velocity::radians_per_second_t rot, bool fieldRelative);

        void UpdateOdometry();

        static constexpr auto MaxSpeed = 3.0_mps; 
        static constexpr units::angular_velocity::radians_per_second_t MaxAngularSpeed{wpi::math::pi};

        units::length::inch_t GetWheelDiameter() const {return m_wheelDiameter;}
        units::length::inch_t GetWheelBase() const {return m_wheelBase; }  
        units::length::inch_t GetTrack() const {return m_track;}
        units::velocity::meters_per_second_t GetMaxSpeed() const {return m_maxSpeed;}
        double GetMaxAcceration() const { return m_maxAcceleration; }


    private:
        frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
        frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
        frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
        frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

        std::shared_ptr<DragonSwerveModule> m_frontLeft;
        std::shared_ptr<DragonSwerveModule> m_frontRight;
        std::shared_ptr<DragonSwerveModule> m_backLeft;
        std::shared_ptr<DragonSwerveModule> m_backRight;

        units::length::inch_t                m_wheelDiameter;
        units::length::inch_t                m_wheelBase;       
        units::length::inch_t                m_track;
        units::velocity::meters_per_second_t m_maxSpeed;
        double                               m_maxAcceleration;

        DragonPigeon* m_pigeon = PigeonFactory::GetFactory()->GetPigeon();
        //frc::AnalogGyro m_gyro{0};

        frc::SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation};

        // Gains are for example purposes only - must be determined for your own
        // robot!
        frc::SwerveDrivePoseEstimator<4> m_poseEstimator{  frc::Rotation2d(), 
                                                           frc::Pose2d(), 
                                                           m_kinematics,
                                                           {0.1, 0.1, 0.1},   
                                                           {0.05},        
                                                           {0.1, 0.1, 0.1} };

        //frc::Translation2d::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
        //frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
};
