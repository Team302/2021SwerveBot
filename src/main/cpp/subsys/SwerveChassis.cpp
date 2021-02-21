
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

// C++ Includes
#include <memory>

// FRC includes
//#include <frc2/Timer.h>

#include <frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

// Team 302 includes
#include <subsys/SwerveChassis.h>

// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>

//#include "ExampleGlobalMeasurementSensor.h"

using namespace std;
using namespace frc;

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
SwerveChassis::SwerveChassis
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
) : m_frontLeft(frontLeft), 
    m_frontRight(frontRight), 
    m_backLeft(backLeft), 
    m_backRight(backRight), 
    m_wheelBase(wheelBase),
    m_track(track),
    m_maxSpeed(maxSpeed),
    m_maxAngularSpeed(maxAngularSpeed),
    m_maxAcceleration(maxAcceleration),
    m_maxAngularAcceleration(maxAngularAcceleration),
    m_pigeon(PigeonFactory::GetFactory()->GetPigeon()),
    m_frontLeftLocation(wheelBase/2.0, track/2.0),
    m_frontRightLocation(wheelBase/2.0, -1.0*track/2.0),
    m_backLeftLocation(-1.0*wheelBase/2.0, track/2.0),
    m_backRightLocation(-1.0*wheelBase/2.0, -1.0*track/2.0)
{
    frontLeft.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration );
    frontRight.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration );
    backLeft.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration );
    backRight.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration );

    ZeroAlignSwerveModules();
}
/// @brief Align all of the swerve modules to point forward
void SwerveChassis::ZeroAlignSwerveModules()
{
    m_frontLeft.get()->ZeroAlignModule();
    m_frontRight.get()->ZeroAlignModule();
    m_backLeft.get()->ZeroAlignModule();
    m_backRight.get()->ZeroAlignModule();
}



/// @brief Drive the chassis
/// @param [in] units::velocity::meters_per_second_t            xSpeed:         forward/reverse speed (positive is forward)
/// @param [in] units::velocity::meters_per_second_t            ySpeed:         left/right speed (positive is left)
/// @param [in] units::angular_velocity::radians_per_second_t   rot:            Rotation speed around the vertical (Z) axis; (positive is counter clockwise)
/// @param [in] bool                                            fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
///                                                                             false: direction is based on robot front/back
void SwerveChassis::Drive( units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, 
                           bool fieldRelative) 
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d currentOrientation {yaw};
    auto states = m_kinematics.ToSwerveModuleStates(
                                fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                xSpeed, ySpeed, rot, currentOrientation) : frc::ChassisSpeeds{xSpeed, ySpeed, rot} );
    //auto states = m_kinematics.ToSwerveModuleStates( frc::ChassisSpeeds{xSpeed, ySpeed, rot}  );

    m_kinematics.NormalizeWheelSpeeds(&states, m_maxSpeed);

    auto [fl, fr, bl, br] = states;

    m_frontLeft.get()->SetDesiredState(fl);
    m_frontRight.get()->SetDesiredState(fr);
    m_backLeft.get()->SetDesiredState(bl);
    m_backRight.get()->SetDesiredState(br);
}

/// @brief Drive the chassis
/// @param [in] frc::ChassisSpeeds  speeds:         kinematics for how to move the chassis
/// @param [in] bool                fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
///                                                 false: direction is based on robot front/back
void SwerveChassis::Drive( ChassisSpeeds speeds, bool fieldRelative) 
{
    Drive( speeds.vx, speeds.vy, speeds.omega, fieldRelative);

    
   // double dmsg = speeds.vx.to<double>();
   // std::string msg = std::to_string(dmsg);

   //Logger::GetLogger()->LogError(string(" speeds.vx "),  msg );
 
}

/// @brief Drive the chassis
/// @param [in] double  drivePercent:   forward/reverse percent output (positive is forward)
/// @param [in] double  steerPercent:   left/right percent output (positive is left)
/// @param [in] double  rotatePercent:  Rotation percent output around the vertical (Z) axis; (positive is counter clockwise)
/// @param [in] bool    fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
///                                     false: direction is based on robot front/back
void SwerveChassis::Drive( double drive, double steer, double rotate, bool fieldRelative )
{
    if ( abs(drive) < 0.01 && abs(steer) < 0.01 && abs(rotate) < 0.01 )
    {
        // feed the motors
        m_frontLeft.get()->RunCurrentState();
        m_frontRight.get()->RunCurrentState();
        m_backLeft.get()->RunCurrentState();
        m_backRight.get()->RunCurrentState();       
    }
    else
    {    
        // scale joystick values to velocities using max chassis values
        auto maxSpeed = GetMaxSpeed();
        auto maxRotation = GetMaxAngularSpeed();

        units::velocity::meters_per_second_t driveSpeed = drive * maxSpeed;
        units::velocity::meters_per_second_t steerSpeed = steer * maxSpeed;
        units::angular_velocity::radians_per_second_t rotateSpeed = rotate * maxRotation;

        Drive( driveSpeed, steerSpeed, rotateSpeed, fieldRelative );
    }
}

/// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
void SwerveChassis::UpdateOdometry() 
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d r2d {yaw};

    m_poseEstimator.Update(r2d, m_frontLeft.get()->GetState(),
                                m_frontRight.get()->GetState(), 
                                m_backLeft.get()->GetState(),
                                m_backRight.get()->GetState());
}

/// @brief Provide the current chassis speed information
ChassisSpeeds SwerveChassis::GetChassisSpeeds() const
{
    return m_kinematics.ToChassisSpeeds({ m_frontLeft.get()->GetState(), 
                                          m_frontRight.get()->GetState(),
                                          m_backLeft.get()->GetState(),
                                          m_backRight.get()->GetState() });
}

/// @brief Reset the current chassis pose based on the provided pose and rotation
/// @param [in] const Pose2d&       pose        Current XY position
/// @param [in] const Rotation2d&   angle       Current rotation angle
void SwerveChassis::ResetPosition
( 
    const Pose2d&       pose,
    const Rotation2d&   angle
)
{
    m_poseEstimator.ResetPosition(pose, angle);
}


void SwerveChassis::ResetPosition
( 
    const Pose2d&       pose
)
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d angle {yaw};
    ResetPosition(pose, angle);
}

