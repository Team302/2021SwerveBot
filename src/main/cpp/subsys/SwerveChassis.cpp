
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

// Team 302 includes
#include <subsys/SwerveChassis.h>



// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>




//#include "ExampleGlobalMeasurementSensor.h"

using namespace std;
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
SwerveChassis::SwerveChassis( shared_ptr<SwerveModule>          frontleft, 
                              shared_ptr<SwerveModule>          frontright, 
                              shared_ptr<SwerveModule>          backleft, 
                              shared_ptr<SwerveModule>          backright, 
                              units::length::inch_t                   wheelBase,
                              units::length::inch_t                   track,
                              units::velocity::meters_per_second_t    maxSpeed,
                              units::radians_per_second_t             maxAngularSpeed,
                              double                                  maxAcceleration ) : m_frontLeft(frontleft), 
                                                                                          m_frontRight(frontright), 
                                                                                          m_backLeft(backleft), 
                                                                                          m_backRight(backright), 
                                                                                          m_wheelBase(wheelBase),
                                                                                          m_track(track),
                                                                                          m_maxSpeed(maxSpeed),
                                                                                          m_maxAngularSpeed(maxAngularSpeed),
                                                                                          m_maxAcceleration(maxAcceleration),
                                                                                          m_pigeon(PigeonFactory::GetFactory()->GetPigeon())
{
    frontleft.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration );
    frontright.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration );
    backleft.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration );
    backright.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration );
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
/// @param [in] units::length::inch_t                   wheelBase:          distance between the front and rear wheels
void SwerveChassis::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) 
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d r2d {yaw};
    auto states = m_kinematics.ToSwerveModuleStates(
                                fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                xSpeed, ySpeed, rot, r2d) : frc::ChassisSpeeds{xSpeed, ySpeed, rot} );

    m_kinematics.NormalizeWheelSpeeds(&states, m_maxSpeed);

    auto [fl, fr, bl, br] = states;

    m_frontLeft.get()->SetDesiredState(fl);
    m_frontRight.get()->SetDesiredState(fr);
    m_backLeft.get()->SetDesiredState(bl);
    m_backRight.get()->SetDesiredState(br);
}

void SwerveChassis::UpdateOdometry() 
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d r2d {yaw};
    m_poseEstimator.Update(r2d, m_frontLeft.get()->GetState(),
                                m_frontRight.get()->GetState(), 
                                m_backLeft.get()->GetState(),
                                m_backRight.get()->GetState());
}
