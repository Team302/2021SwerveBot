
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
#include <frc2/Timer.h>

// Team 302 includes
#include <subsys/SwerveChassis.h>



// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>




//#include "ExampleGlobalMeasurementSensor.h"

using namespace std;

SwerveChassis::SwerveChassis( shared_ptr<DragonSwerveModule> frontleft, 
                              shared_ptr<DragonSwerveModule> frontright, 
                              shared_ptr<DragonSwerveModule> backleft, 
                              shared_ptr<DragonSwerveModule> backright, 
                              units::length::inch_t wheelDiameter,
                              units::length::inch_t wheelBase,
                              units::length::inch_t track,
                              units::velocity::meters_per_second_t maxSpeed,
                              double maxAcceleration ) :  m_frontLeft(frontleft), 
                                                          m_frontRight(frontright), 
                                                          m_backLeft(backleft), 
                                                          m_backRight(backright), 
                                                          m_wheelDiameter(wheelDiameter),
                                                          m_wheelBase(wheelBase),
                                                          m_track(track),
                                                          m_maxSpeed(maxSpeed),
                                                          m_maxAcceleration(maxAcceleration)
{
  
}



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
