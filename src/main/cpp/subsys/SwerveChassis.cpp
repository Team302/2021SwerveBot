#include "SwerveChassis.h"


#include <frc2/Timer.h>

//#include "ExampleGlobalMeasurementSensor.h"

SwerveChassis::SwerveChassis(DragonSwerveModule* frontleft, DragonSwerveModule* frontright, DragonSwerveModule* backleft, DragonSwerveModule* backright, units::velocity::meters_per_second_t maxSpeed) : 
m_frontLeft(frontleft), m_frontRight(frontright), m_backLeft(backleft), m_backRight(backright), m_maxSpeed(maxSpeed)
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

  m_frontLeft->SetDesiredState(fl);
  m_frontRight->SetDesiredState(fr);
  m_backLeft->SetDesiredState(bl);
  m_backRight->SetDesiredState(br);
}

void SwerveChassis::UpdateOdometry() 
{
  units::degree_t yaw{m_pigeon->GetYaw()};
  Rotation2d r2d {yaw};
  m_poseEstimator.Update(r2d, m_frontLeft->GetState(),
                              m_frontRight->GetState(), 
                              m_backLeft->GetState(),
                              m_backRight->GetState());
}
