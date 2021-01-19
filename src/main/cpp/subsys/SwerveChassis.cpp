#include "SwerveChassis.h"

#include <frc2/Timer.h>

//#include "ExampleGlobalMeasurementSensor.h"

void SwerveChassis::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.NormalizeWheelSpeeds(&states, maxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void SwerveChassis::UpdateOdometry() {
  m_poseEstimator.Update(m_gyro.GetRotation2d(), m_frontLeft.GetState(),
                         m_frontRight.GetState(), m_backLeft.GetState(),
                         m_backRight.GetState());
