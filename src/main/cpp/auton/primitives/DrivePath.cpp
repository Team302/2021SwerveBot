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

// 302 Includes

#include <auton/primitives/DrivePath.h> //Geo3

using namespace std;
using namespace frc;

using namespace wpi::math;

DrivePath::DrivePath() : m_chassis(SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
                         m_timer(make_unique<Timer>()),
                         m_maxTime(0.0),
                         m_currentChassisPosition(units::meter_t(0), units::meter_t(0), units::radian_t(0))
{
}

void DrivePath::Init(PrimitiveParams *params)
{

  // Read path into trajectory for deploy directory.  JSON File ex. Bounce1.wpilid.json
  wpi::SmallString<64> deployDir;
  frc::filesystem::GetDeployDirectory(deployDir);
  wpi::sys::path::append(deployDir, "paths");
  wpi::sys::path::append(deployDir, params->GetPathName());

  m_timer->Reset();
 

  // set current position to initial position which should be first node in path.
  m_currentChassisPosition = m_trajectory.InitialPose();

  // assume start angel of zero at start of path
  frc::Rotation2d StartAngle;
  StartAngle.Degrees() = units::degree_t(0);
  m_chassis.get()->ResetPosition(m_currentChassisPosition, StartAngle);
  
  
  m_PosChgTimer.Start(); // start scan timer to detect motion
}

void DrivePath::Run()
{
  // Update odometry.
  //m_chassis->UpdateOdometry();  done in robot.cpp

  if (units::second_t(m_timer.get()->Get()) < m_trajectory.TotalTime())
  {
    auto desiredPose = m_trajectory.Sample(units::second_t(m_timer.get()->Get()));
    // Get the reference chassis speeds from the Ramsete Controller.
    m_currentChassisPosition = m_chassis.get()->GetPose().GetEstimatedPosition();
    auto refChassisSpeeds = m_ramseteController.Calculate(m_currentChassisPosition, desiredPose);
    m_chassis->Drive(refChassisSpeeds, false); //
  } //if (units::second_t(m_timer.get()->Get()) < m_trajectory.TotalTime())

  // Motion Detection //
  if (m_PosChgTimer.Get() >= .75) // Scan time for comparing current pose with previous pose
  {
    m_CurPos = m_chassis.get()->GetPose().GetEstimatedPosition();
    m_bRobotStopped = lRobotStopped(m_CurPos, m_PrevPos);
    m_PrevPos = m_CurPos;
    m_PosChgTimer.Reset(); //reset scan for change timer
  }
  //////////////////////////////////////////////
}

bool DrivePath::IsDone()
{

  bool bTimeDone = units::second_t(m_timer.get()->Get()) >= m_trajectory.TotalTime();
  return (bTimeDone && m_bRobotStopped);
}

bool DrivePath::lRobotStopped(frc::Pose2d lCurPos, frc::Pose2d lPrevPos)
{

  // detect if motion has stopped /////////////////////////

  bool lresultStopped = false;                      // motion stopped
  double dCurPosX = lCurPos.X().to<double>() * 100; //cm
  double dCurPosY = lCurPos.Y().to<double>() * 100;
  double dPrevPosX = lPrevPos.X().to<double>() * 100;
  double dPrevPosY = lPrevPos.Y().to<double>() * 100;

  int iCurPosX = dCurPosX;
  int iCurPosY = dCurPosY;

  int iPrevPosX = dPrevPosX; //remove decimals
  int iPrevPosY = dPrevPosY;

  int iDeltaX = abs(iPrevPosX - iCurPosX);
  int iDeltaY = abs(iPrevPosY - iCurPosY);

  int iMovThresHold = 1; // cm used for detecting no motion adust this for encoder noise if needed.

  //  If Position of X or Y has moved since last scan..  Using Delta X/Y
  if (iDeltaX <= iMovThresHold && iDeltaY <= iMovThresHold)
  { //STOPPED
    lresultStopped = true;
  }
  else // MOVING
  {
    //Robot is Moving
    lresultStopped = false;
  }
  return lresultStopped;
}
/*  NOTES Users can create their own constraint by implementing the TrajectoryConstraint interface.
SwerveDriveKinematicsConstraint: Limits the velocity of the robot around turns
such that no wheel of a swerve-drive robot goes over a specified maximum velocity.

he MaxVelocity method should return the maximum allowed velocity for the given pose, curvature, and original velocity of the trajectory without any constraints. 
The MinMaxAcceleration method should return the minimum 
and maximum allowed acceleration for the given pose, curvature, and constrained velocity.

*/