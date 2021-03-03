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

//C++
#include <string>

//FRC Includes
#include <frc/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

// 302 Includes
#include <auton/primitives/DrivePath.h>

using namespace std;
using namespace frc;

using namespace wpi::math;

DrivePath::DrivePath() : m_chassis(SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
                         m_timer(make_unique<Timer>()),
                         m_maxTime(0.0),
                         m_currentChassisPosition(units::meter_t(0), units::meter_t(0), units::radian_t(0)),
                         m_holoController(frc2::PIDController{1, 0, 0}, 
                                                 frc2::PIDController{1, 0, 0}, 
                                                 frc::ProfiledPIDController<units::radian>{1, 0, 0, 
                                                 frc::TrapezoidProfile<units::radian>::Constraints{6.28_rad_per_s, 3.14_rad_per_s / 1_s}})
                                                 //max velocity of 1 rotation per second and a max acceleration of 180 degrees per second squared.
{
  Logger::GetLogger()->ToNtTable("DrivePath", "Initialized", "False");
  Logger::GetLogger()->ToNtTable("DrivePath", "Running", "False");
  Logger::GetLogger()->ToNtTable("DrivePath", "Done", "False");
  Logger::GetLogger()->ToNtTable("DrivePath", "Times Ran", 0);
}
void DrivePath::Init(PrimitiveParams *params)
{

 Logger::GetLogger()->ToNtTable("DrivePath", "Initialized", "True");
  
 sPath2Load = params->GetPathName();     //sPath2Load = "Slalom1.wpilib.json" example
                                         // 
 Logger::GetLogger()->LogError(string("DrivePath - Loaded = "), sPath2Load);

  if (sPath2Load != "") // only go if path name found
  {

    Logger::GetLogger()->LogError(string("DrivePath"), string("Finding Deploy Directory"));

    // Read path into trajectory for deploy directory.  JSON File ex. Bounce1.wpilid.json
    wpi::SmallString<64> deployDir;
    frc::filesystem::GetDeployDirectory(deployDir);
    wpi::sys::path::append(deployDir, "paths");
    wpi::sys::path::append(deployDir, sPath2Load); // load path from deploy directory

    Logger::GetLogger()->LogError(string("Deploy path is "), deployDir.str());

    m_timer->Reset();
    m_timer->Start();

    Logger::GetLogger()->LogError(string("At line"), string("64, grabbing trajectory from json"));
    // set current position to initial position which should be first node in path.
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDir);
    Logger::GetLogger()->LogError(string("At line"), string("66, before setting chassis pos to trajectory"));
    //m_currentChassisPosition = m_trajectory.InitialPose();

    Logger::GetLogger()->LogError(string("At line"), string("67, just set current chassis pos to trajectory"));

    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosX", m_currentChassisPosition.X().to<double>());
    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosY", m_currentChassisPosition.Y().to<double>());

    m_PosChgTimer.Start(); // start scan timer to detect motion
  }
}
void DrivePath::Run()
{
  // Update odometry.
  //m_chassis->UpdateOdometry();  done in robot.cpp

  //Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentTimeNoIf", m_timer.get()->Get());

  Logger::GetLogger()->ToNtTable("DrivePath", "Running", "True");
 
  if (sPath2Load != "")
  {

    Logger::GetLogger()->ToNtTable("DrivePath", "Found Path = ", sPath2Load);

      int timesRan = 1;

      auto desiredPose = m_trajectory.Sample(units::second_t(m_timer.get()->Get() + 0.02));
      // Get the reference chassis speeds from the Ramsete Controller.
      m_currentChassisPosition = m_chassis.get()->GetPose().GetEstimatedPosition();
      //Pose2d pull out attributes
      Logger::GetLogger()->ToNtTable("DrivePathValues", "DesiredPoseX", desiredPose.pose.X().to<double>());
      Logger::GetLogger()->ToNtTable("DrivePathValues", "DesiredPoseY", desiredPose.pose.Y().to<double>());
      Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosX", m_currentChassisPosition.X().to<double>());
      Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosY", m_currentChassisPosition.Y().to<double>());
      Logger::GetLogger()->ToNtTable("DeltaValues", "DeltaX", desiredPose.pose.X().to<double>() - m_currentChassisPosition.X().to<double>());
      Logger::GetLogger()->ToNtTable("DeltaValues", "DeltaY", desiredPose.pose.Y().to<double>() - m_currentChassisPosition.Y().to<double>());


      /*
      Logger::GetLogger()->ToNtTable("EncoderValues", "BLEncoder", to_string(m_chassis->GetBackLeft()->GetEncoderValues()));
      Logger::GetLogger()->ToNtTable("EncoderValues", "BREncoder", to_string(m_chassis->GetBackRight()->GetEncoderValues()));
      Logger::GetLogger()->ToNtTable("EncoderValues", "FLEncoder", to_string(m_chassis->GetFrontLeft()->GetEncoderValues()));
      Logger::GetLogger()->ToNtTable("EncoderValues", "FREncoder", to_string(m_chassis->GetFrontRight()->GetEncoderValues()));
      */

      Logger::GetLogger()->ToNtTable("DrivePath", "Times Ran", to_string(timesRan));
      timesRan++;

      //m_ramseteController.SetEnabled(true);

      //switching to holo controller would take in same args for Calculate 
      //auto refChassisSpeeds = m_ramseteController.Calculate(m_currentChassisPosition, desiredPose);
      auto refChassisSpeeds = m_holoController.Calculate(m_currentChassisPosition, desiredPose, desiredPose.pose.Rotation());

      Logger::GetLogger()->ToNtTable("DrivePathValues", "ChassisSpeedsX", refChassisSpeeds.vx());
      Logger::GetLogger()->ToNtTable("DrivePathValues", "ChassisSpeedsY", refChassisSpeeds.vy());

      Logger::GetLogger()->ToNtTable("DrivePathValues", "TrajectoryTotalTime", m_trajectory.TotalTime().to<double>());
      Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentTime", m_timer.get()->Get());

      //refChassisSpeeds.omega = units::radians_per_second_t(0);
      m_chassis->Drive(refChassisSpeeds, false);
      
      //m_chassis->Drive( 0.5, 0, 0, false);

      Logger::GetLogger()->LogError(string("DrivePath - Running Path = "), sPath2Load);    

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
}
bool DrivePath::IsDone()
{

  if (sPath2Load != "")
  {
    bool bTimeDone = units::second_t(m_timer.get()->Get()) >= m_trajectory.TotalTime();

  //  sPath2Load = "";)
  m_currentChassisPosition = m_chassis.get()->GetPose().GetEstimatedPosition();
    if ( lRobotStopped(m_trajectory.Sample(m_trajectory.TotalTime()).pose, m_currentChassisPosition))
      //bTimeDone && m_bRobotStopped
    {
      Logger::GetLogger()->ToNtTable("DrivePath", "Done", "True");
      Logger::GetLogger()->LogError(string("DrivePath - DONE = "), sPath2Load);
    }
    return (bTimeDone // && m_bRobotStopped
    );
  }
  else
  {
    return false;
  }
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

