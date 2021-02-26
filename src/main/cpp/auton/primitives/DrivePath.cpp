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

// 302 Includes

#include <auton/primitives/DrivePath.h>

using namespace std;
using namespace frc;

using namespace wpi::math;

DrivePath::DrivePath() : m_chassis(SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
                         m_timer(make_unique<Timer>()),
                         m_maxTime(0.0),
                         m_currentChassisPosition(units::meter_t(0), units::meter_t(0), units::radian_t(0))
{


  //m_nt = nt::NetworkTableInstance::GetDefault().GetTable("DrivePath");
  Logger::GetLogger()->ToNtTable("DrivePath", "Initialized", "False");
  Logger::GetLogger()->ToNtTable("DrivePath", "Running", "False");
  Logger::GetLogger()->ToNtTable("DrivePath", "Done", "False");
  Logger::GetLogger()->ToNtTable("DrivePath", "Times Ran", "0");
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

    // assume start angel of zero at start of path
    frc::Rotation2d StartAngle;
    StartAngle.Degrees() = units::degree_t(0);

    m_chassis->SetEncodersToZero();

    m_chassis->ResetPosition(m_trajectory.InitialPose(), StartAngle);

    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosX", to_string(m_currentChassisPosition.X().to<double>()));
    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosY", to_string(m_currentChassisPosition.Y().to<double>()));

    m_PosChgTimer.Start(); // start scan timer to detect motion
  }
}
void DrivePath::Run()
{
  // Update odometry.
  //m_chassis->UpdateOdometry();  done in robot.cpp

  //Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentTimeNoIf", to_string(m_timer.get()->Get()));

  Logger::GetLogger()->ToNtTable("DrivePath", "Running", "True");
 
  if (sPath2Load != "")
  {

    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentTimeBlankString", to_string(m_timer.get()->Get()));

    Logger::GetLogger()->ToNtTable("DrivePath", "Found Path = ", sPath2Load);

    if (units::second_t(m_timer.get()->Get()) < m_trajectory.TotalTime())
    {
      int timesRan = 1;

      auto desiredPose = m_trajectory.Sample(units::second_t(m_timer.get()->Get()));
      // Get the reference chassis speeds from the Ramsete Controller.
      m_currentChassisPosition = m_chassis.get()->GetPose().GetEstimatedPosition();
      //Pose2d pull out attributes
      Logger::GetLogger()->ToNtTable("DrivePathValues", "DesiredPoseX", to_string(desiredPose.pose.X().to<double>()));
      Logger::GetLogger()->ToNtTable("DrivePathValues", "DesiredPoseY", to_string(desiredPose.pose.Y().to<double>()));
      Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosX", to_string(m_currentChassisPosition.X().to<double>()));
      Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosY", to_string(m_currentChassisPosition.Y().to<double>()));

      Logger::GetLogger()->ToNtTable("DrivePath", "Times Ran", to_string(timesRan));
      timesRan++;

      m_ramseteController.SetEnabled(true);

      auto refChassisSpeeds = m_ramseteController.Calculate(m_currentChassisPosition, desiredPose);

      Logger::GetLogger()->ToNtTable("DrivePathValues", "ChassisSpeedsX", to_string(refChassisSpeeds.vx()));
      Logger::GetLogger()->ToNtTable("DrivePathValues", "ChassisSpeedsY", to_string(refChassisSpeeds.vy()));

      Logger::GetLogger()->ToNtTable("DrivePathValues", "TrajectoryTotalTime", to_string(m_trajectory.TotalTime().to<double>()));
      Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentTime", to_string(m_timer.get()->Get()));


      m_chassis->Drive(refChassisSpeeds, false);
      
      //m_chassis->Drive( 0.5, 0, 0, false);

      Logger::GetLogger()->LogError(string("DrivePath - Running Path = "), sPath2Load);
    }
    else
    {
      //Logger::GetLogger()->LogError(string("DrivePath - Error = "), string("Current Time greater than trajectory total time"));
      //Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentTime", to_string(m_timer.get()->Get()));
      //Logger::GetLogger()->ToNtTable("DrivePathValues", "TrajectoryTotalTime", to_string(m_trajectory.TotalTime().to<double>()));
    }
    

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
    if ( bTimeDone //&& m_bRobotStopped
     )
    {
      Logger::GetLogger()->ToNtTable("DrivePath", "Done", "True");
      Logger::GetLogger()->LogError(string("DrivePath - DONE = "), sPath2Load);
    }
    return (bTimeDone //&& m_bRobotStopped
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

