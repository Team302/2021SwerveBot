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

#include <auton/primitives/DrivePath.h>

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
  /* Path Names to load should be definded as followed in xml file Case Sensitive.
      Bounce1,Bounce2,Bounce3,Bounce4
      Barrel1
      GS  
      Slalom1

      Note: GS is for galatic search and will select one of four possible search paths
      GS_A_Blue
      GS_A_Red
      GS_B_Blue
      GS_B_Red
*/
  if (params->GetID() == DRIVE_PATH)
  {
    sPath2Load = params->GetPathName(); //Get Path Name from Primitives if Path Name = "GS" then
                                        // determine which galatic search path to run from network table

    if (sPath2Load == "GS") // If primitives passes "GS"  then get path based on vision results
    {
      //routine to Fetch Galactic seach path based on vision results from network table.
      std::string lPath = lGetGSPathFromVisionTbl();
      if (lPath == "GS_A_Red" || lPath == "GS_A_Blue" || lPath == "GS_B_Red" || lPath == "GS_B_Blue")
      {
        sPath2Load = lPath;
      }
      else
      {
        // error // anything other than a path name is and error text//
        Logger::GetLogger()->LogError(string("DrivePath"), string("lGetGSPathFromVisionTbl return not Valid"));
        sPath2Load = "";
      }
    }

    if (sPath2Load != "") // only go if path name found
    {

      // Read path into trajectory for deploy directory.  JSON File ex. Bounce1.wpilid.json
      wpi::SmallString<64> deployDir;
      frc::filesystem::GetDeployDirectory(deployDir);
      wpi::sys::path::append(deployDir, "paths");
      wpi::sys::path::append(deployDir, sPath2Load); // load path from deploy directory

      m_timer->Reset();

      // set current position to initial position which should be first node in path.
      m_currentChassisPosition = m_trajectory.InitialPose();

      // assume start angel of zero at start of path
      frc::Rotation2d StartAngle;
      StartAngle.Degrees() = units::degree_t(0);
      m_chassis.get()->ResetPosition(m_currentChassisPosition, StartAngle);

      m_PosChgTimer.Start(); // start scan timer to detect motion
    }
  } //if (sPath2Load != "")
}
void DrivePath::Run()
{
  // Update odometry.
  //m_chassis->UpdateOdometry();  done in robot.cpp

  if (sPath2Load != "")
  {

    if (units::second_t(m_timer.get()->Get()) < m_trajectory.TotalTime())
    {
      auto desiredPose = m_trajectory.Sample(units::second_t(m_timer.get()->Get()));
      // Get the reference chassis speeds from the Ramsete Controller.
      m_currentChassisPosition = m_chassis.get()->GetPose().GetEstimatedPosition();
      auto refChassisSpeeds = m_ramseteController.Calculate(m_currentChassisPosition, desiredPose);
      m_chassis->Drive(refChassisSpeeds, false); //
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
    sPath2Load = "";
    return (bTimeDone && m_bRobotStopped);
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
std::string DrivePath::lGetGSPathFromVisionTbl()
{
  //get id
  //SwerveModule::ModuleID id = m_SwerveModule.get()->GetType();
  /* via chris castillo
        Network Table Name : "Vision Table"
        Entries: 
        "CellVisionLateralTranslation"
        "CellVisionLongitundinalTranslation"
  */
  std::string lGSPath2Load = "";
  auto NetTable = inst.GetTable(sTableName);
  double dNTDistance = NetTable->GetNumber(sRef_TblCVDistance, 999.9);
  double dNTAngle = NetTable->GetNumber(sRef_TblCVAngle, 999.9);  
  // returns a 999.9 (default) if table not found
  if (dNTAngle == 999.9 || dNTDistance == 999.9)
  {
    Logger::GetLogger()->LogError(string("DrivePath"), string("visionTable No Read error"));
    return "NT_Error"; //network table not read returning default values.
  }

  //Convert from polar coordinates to Cartesian coordinates XY.
  units::meter_t Dis2d = units::meter_t(dNTDistance); 
  frc::Rotation2d Rot2d = units::degree_t(dNTAngle);
  frc::Translation2d NT2dTransLate{Dis2d, Rot2d};
  // use field relative offset starting point of x .6096 (1.5ft)  y -2.285 (7.5ft) camara 
  // position at starting point
  double d_TransX = (double)NT2dTransLate.X() + (0.6096); //1.5ft // always positive
  double d_TransY = (double)NT2dTransLate.Y() - (2.285);  // neg or positive

  // setup window of %tolerance for detecting which path to run based on vision targets
  /*  GS_A_Blue, GS_A_Red, GS_B_Blue, GS_B_Red
  // Theroetical Calculations....    
  // field offset at start position X= 0.6096 Y= -2.286 camera start position    
  //GS A Red
        // double dAngle = 0;         FP = x2.286,  y -2.286     
        // double dDist = 1.676;         
  //GS A Blue
        //double dAngle = -21.255;    FP = x4.572,  y -3.81  
        //double dDist = 4.2096;          
  // GS B Red
        //  double dAngle = 24.444;   FP = x2.286,  y -1.524
        //  double dDist = 1.8415;         
  // GS B Blue
        //  double dAngle = -10.886;  FP = x4.572,  y -3.048 
        //  double dDist = 4.035;          
  */
  // x=lentgh of field  y=width  30ftX15ft
  double dPercentTolX = .15;   // Allow +/- 15 Percent
  double dPercentTolY = .075;  // Allow +/- 7.5 Percent
  bool TargetFound = false;
  int nFoundCnt = 0;

  double GS_A_RedTarget[]{2.286, -2.286 };  //FP = x2.286,  y -2.286 
  TargetFound = CheckTarget(GS_A_RedTarget,  d_TransX, d_TransY, dPercentTolX,dPercentTolY);
  if (TargetFound)
  {
    lGSPath2Load = "GS_A_Red";
    nFoundCnt++;
  }
 TargetFound = false;

  double GS_A_BlueTarget[]{4.572, -3.81}; // FP = x4.572,  y -3.81 
  TargetFound = CheckTarget(GS_A_BlueTarget, d_TransX, d_TransY, dPercentTolX,dPercentTolY);
  if (TargetFound)
  {
    lGSPath2Load = "GS_A_Blue";
    nFoundCnt++;
  }

  TargetFound = false;
  double GS_B_RedTarget[]{2.286, -1.524};  //FP = x2.286,  y -1.524
  TargetFound = CheckTarget(GS_B_RedTarget,  d_TransX, d_TransY, dPercentTolX,dPercentTolY);
  if (TargetFound)
  {
    lGSPath2Load = "GS_B_Red";
    nFoundCnt++;
  }

  TargetFound = false;
  double GS_B_BlueTarget[]{4.572, -3.048};  //FP = x4.572,  y -3.048 
  TargetFound = CheckTarget(GS_B_BlueTarget,  d_TransX, d_TransY, dPercentTolX,dPercentTolY);
  if (TargetFound)
  {
    lGSPath2Load = "GS_B_Blue";
    nFoundCnt++;
  }


  if (nFoundCnt > 1) // if more than one course found then return error
  {
    Logger::GetLogger()->LogError(string("DrivePath"), string("Error - GS Courses found > 1"));
    return "Error - GS Courses found > 1";
  }
  else
  {
    return lGSPath2Load; // returns Path Name for Galatic search or "TargetNotFound"
  }
}
bool DrivePath::CheckTarget(double dTargets[], double dNT_X, double dNT_Y, double dPercentTolX,double dPercentTolY)
{
  // return true if vision targets match Network table camera results
  //**NOTE Y can be negitive or positive.

  //targets [0] - X    [1] - Y
  // create a tolerance window and if the values from network table fall within the window
  // of theoretcial values then found=true
  bool lGSTargetFound = false;
  double dX_UpperLim = dTargets[0] + (dTargets[0] * dPercentTolX);
  double dX_LowerLim = dTargets[0] - (dTargets[0] * dPercentTolX);
  //swap for neg numbers Upper lim becomes lower lim
  double dY_LowerLim = dTargets[1] + (dTargets[1] * dPercentTolY); //-3.04
  double dY_UpperLim = dTargets[1] - (dTargets[1] * dPercentTolY); //

  // compare 2dtranslated numbers with theoretical windows
  if (dNT_X > dX_LowerLim && dNT_X < dX_UpperLim)
  {
    if (dNT_Y > dY_LowerLim && dNT_Y < dY_UpperLim)
    {
      lGSTargetFound = true;
    }
  }
  return lGSTargetFound;
}


