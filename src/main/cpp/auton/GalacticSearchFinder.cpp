
#pragma once

//C++ Includes
#include <memory>

#include <auton/GalacticSearchFinder.h>

using namespace std;
using namespace frc;

//using namespace wpi::math;

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

std::string GalacticSearchFinder::GetGalacticSearchPath()
{

  std::string lPath2Load = "";

  //routine to Fetch Galactic seach path based on vision results from network table.
  std::string lPath = GetGSPathFromVisionTbl();
  if (lPath == "GS_A_Red.wpilib.json" || lPath == "GS_A_Blue.wpilib.json" 
                                      || lPath == "GS_B_Red.wpilib.json" 
                                      || lPath == "GS_B_Blue.wpilib.json")
  {
    lPath2Load = lPath;
  }
  else
  {
    // error // anything other than a path name is an error text//
    Logger::GetLogger()->LogError(string("DrivePath"), string("GetGSPathFromVisionTbl return not Valid"));
    lPath2Load = "";
  }

  return lPath2Load;
}

std::string GalacticSearchFinder::GetGSPathFromVisionTbl()
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
 // double dNTDistance = NetTable->GetNumber(sRef_TblCVDistance, 999.9);
 // double dNTAngle = NetTable->GetNumber(sRef_TblCVAngle, 999.9);
  //debug
  double dNTDistance = NetTable->GetNumber("GS Distance", 999.9);
  double dNTAngle = NetTable->GetNumber("GS Angle", 999.9);
  
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
  double dPercentTolX = .15;  // Allow +/- 15 Percent
  double dPercentTolY = .075; // Allow +/- 7.5 Percent
  bool TargetFound = false;
  int nFoundCnt = 0;

  double GS_A_RedTarget[]{2.286, -2.286}; //FP = x2.286,  y -2.286
  TargetFound = CheckTarget(GS_A_RedTarget, d_TransX, d_TransY, dPercentTolX, dPercentTolY);
  if (TargetFound)
  {
    lGSPath2Load = "galactic_red_a.xml";
    nFoundCnt++;
  }
  TargetFound = false;

  double GS_A_BlueTarget[]{4.572, -3.81}; // FP = x4.572,  y -3.81
  TargetFound = CheckTarget(GS_A_BlueTarget, d_TransX, d_TransY, dPercentTolX, dPercentTolY);
  if (TargetFound)
  {
    lGSPath2Load = "galactic_blue_a.xml";
    nFoundCnt++;
  }

  TargetFound = false;
  double GS_B_RedTarget[]{2.286, -1.524}; //FP = x2.286,  y -1.524
  TargetFound = CheckTarget(GS_B_RedTarget, d_TransX, d_TransY, dPercentTolX, dPercentTolY);
  if (TargetFound)
  {
    lGSPath2Load = "galactic_red_b.xml";
    nFoundCnt++;
  }

  TargetFound = false;
  double GS_B_BlueTarget[]{4.572, -3.048}; //FP = x4.572,  y -3.048
  TargetFound = CheckTarget(GS_B_BlueTarget, d_TransX, d_TransY, dPercentTolX, dPercentTolY);
  if (TargetFound)
  {
    lGSPath2Load = "galactic_blue_b.xml";
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

bool GalacticSearchFinder::CheckTarget(double dTargets[], double dNT_X, double dNT_Y, double dPercentTolX, double dPercentTolY)
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