
//C++ Includes
#include <memory>
#include <string>

#include <auton/GalacticSearchFinder.h>
#include <utils/Logger.h>

using namespace std;
using namespace frc;


/* Path Names to load should be definded as followed in xml file Case Sensitive.
      Note: GS is for galatic search and will select one of four possible search paths
      GS_A_Blue
      GS_A_Red
      GS_B_Blue
      GS_B_Red
*/

std::string GalacticSearchFinder::GetGSPathFromVisionTbl_FP()
{
   /*
        Network Table Name : "visionTable"
        Entries: 
        "horAngle"
        "CellDistance"
  */

    std::string lGSPath2Load = "";
    auto NetTable = inst.GetTable(sTableName);

    double dNTDistance = NetTable->GetNumber(sRef_TblCVDistance, 999.9);
    double dNTAngle = NetTable->GetNumber(sRef_TblCVAngle, 999.9);
    //debug

    // returns a 999.9 (default) if table not found
    if (dNTAngle == 999.9 || dNTDistance == 999.9)
    {
        Logger::GetLogger()->LogError(string("DrivePath"), string("visionTable No Read error 999.9"));
        return "NT_Error"; //network table not read returning default values.
    }

    //Convert from polar coordinates to Cartesian coordinates XY.
    units::meter_t Dis2d = units::meter_t(dNTDistance);
    frc::Rotation2d Rot2d = units::degree_t(dNTAngle);
    frc::Translation2d NT2dTransLate{Dis2d, Rot2d};

    Logger::GetLogger()->ToNtTable("visionTable", "dNTDistance", dNTDistance);
    Logger::GetLogger()->ToNtTable("visionTable", "dNTAngle", dNTAngle);

    //frc::SmartDashboard::PutNumber("GS Angle", 0);
    //frc::SmartDashboard::PutNumber("GS Distance", 0);

    // use field relative offset starting point of x .6096 (1.5ft)  y -2.285 (7.5ft) camara
    // position at starting point
    double d_TransX = (double)NT2dTransLate.X() + (0.6096); //1.5ft // always positive
    double d_TransY = (double)NT2dTransLate.Y() - (2.285);  // neg or positive

    Logger::GetLogger()->ToNtTable("visionTable", "d_TransX", d_TransX);
    Logger::GetLogger()->ToNtTable("visionTable", "d_TransY", d_TransY);

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

      if (nFoundCnt == 0)
        {
            Logger::GetLogger()->LogError(string("GS Path"), string("Error - GS Courses found = 0"));
            lGSPath2Load = "galactic_blue_a.xml"; // what to do if no targets found
        }

    if (nFoundCnt > 1) // if more than one course found then log error but run last found path
    {
        Logger::GetLogger()->LogError(string("GS Path"), lGSPath2Load);
        Logger::GetLogger()->LogError(string("GS Path"), string("Error - GS Courses found > 1"));
        return lGSPath2Load;
    }
    else
    {
        Logger::GetLogger()->LogError(string("GS Path"), lGSPath2Load);
        return lGSPath2Load; // returns Path Name for Galatic search or "TargetNotFound"
    }
}
bool GalacticSearchFinder::CheckTarget(double dTargets[], double dNT_X, double dNT_Y, double dPercentTolX, double dPercentTolY)
{

    //FIELD POSITION WITH ANGEL AND DISTANCE TRANSLATED TO FP
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


//*****************  NEW ANGLE ONLY ********************************************************

std::string GalacticSearchFinder::GetGSPathFromVisionTbl_Angle()
{
   /*
        Network Table Name : "visionTable"
        Entries: 
        "horAngle"
        "CellDistance"
  */

    std::string lGSPath2Load = "";
    auto NetTable = inst.GetTable(sTableName);

   // double dNTDistance = NetTable->GetNumber(sRef_TblCVDistance, 999.9);
    double dNTAngle = NetTable->GetNumber(sRef_TblCVAngle, 999.9);
    //debug

    // returns a 999.9 (default) if table not found
    if (dNTAngle == 999.9)
    {
       // Logger::GetLogger()->LogError(string("GS Path"), string("Angle visionTable No Read error 999.9"));
        Logger::GetLogger()->ToNtTable("visionTable", "Error", "No NT Read 999");
        return "NT_Error"; //network table not read returning default values.
    }

 
    Logger::GetLogger()->ToNtTable("visionTable", "dNTAngle", dNTAngle);

  
  //GS A Red
        // dNTAngle = 0; 
 
  //GS A Blue
        // dNTAngle = 44.0; 
     
  // GS B Red
        // dNTAngle = -44.5; 
             
  // GS B Blue
        // dNTAngle = 32; //maybe 37
     
  
    // x=lentgh of field  y=width  30ftX15ft
    double dPercentTol_Angle = .15;  // Allow +/- 15 Percent

    double GS_A_RedTarget = 0.0; double GS_A_RedUpWin = 10.0 ;double GS_A_RedLowWin = -10.0;// Vision angle = 0 
    double GS_A_BlueTarget = 44.0; double GS_A_BlueUpWin = 60.0 ;double GS_A_BlueLowWin = 40.0;// FP = x4.572,  y -3.81
    double GS_B_RedTarget = -44.5; double GS_B_RedUpWin = -25.0 ;double GS_B_RedLowWin = -65.0;//FP = x2.286,  y -1.524
    double GS_B_BlueTarget = 32.0; double GS_B_BlueUpWin = 39.9 ;double GS_B_BlueLowWin = 10.0;//FP = x4.572,  y -3.048

    bool TargetFound = false;
    int nFoundCnt = 0;

    
    TargetFound = CheckTarget_Angle(GS_A_RedTarget, dNTAngle,GS_A_RedUpWin,GS_A_RedLowWin);
    if (TargetFound)
    {
        lGSPath2Load = "galactic_red_a.xml";
        nFoundCnt++;
    }
    TargetFound = false;

    
    TargetFound = CheckTarget_Angle(GS_A_BlueTarget, dNTAngle,GS_A_BlueUpWin,GS_A_BlueLowWin);
    if (TargetFound)
    {
        lGSPath2Load = "galactic_blue_a.xml";
        nFoundCnt++;
    }

    TargetFound = false;
    
    TargetFound = CheckTarget_Angle(GS_B_RedTarget,dNTAngle,GS_B_RedUpWin,GS_B_RedLowWin);
    if (TargetFound)
    {
        lGSPath2Load = "galactic_red_b.xml";
        nFoundCnt++;
    }

    TargetFound = false;
    
    TargetFound = CheckTarget_Angle(GS_B_BlueTarget,dNTAngle,GS_B_BlueUpWin,GS_B_BlueLowWin);
    if (TargetFound)
    {
        lGSPath2Load = "galactic_blue_b.xml";
        nFoundCnt++;
    }

    if (nFoundCnt == 0)
        {
            Logger::GetLogger()->LogError(string("GS Path"), string("Angle Error - GS Courses found = 0"));
            Logger::GetLogger()->ToNtTable("visionTable", "Error", "Angle Not Found");
            Logger::GetLogger()->ToNtTable("visionTable", "Path", lGSPath2Load);
            lGSPath2Load = "galactic_blue_a.xml"; // what to do if no targets found
            return lGSPath2Load;
        }

    if (nFoundCnt > 1) // if more than one course found then log error but run last found path
    {
        Logger::GetLogger()->LogError(string("GS Path"), lGSPath2Load);
        Logger::GetLogger()->LogError(string("GS Path"), string("Error - GS Courses found > 1"));
        Logger::GetLogger()->ToNtTable("visionTable", "Path", lGSPath2Load);
        Logger::GetLogger()->ToNtTable("visionTable", "Error", "Found > 1");
        return lGSPath2Load;
    }
    else
    {
        Logger::GetLogger()->LogError(string("GS Path"), lGSPath2Load);
        Logger::GetLogger()->ToNtTable("visionTable", "Path", lGSPath2Load);
        Logger::GetLogger()->ToNtTable("visionTable", "Error", "No Error");
        return lGSPath2Load; // returns Path Name for Galatic search or "TargetNotFound"
    }
}

//***********************************************************************************
bool GalacticSearchFinder::CheckTarget_Angle(double dTarget,double DNT_Angle,  double d_UpWin, double d_LowWin)
{
  
  //  USING ANGLE ONLY FROM NETWORK TABLE AND FIND XML FILE TO RUN BASED ON ANGLE ONLY.
    bool lGSTargetFound = false;
    double d_UpperLim = d_UpWin;
    double d_LowerLim = d_LowWin;
    //Neg Angles
  

     // If value falls between upper and lower limits
    if (DNT_Angle >= d_LowerLim && DNT_Angle <= d_UpperLim)
    {
            lGSTargetFound = true;
    }
    return lGSTargetFound;
}

//**********************************************************************************
//********************************************************************************************

