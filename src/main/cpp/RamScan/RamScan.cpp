//================================================================================================================
// Copyright 2021 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, // and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions
// of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
// TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//================================================================================================================

// RamScan.cpp

// C++ Includes
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string>

using namespace std;

// team 302 includes
#include <Robot.h>
// ?? //#include <auton/AutonSelector.h>
// ?? //#include <auton/CyclePrimitives.h>
// ?? //#include <auton/primitives/DriveDistance.h>
// ?? //#include <auton/primitives/SuperDrive.h>
#include <hw/factories/PigeonFactory.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/IDragonMotorControllerMap.h>
#include <hw/usages/MotorControllerUsage.h>
#include <RamScan/RamScan.h>
#include <states/chassis/SwerveDrive.h>
#include <subsys/SwerveChassisFactory.h>
#include <subsys/SwerveModule.h>
// ?? //#include <subsys/interfaces/IChassis.h>
// ?? //#include <subsys/IMechanism.h>
#include <subsys/MechanismFactory.h>
#include <utils/Logger.h>

// frc/wpi includes
#include <frc/Preferences.h>    // TODO: make a more user-friendly user input
#include <frc/SmartDashboard/SmartDashboard.h>

// constructor
RamScan::RamScan()
{
    m_bitMask       = 0;                // nothing enabled
    m_bitMaskPrev   = 0xDEADBEEF;       // Big Mac
    m_PacketCntr20ms = 0;
    m_prefs         = nullptr;
    m_RunMode       = INIT;

    //
    // pointers to objects containing variables that may be scanned
    //
    m_chassis       = nullptr;
    m_pigeon        = nullptr;
    m_teleopControl = nullptr;
}

// destructor
RamScan::~RamScan() {
    // TODO Auto-generated destructor stub
}


//
/// @brief This is called from RobotInit() after all other classes' objects are created.
/// @return void
void RamScan::Init()
{
    Logger::GetLogger()->LogError("RamScan::Init", "enter");
    frc::SmartDashboard::PutString("SWDate:", __DATE__);        // date that this line was compiled
    frc::SmartDashboard::PutString("SWTime:", __TIME__);        // time that this line was compiled

//
// TODO: make a more user-friendly user input
//  The user interface to define which variables are to be monitored needs work to be more user-friendly!!!
//      Each variable corresponds to a bit in m_bitMask; when the bit is set the variable is displayed.
//
    // get default bitmask for selection of debug values
    m_prefs = frc::Preferences::GetInstance();
    m_bitMask = m_prefs->GetLong("Pref Mask", 0x000F03FF);
    frc::SmartDashboard::PutNumber("Initial mask: ", m_bitMask);

    Logger::GetLogger()->LogError("RamScan::Init- m_bitMask size in bytes=", to_string(sizeof(m_bitMask)));

    //
    // get pointers to various objects
    //
    m_chassis = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis();
    if (m_chassis == nullptr)
    {   Logger::GetLogger()->LogError("RamScan::Init", "m_chassis = nullptr"); }

    m_pigeon = PigeonFactory::GetFactory()->GetPigeon();
    if (m_pigeon == nullptr)
    {   Logger::GetLogger()->LogError("RamScan::Init", "m_pigeon = nullptr"); }

    m_teleopControl = TeleopControl::GetInstance();
    if (m_teleopControl == nullptr)
    {   Logger::GetLogger()->LogError("RamScan::Init", "m_teleopControl = nullptr"); }


// ?? ////
// ?? ////  Get pointers to all the mechanisms
// ?? ////
// ?? //    auto Mfactory = MechanismFactory::GetMechanismFactory();
// ?? //
// ?? //    if ( Mfactory == nullptr )
// ?? //    {   Logger::GetLogger()->LogError("RamScan::Init", "Mfactory = nullptr"); }
// ?? //    // then what?


//
// Show which motors are not created & saved by SaveMechanismMotorUsageSaveMotorUsage() in MotorControllerArray.
//  Motors are indexed in MotorControllerArray by the enum MOTOR_CONTROLLER_USAGE in MotorControllerUsage.h
//
    printf("Missing motors:\n");
    for (int indx=0; indx < MotorControllerUsage::MAX_MOTOR_CONTROLLER_USAGES; ++indx)
    {
        if (MotorControllerArray[indx] == nullptr)
        {    printf (" %d,", indx); }
        else
        {    printf (","); }
    }
    printf ("<\n");

    Logger::GetLogger()->LogError("RamScan::Init", "exit");
}


/// @brief Put the values of the selected variables on the Smart Dashboard
//      This is called from RobotPeriodic() which is AFTER other periodic methods.
/// @return void
void RamScan::ScanVariables()
{
    m_PacketCntr20ms++;                 // count of 20ms PACKETS since power-on

// debug print every 5.12 sec
if ((m_PacketCntr20ms & 0x00FF) == 0)
{
    printf("....every 256 loops (5.12s)\n");
// ?? //    printf("Chassis current speed = %5.2f\n", m_chassis->GetCurrentSpeed());
// ?? //    printf("CurrentRightPosition = %5.2f\n", m_chassis->GetCurrentRightPosition());
// ?? //    printf("LD motor RPS = %5.1f\n",
// ?? //            MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::LEFT_DRIVE_MASTER]->GetRPS());
    printf("Driver speed = %4.3f\n", m_teleopControl->GetAxisValue(TeleopControl::SWERVE_DRIVE_DRIVE));
}

//
//  process each element that is selected
//
    unsigned long int   UnitBit = 1;    // start with mask in bit 0

    m_bitMask = m_prefs->GetLong("Pref Mask", 0);   //

    bool BitMaskChanged = (m_bitMask != m_bitMaskPrev);
    if (BitMaskChanged)
    {
        m_bitMaskPrev = m_bitMask;
        // TODO: display as hexadecimal or binary
        frc::SmartDashboard::PutNumber("m_bitMask(decimal)=", m_bitMask); // just for debugging...
    }

    //
    //  each bit that is set in BitMask enables one of the elements in ScanElementArray[]
    //
    for (unsigned int i=0; i < NUM_OF_SCAN_ELEMENTS; i++ )
    {
        if ((m_bitMask & UnitBit) != 0)         // is this bit set?
        {
            char*               ElementName = ScanElementArray[i].NamePtr;
            ScanElementType     ElementType = ScanElementArray[i].Type;
            (*ScanElementArray[i].AccAdd)();    // execute the method associated with this element

            //
            //  hey baby, what's your type?
            //
            switch (ElementType)
            {
                case BOOL:
                    frc::SmartDashboard::PutBoolean(ElementName, result_BOOL);
                    break;

                case CHAR:
                    frc::SmartDashboard::PutNumber(ElementName, result_CHAR);
                    break;

                case INT:
                    frc::SmartDashboard::PutNumber(ElementName, result_INT);
                    break;

                case LONG:
                    frc::SmartDashboard::PutNumber(ElementName, result_LONG);
                    break;

                case UCHAR:
                    frc::SmartDashboard::PutNumber(ElementName, result_UCHAR);
                    break;

                case UINT:
                    frc::SmartDashboard::PutNumber(ElementName, result_UINT);
                    break;

                case ULONG:
                    frc::SmartDashboard::PutNumber(ElementName, result_ULONG);
                    break;

                case FLOAT:
                    frc::SmartDashboard::PutNumber(ElementName, result_FLOAT);
                    break;

                case DOUBLE:
                    frc::SmartDashboard::PutNumber(ElementName, result_DOUBLE);
                    break;

                // //case STRING:
                 // //   frc::SmartDashboard::PutString(ElementName, result_STRING);
                 // //   break;

                default:
                    Logger::GetLogger()->LogError("RamScan::ScanVariables - at ", to_string(i));
                    Logger::GetLogger()->LogError(" invalid ElementType:",to_string(ScanElementArray[i].Type));
                    frc::SmartDashboard::PutNumber("No such type: ", ScanElementArray[i].Type);
            }// switch
        }// if set
        UnitBit <<= 1;                  // shift up to next bit
    }// for...
}


/// @brief Fill MotorControllerArray (indexed by the MOTOR_CONTROLLER_USAGE) with the motors used in mechanisms
/// @return void
void RamScan::SaveMechanismMotorUsage(  shared_ptr<IDragonMotorController>              motor,
                                        MotorControllerUsage::MOTOR_CONTROLLER_USAGE    usage)
{
    if ((usage < 0) || (usage >= MotorControllerUsage::MAX_MOTOR_CONTROLLER_USAGES))
    {
        Logger::GetLogger()->LogError("SaveMechanismMotorUsage- bad usage:", to_string(usage));
    }
    else
    {
        MotorControllerArray[static_cast<int>(usage)] = motor;
        Logger::GetLogger()->LogError("SaveMechanismMotorUsage for:", to_string(usage));
    }
}


//
/// @brief Save the swerve motor controllers from SwerveChassisFactory::CreateSwerveModule()
/// @return void
void RamScan::SaveSwerveMotorUsage( shared_ptr<IDragonMotorController>  driveMotor,
                                    shared_ptr<IDragonMotorController>  turnMotor,
                                    SwerveModule::ModuleID              position)   // which corner of robot
{
    if ((position < 0) || (position >= 4))
    {
        Logger::GetLogger()->LogError("SaveSwerveMotorUsage- bad position", to_string(position));
    }
    else
    {
        SwerveDriveMotorArray[static_cast<int>(position)] = driveMotor;
        SwerveTurnMotorArray[static_cast<int>(position)]  = turnMotor;
        Logger::GetLogger()->LogError("SaveSwerveMotorUsage for:", to_string(position));
    }
}


//
//  static variables in RamScan
//
    int                         RamScan::m_PacketCntr20ms;
    RamScan::RunModeType        RamScan::m_RunMode;

    shared_ptr<SwerveChassis>   RamScan::m_chassis;
    DragonPigeon*               RamScan::m_pigeon;
    TeleopControl*              RamScan::m_teleopControl;

    //  Arrays of motor controllers
    shared_ptr<IDragonMotorController> RamScan::SwerveDriveMotorArray[4];
    shared_ptr<IDragonMotorController> RamScan::SwerveTurnMotorArray[4];

    shared_ptr<IDragonMotorController>
        RamScan::MotorControllerArray[static_cast<int>(MotorControllerUsage::MAX_MOTOR_CONTROLLER_USAGES)];

    // mechanisms
    //??

    //
    //  result variables for each type
    //
    bool            RamScan::result_BOOL;
    char            RamScan::result_CHAR;
    int             RamScan::result_INT;
    long            RamScan::result_LONG;
    unsigned char   RamScan::result_UCHAR;
    unsigned int    RamScan::result_UINT;
    unsigned long   RamScan::result_ULONG;
    float           RamScan::result_FLOAT;
    double          RamScan::result_DOUBLE;
    string          RamScan::result_STRING;


//**********************************************************************************//
//                                                                                  //
//  These accessors set the 'result_*' variable of the appropriate type             //
//  but do not return a value, so they are all void methods with no parameters.     //
//                                                                                  //
//**********************************************************************************//

void RamScan::CrazyEights(void)             { result_INT = 8888;}   // for dummy functions
void RamScan::Get_RunMode(void)             { result_INT = static_cast<int>(m_RunMode);}
void RamScan::Get_PacketCntr20ms(void)      { result_INT = m_PacketCntr20ms;}

//
// pigeon variables
//
void RamScan::Get_Pigeon_Pitch(void)    {
    if (m_pigeon==nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = m_pigeon->GetPitch();} }

void RamScan::Get_Pigeon_Roll(void)     {
    if (m_pigeon==nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = m_pigeon->GetRoll();} }

void RamScan::Get_Pigeon_Yaw(void)      {
    if (m_pigeon==nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = m_pigeon->GetYaw();} }

//
//  accessors for chassis variables
//
void RamScan::Chassis_CurrentSpeed(void)
// ?? //{ if (m_chassis==nullptr)
{result_DOUBLE= 9999.99;}
// ?? //    else {result_DOUBLE = m_chassis->GetCurrentSpeed();}
// ?? //}

void RamScan::Chassis_CurrentPosition(void)
// ?? //{ if (m_chassis==nullptr)
{result_DOUBLE= 9999.99;}
// ?? //    else {result_DOUBLE = m_chassis->GetCurrentPosition();}
// ?? //}

//
// for the chassis motors
//
// Left Front swerve module
void RamScan::Chassis_LF_Drive_Rotations(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::LEFT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Chassis_LF_Drive_RPS(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::LEFT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Chassis_LF_Drive_Current(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::LEFT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

void RamScan::Chassis_LF_Turn_Rotations(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::LEFT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Chassis_LF_Turn_RPS(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::LEFT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Chassis_LF_Turn_Current(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::LEFT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

// Right Front swerve module
void RamScan::Chassis_RF_Drive_Rotations(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::RIGHT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Chassis_RF_Drive_RPS(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::RIGHT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Chassis_RF_Drive_Current(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::RIGHT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

void RamScan::Chassis_RF_Turn_Rotations(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::RIGHT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Chassis_RF_Turn_RPS(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::RIGHT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Chassis_RF_Turn_Current(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::RIGHT_FRONT];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

// Left Back swerve module
void RamScan::Chassis_LB_Drive_Rotations(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::LEFT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Chassis_LB_Drive_RPS(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::LEFT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Chassis_LB_Drive_Current(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::LEFT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

void RamScan::Chassis_LB_Turn_Rotations(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::LEFT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Chassis_LB_Turn_RPS(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::LEFT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Chassis_LB_Turn_Current(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::LEFT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

// Right Back swerve module
void RamScan::Chassis_RB_Drive_Rotations(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::RIGHT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Chassis_RB_Drive_RPS(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::RIGHT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Chassis_RB_Drive_Current(void) {
    auto controller = SwerveDriveMotorArray[SwerveModule::ModuleID::RIGHT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

void RamScan::Chassis_RB_Turn_Rotations(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::RIGHT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Chassis_RB_Turn_RPS(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::RIGHT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Chassis_RB_Turn_Current(void) {
    auto controller = SwerveTurnMotorArray[SwerveModule::ModuleID::RIGHT_BACK];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

//
//  For the mechanism motors...
//
void RamScan::Motor_INTAKE1_GetRotations(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE1];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Motor_INTAKE1_GetRPS(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE1];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Motor_INTAKE1_GetCurrent(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE1];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

void RamScan::Motor_INTAKE2_GetRotations(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE2];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Motor_INTAKE2_GetRPS(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE2];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Motor_INTAKE2_GetCurrent(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE2];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

void RamScan::Motor_BALL_TRANSFER_GetRotations(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::BALL_TRANSFER];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Motor_BALL_TRANSFER_GetRPS(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::BALL_TRANSFER];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Motor_BALL_TRANSFER_GetCurrent(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::BALL_TRANSFER];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

void RamScan::Motor_TURRET_GetRotations(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::TURRET];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Motor_TURRET_GetRPS(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::TURRET];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Motor_TURRET_GetCurrent(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::TURRET];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

void RamScan::Motor_SHOOTER_1_GetRotations(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER_1];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Motor_SHOOTER_1_GetRPS(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER_1];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Motor_SHOOTER_1_GetCurrent(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER_1];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

void RamScan::Motor_SHOOTER_2_GetRotations(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER_2];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Motor_SHOOTER_2_GetRPS(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER_2];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Motor_SHOOTER_2_GetCurrent(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER_2];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

void RamScan::Motor_SHOOTER_HOOD_GetRotations(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER_HOOD];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRotations();} }

void RamScan::Motor_SHOOTER_HOOD_GetRPS(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER_HOOD];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetRPS();} }

void RamScan::Motor_SHOOTER_HOOD_GetCurrent(void) {
    shared_ptr<IDragonMotorController> controller =
        MotorControllerArray[MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER_HOOD];
    if (controller == nullptr) {result_DOUBLE= 9999.99;}
    else {result_DOUBLE = controller->GetCurrent();} }

//
//  For Teleop Axis inputs
//
void RamScan::TeleopAxis_SWERVE_DRIVE_DRIVE(void)   {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SWERVE_DRIVE_DRIVE);} }

void RamScan::TeleopAxis_SWERVE_DRIVE_ROTATE(void)  {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SWERVE_DRIVE_ROTATE);} }

void RamScan::TeleopAxis_SWERVE_DRIVE_STEER(void)   {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SWERVE_DRIVE_STEER);} }

void RamScan::TeleopAxis_SWITCH_DRIVE_MODE(void)    {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SWITCH_DRIVE_MODE);} }

void RamScan::TeleopAxis_CURVATURE_DRIVE_QUICK_TURN(void) {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::CURVATURE_DRIVE_QUICK_TURN);} }

void RamScan::TeleopAxis_IMPELLER_OFF(void)         {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::IMPELLER_OFF);} }

void RamScan::TeleopAxis_IMPELLER_HOLD(void)
    {if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::IMPELLER_HOLD);} }

void RamScan::TeleopAxis_IMPELLER_TO_SHOOTER(void)  {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::IMPELLER_TO_SHOOTER);} }

void RamScan::TeleopAxis_INTAKE_ON(void)            {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::INTAKE_ON);} }

void RamScan::TeleopAxis_INTAKE_OFF(void)           {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::INTAKE_OFF);} }

void RamScan::TeleopAxis_BALL_TRANSFER_OFF(void)    {
    if (m_teleopControl==nullptr) {result_FLOAT= 9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::BALL_TRANSFER_OFF);} }

void RamScan::TeleopAxis_BALL_TRANSFER_TO_IMPELLER(void) {
    if (m_teleopControl==nullptr) {result_FLOAT= 9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::BALL_TRANSFER_TO_IMPELLER);} }

void RamScan::TeleopAxis_BALL_TRANSFER_TO_SHOOTER(void)
    {if (m_teleopControl==nullptr) {result_FLOAT= 9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::BALL_TRANSFER_TO_SHOOTER);} }

void RamScan::TeleopAxis_SHOOTER_PREPARE_TO_SHOOT(void){
    if (m_teleopControl==nullptr) {result_FLOAT= 9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SHOOTER_PREPARE_TO_SHOOT);} }

void RamScan::TeleopAxis_SHOOTER_AUTO_SHOOT(void)   {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SHOOTER_AUTO_SHOOT);} }

void RamScan::TeleopAxis_SHOOTER_MANUAL_AIM(void)   {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SHOOTER_MANUAL_AIM);} }

void RamScan::TeleopAxis_SHOOTER_MANUAL_ADJUST_DISTANCE(void) {
    if (m_teleopControl==nullptr) {result_FLOAT= 9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SHOOTER_MANUAL_ADJUST_DISTANCE);} }

void RamScan::TeleopAxis_SHOOTER_MANUAL_SHOOT(void) {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SHOOTER_MANUAL_SHOOT);} }

void RamScan::TeleopAxis_SHOOTER_OFF(void)          {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SHOOTER_OFF);} }

void RamScan::TeleopAxis_SHOOTER_HOOD_MOVE_UP(void) {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SHOOTER_HOOD_MOVE_UP);} }

void RamScan::TeleopAxis_SHOOTER_HOOD_MOVE_DOWN(void) {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SHOOTER_HOOD_MOVE_DOWN);} }

void RamScan::TeleopAxis_SHOOTER_HOOD_HOLD_POSITION(void) {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SHOOTER_HOOD_HOLD_POSITION);} }

void RamScan::TeleopAxis_SHOOTER_HOOD_MANUAL_BUTTON(void) {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SHOOTER_HOOD_MANUAL_BUTTON);} }

void RamScan::TeleopAxis_SHOOTER_HOOD_MANUAL_AXIS(void) {
if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::SHOOTER_HOOD_MANUAL_AXIS);} }

void RamScan::TeleopAxis_TURRET_MANUAL_AXIS(void)     {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::TURRET_MANUAL_AXIS);} }

void RamScan::TeleopAxis_TURRET_MANUAL_BUTTON(void)   {
    if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::TURRET_MANUAL_BUTTON);} }

void RamScan::TeleopAxis_TURRET_LIMELIGHT_AIM(void) {
if (m_teleopControl==nullptr) {result_FLOAT=9999.99;}
    else {result_FLOAT = m_teleopControl->GetAxisValue(TeleopControl::TURRET_LIMELIGHT_AIM);} }

