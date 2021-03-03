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

// RamScan.h

#pragma once

#include <frc/DriverStation.h>
#include <frc/Preferences.h>

// include files for data to scan
#include <gamepad/TeleopControl.h>
#include <hw/DragonPigeon.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/MotorControllerUsage.h>
#include <subsys/SwerveChassis.h>

using namespace frc;
using namespace std;


class RamScan {

public:
    RamScan();
    virtual ~RamScan();

    void Init();
    void ScanVariables();

    static void SaveMechanismMotorUsage(shared_ptr<IDragonMotorController>              motor,
                                        MotorControllerUsage::MOTOR_CONTROLLER_USAGE    usage);

    static void SaveSwerveMotorUsage(shared_ptr<IDragonMotorController>     driveMotor,
                                    shared_ptr<IDragonMotorController>      turnMotor,
                                    SwerveModule::ModuleID                  position);


private:

// definition of type for each scanned variable
    typedef enum
    {
        BOOL,
        CHAR,
        INT,
        LONG,
        UCHAR,
        UINT,
        ULONG,
        FLOAT,
        DOUBLE,
    } ScanElementType;

// result variables for each type
    static bool             result_BOOL;
    static char             result_CHAR;
    static int              result_INT;
    static long             result_LONG;
    static unsigned char    result_UCHAR;
    static unsigned int     result_UINT;
    static unsigned long    result_ULONG;
    static float            result_FLOAT;
    static double           result_DOUBLE;

// structure to define elements of ScanElementArray(s)
    typedef struct
    {
        char*               NamePtr;            // this will be displayed on dashboard
        ScanElementType     Type;               // type determines display format
        void                (* AccAdd)(void);   // address of accessor function
    } ScanElement;

// other variables
    frc::Preferences        *m_prefs;           // Gets user input of bitmask
    unsigned long int       m_bitMask;          // 32 bits
    unsigned long int       m_bitMaskPrev;      //  "

    static int              m_PacketCntr20ms;   // count comm. packets from driver station (every 20ms)

// pointers to objects containing variables that may be scanned
    static std::shared_ptr<SwerveChassis>   m_chassis;
    static DriverStation*                   m_DriverStation;
    static DragonPigeon*                    m_pigeon;
    static TeleopControl*                   m_teleopControl;

//  Arrays of swerve motor controllers, indexed by SwerveModule::ModuleID
    static shared_ptr<IDragonMotorController> SwerveDriveMotorArray[4];
    static shared_ptr<IDragonMotorController> SwerveTurnMotorArray[4];

//  Array of mechanism motor controllers, indexed by MOTOR_CONTROLLER_USAGE
    static shared_ptr<IDragonMotorController>
        MotorControllerArray[static_cast<int>(MotorControllerUsage::MAX_MOTOR_CONTROLLER_USAGES)];

// pointers to motor controllers (nullptr if that motor doesn't exist)
    static shared_ptr<IDragonMotorController>  m_Motor_INTAKE;
    static shared_ptr<IDragonMotorController>  m_Motor_BALL_HOPPER;
    static shared_ptr<IDragonMotorController>  m_Motor_BALL_TRANSFER;
    static shared_ptr<IDragonMotorController>  m_Motor_TURRET;
    static shared_ptr<IDragonMotorController>  m_Motor_SHOOTER_1;
    static shared_ptr<IDragonMotorController>  m_Motor_SHOOTER_2;

//==============================================================================//
//                                                                              //
//  accessors for the ScanElement tables                                        //
//                                                                              //
//==============================================================================//

    static void CrazyEights(void);                  // returns 8888 for dummy functions
    static void Get_RunMode(void);
    static void Get_PacketCntr20ms(void);
//
//  inputs from the pigeon
    static void Get_Pigeon_Pitch(void);
    static void Get_Pigeon_Roll(void);
    static void Get_Pigeon_Yaw(void);

    static void ChassisPoseX(void);
    static void ChassisPoseY(void);
    static void ChassisPoseDeg(void);

    static void ChassisSpeedX(void);
    static void ChassisSpeedY(void);
    static void ChassisSpeedOmega(void);

// for the chassis motors
    // Left Front swerve module
    static void Chassis_LF_Drive_Rotations(void);
    static void Chassis_LF_Drive_RPS(void);
    static void Chassis_LF_Drive_Current(void);
    static void Chassis_LF_Turn_Rotations(void);
    static void Chassis_LF_Turn_RPS(void);
    static void Chassis_LF_Turn_Current(void);
    // Right Front swerve module
    static void Chassis_RF_Drive_Rotations(void);
    static void Chassis_RF_Drive_RPS(void);
    static void Chassis_RF_Drive_Current(void);
    static void Chassis_RF_Turn_Rotations(void);
    static void Chassis_RF_Turn_RPS(void);
    static void Chassis_RF_Turn_Current(void);
    // Left Back swerve module
    static void Chassis_LB_Drive_Rotations(void);
    static void Chassis_LB_Drive_RPS(void);
    static void Chassis_LB_Drive_Current(void);
    static void Chassis_LB_Turn_Rotations(void);
    static void Chassis_LB_Turn_RPS(void);
    static void Chassis_LB_Turn_Current(void);
    //
    // Right Back swerve module
    static void Chassis_RB_Drive_Rotations(void);
    static void Chassis_RB_Drive_RPS(void);
    static void Chassis_RB_Drive_Current(void);
    static void Chassis_RB_Turn_Rotations(void);
    static void Chassis_RB_Turn_RPS(void);
    static void Chassis_RB_Turn_Current(void);

//  for the mechanism motors
    static void Motor_INTAKE_GetRotations(void);
    static void Motor_INTAKE_GetRPS(void);
    static void Motor_INTAKE_GetCurrent(void);
    static void Motor_BALL_HOPPER_GetRotations(void);
    static void Motor_BALL_HOPPER_GetRPS(void);
    static void Motor_BALL_HOPPER_GetCurrent(void);
    static void Motor_BALL_TRANSFER_GetRotations(void);
    static void Motor_BALL_TRANSFER_GetRPS(void);
    static void Motor_BALL_TRANSFER_GetCurrent(void);
    static void Motor_TURRET_GetRotations(void);
    static void Motor_TURRET_GetRPS(void);
    static void Motor_TURRET_GetCurrent(void);
    static void Motor_SHOOTER_1_GetRotations(void);
    static void Motor_SHOOTER_1_GetRPS(void);
    static void Motor_SHOOTER_1_GetCurrent(void);
    static void Motor_SHOOTER_2_GetRotations(void);
    static void Motor_SHOOTER_2_GetRPS(void);
    static void Motor_SHOOTER_2_GetCurrent(void);
//
//  teleop axis inputs
    static void TeleopAxis_SWERVE_DRIVE_DRIVE(void);
    static void TeleopAxis_SWERVE_DRIVE_ROTATE(void);
    static void TeleopAxis_SWERVE_DRIVE_STEER(void);
//
//  teleop button inputs
    static void TeleopButton_INTAKE_ON(void);
    static void TeleopButton_INTAKE_OFF(void);
    static void TeleopButton_BALL_TRANSFER_OFF(void);
    static void TeleopButton_BALL_TRANSFER_TO_SHOOTER(void);
    static void TeleopAxis_SHOOTER_PREPARE_TO_SHOOT(void);
    static void TeleopButton_SHOOTER_MANUAL_SHOOT_GREEN(void);
    static void TeleopButton_SHOOTER_MANUAL_SHOOT_YELLOW(void);
    static void TeleopButton_SHOOTER_MANUAL_SHOOT_BLUE(void);
    static void TeleopButton_SHOOTER_MANUAL_SHOOT_RED(void);
    static void TeleopButton_TURRET_LIMELIGHT_AIM(void);
    static void TeleopButton_REZERO_PIGEON(void);
    static void TeleopButton_OFF(void);
    static void TeleopButton_SHOOT(void);


//==============================================================================//
//                                                                              //
//  ScanElementArrays have a line for each variable that can be displayed       //
//                                                                              //
//==============================================================================//

  #define NUM_OF_SCAN_ELEMENTS    32      // TODO: use multiple 32-bit arrays

  const ScanElement ScanElementArray[NUM_OF_SCAN_ELEMENTS+1] =
  {
//  NamePtr (displayed on dashboard)        Type    AccAdd (address of accessor function)   Hex value of bit
//  --------------------------------        ----    -------------------------------------   ----------------
    {(char*)("B00 RunMode:"),               INT,    (&Get_RunMode)},                        // 0.0001
    {(char*)("B01 PigeonPitch:"),           DOUBLE, (&Get_Pigeon_Pitch)},                   // 0.0001
    {(char*)("B02 PigeonRoll:"),            DOUBLE, (&Get_Pigeon_Roll)},                    // 0.0002
    {(char*)("B03 PigeonYaw:"),             DOUBLE, (&Get_Pigeon_Yaw)},                     // 0.0004

    {(char*)("B04 CHASSIS_POSE_X"),         DOUBLE, (&ChassisPoseX)},                       // 0.0010
    {(char*)("B05 CHASSIS_POSE_Y"),         DOUBLE, (&ChassisPoseY)},                       // 0.0020
    {(char*)("B06 CHASSIS_POSE_DEG"),       DOUBLE, (&ChassisPoseDeg)},                     // 0.0040
    {(char*)("B07 PacketCntr20ms:"),        INT,    (&Get_PacketCntr20ms)},                 // 0.0080


    {(char*)("B08 CHASSIS_SPEED_X"),        DOUBLE, (&ChassisSpeedX)},                      // 0.0100
    {(char*)("B09 CHASSIS_SPEED_Y"),        DOUBLE, (&ChassisSpeedY)},                      // 0.0200
    {(char*)("B10 CHASSIS_SPEED_DEG"),      DOUBLE, (&ChassisSpeedOmega)},                  // 0.0400
    {(char*)("B11 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0800

    {(char*)("B12 INTAKE_Rotations"),       DOUBLE, (&Motor_INTAKE_GetRotations)},          // 0.1000
    {(char*)("B13 INTAKE_RPS"),             DOUBLE, (&Motor_INTAKE_GetRPS)},                // 0.2000
    {(char*)("B14 INTAKE_Current"),         DOUBLE, (&Motor_INTAKE_GetCurrent)},            // 0.4000
    {(char*)("B15 dummy:"),                 INT,    (&CrazyEights)},                        // 0.8000

    {(char*)("B16 BALL_HOPPER_Rotations"),  DOUBLE, (&Motor_BALL_HOPPER_GetRotations)},     // 0001.0
    {(char*)("B17 BALL_HOPPER_RPS"),        DOUBLE, (&Motor_BALL_HOPPER_GetRPS)},           // 0002.0
    {(char*)("B18 BALL_HOPPER_Current"),    DOUBLE, (&Motor_BALL_HOPPER_GetCurrent)},       // 0004.0
    {(char*)("B19 dummy:"),                 INT,    (&CrazyEights)},                        // 0008.0

    {(char*)("B20 BALL_TRANSFER_Rotations"),DOUBLE, (&Motor_BALL_TRANSFER_GetRotations)},   // 0010.0
    {(char*)("B21 BALL_TRANSFER_RPS"),      DOUBLE, (&Motor_BALL_TRANSFER_GetRPS)},         // 0020.0
    {(char*)("B22 BALL_TRANSFER_Current"),  DOUBLE, (&Motor_BALL_TRANSFER_GetCurrent)},     // 0040.0
    {(char*)("B23 dummy:"),                 INT,    (&CrazyEights)},                        // 0080.0

    {(char*)("B24 TURRET_Rotations"),       DOUBLE, (&Motor_TURRET_GetRotations)},          // 0100.0
    {(char*)("B25 TURRET_RPS"),             DOUBLE, (&Motor_TURRET_GetRPS)},                // 0200.0
    {(char*)("B26 TURRET_Current"),         DOUBLE, (&Motor_TURRET_GetCurrent)},            // 0400.0
    {(char*)("B27 dummy:"),                 INT,    (&CrazyEights)},                        // 0800.0

    {(char*)("B28 SHOOTER_1_RPS"),          DOUBLE, (&Motor_SHOOTER_1_GetRPS)},             // 1000.0
    {(char*)("B29 SHOOTER_1_Current"),      DOUBLE, (&Motor_SHOOTER_1_GetCurrent)},         // 2000.0
    {(char*)("B30 SHOOTER_2_RPS"),          DOUBLE, (&Motor_SHOOTER_2_GetRPS)},             // 4000.0
    {(char*)("B31 SHOOTER_2_Current"),      DOUBLE, (&Motor_SHOOTER_2_GetCurrent)},         // 8000.0

    {(char*)("THE END"), INT, nullptr }    // Great move, Columbus, you just fell off the edge of the world!
  };

  // not used yet - wait for multiple 32-bit arrays
  const ScanElement ScanElementArray2[NUM_OF_SCAN_ELEMENTS+1] =       // Driver Station inputs
  {
    {(char*)("B00 SWERVE_DRIVE_DRIVE"),     FLOAT,  (&TeleopAxis_SWERVE_DRIVE_DRIVE)},      // 0.0001
    {(char*)("B01 SWERVE_DRIVE_ROTATE"),    FLOAT,  (&TeleopAxis_SWERVE_DRIVE_ROTATE)},     // 0.0002
    {(char*)("B02 SWERVE_DRIVE_STEER"),     FLOAT,  (&TeleopAxis_SWERVE_DRIVE_STEER)},      // 0.0004
    {(char*)("B03 INTAKE_ON"),              BOOL,   (&TeleopButton_INTAKE_ON)},             // 0.0008

    {(char*)("B04 INTAKE_OFF"),             BOOL,   (&TeleopButton_INTAKE_OFF)},            // 0.0010
    {(char*)("B05 BALL_TRANSFER_OFF"),      BOOL,   (&TeleopButton_BALL_TRANSFER_OFF)},     // 0.0020
    {(char*)("B06 BALL_TRANSFER_SHOOTER"),  BOOL,   (&TeleopButton_BALL_TRANSFER_TO_SHOOTER)},//0.0040
    {(char*)("B07 SHOOTER_PREPARE_SHOOT"),  FLOAT,  (&TeleopAxis_SHOOTER_PREPARE_TO_SHOOT)}, //0.0080

    {(char*)("B08 SHOOTER_MN_SHOOT_GREEN"), BOOL,   (&TeleopButton_SHOOTER_MANUAL_SHOOT_GREEN)}, // 0.0100
    {(char*)("B09 SHOOTER_MN_SHOOT_YELLOW"),BOOL,   (&TeleopButton_SHOOTER_MANUAL_SHOOT_YELLOW)},// 0.0200
    {(char*)("B10 SHOOTER_MN_SHOOT_BLUE"),  BOOL,   (&TeleopButton_SHOOTER_MANUAL_SHOOT_BLUE)},// 0.0400
    {(char*)("B11 SHOOTER_MN_SHOOT_RED"),   BOOL,   (&TeleopButton_SHOOTER_MANUAL_SHOOT_RED)},// 0.0800

    {(char*)("B12 TURRET_LIMELIGHT_AIM"),   BOOL,   (&TeleopButton_TURRET_LIMELIGHT_AIM)},  // 0.1000
    {(char*)("B13 REZERO_PIGEON"),          BOOL,   (&TeleopButton_REZERO_PIGEON)},         // 0.2000
    {(char*)("B14 OFF"),                    BOOL,   (&TeleopButton_OFF)},                   // 0.4000
    {(char*)("B15 dummy:"),                 INT,    (&CrazyEights)},                        // 0.8000

    {(char*)("B16 dummy:"),                 INT,    (&CrazyEights)},                        // 0001.0
    {(char*)("B17 dummy:"),                 INT,    (&CrazyEights)},                        // 0002.0
    {(char*)("B18 dummy:"),                 INT,    (&CrazyEights)},                        // 0004.0
    {(char*)("B19 dummy:"),                 INT,    (&CrazyEights)},                        // 0008.0

    {(char*)("B20 dummy:"),                 INT,    (&CrazyEights)},                        // 0010.0
    {(char*)("B21 dummy:"),                 INT,    (&CrazyEights)},                        // 0020.0
    {(char*)("B22 dummy:"),                 INT,    (&CrazyEights)},                        // 0040.0
    {(char*)("B23 dummy:"),                 INT,    (&CrazyEights)},                        // 0080.0

    {(char*)("B24 dummy:"),                 INT,    (&CrazyEights)},                        // 0100.0
    {(char*)("B25 dummy:"),                 INT,    (&CrazyEights)},                        // 0200.0
    {(char*)("B26 dummy:"),                 INT,    (&CrazyEights)},                        // 0400.0
    {(char*)("B27 dummy:"),                 INT,    (&CrazyEights)},                        // 0800.0

    {(char*)("B28 dummy:"),                 INT,    (&CrazyEights)},                        // 1000.0
    {(char*)("B29 dummy:"),                 INT,    (&CrazyEights)},                        // 2000.0
    {(char*)("B30 dummy:"),                 INT,    (&CrazyEights)},                        // 4000.0
    {(char*)("B31 dummy:"),                 INT,    (&CrazyEights)},                        // 8000.0

    {(char*)("ANOTHER END"), INT, nullptr }
  };

  // not used yet - wait for multiple 32-bit arrays
  const ScanElement ScanElementArray3[NUM_OF_SCAN_ELEMENTS+1] =       // Swerve motor variables
  {
  // Left Front module
    {(char*)("B00 CHASS_LF_DRV_ROTATIONS"), DOUBLE, (&Chassis_LF_Drive_Rotations)},         // 0.0001
    {(char*)("B01 CHASS_LF_DRV_RPS"),       DOUBLE, (&Chassis_LF_Drive_RPS), },             // 0.0002
    {(char*)("B02 CHASS_LF_DRV_CURRENT"),   DOUBLE, (&Chassis_LF_Drive_Current)},           // 0.0004
    {(char*)("B03 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0008
    {(char*)("B04 CHASS_LF_TURN_ROTATIONS"),DOUBLE, (&Chassis_LF_Turn_Rotations)},          // 0.0010
    {(char*)("B05 CHASS_LF_TURN_RPS"),      DOUBLE, (&Chassis_LF_Turn_RPS), },              // 0.0020
    {(char*)("B06 CHASS_LF_TURN_CURRENT"),  DOUBLE, (&Chassis_LF_Turn_Current)},            // 0.0040
    {(char*)("B07 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0080
  // Right Front module
    {(char*)("B08 CHASS_RF_DRV_ROTATIONS"), DOUBLE, (&Chassis_RF_Drive_Rotations)},         // 0.0100
    {(char*)("B09 CHASS_RF_DRV_RPS"),       DOUBLE, (&Chassis_RF_Drive_RPS), },             // 0.0200
    {(char*)("B11 CHASS_RF_DRV_CURRENT"),   DOUBLE, (&Chassis_LB_Drive_Current)},           // 0.0400
    {(char*)("B11 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0800
    {(char*)("B12 CHASS_RF_TURN_ROTATIONS"),DOUBLE, (&Chassis_RF_Turn_Rotations)},          // 0.1000
    {(char*)("B13 CHASS_RF_TURN_RPS"),      DOUBLE, (&Chassis_RF_Turn_RPS), },              // 0.2000
    {(char*)("B14 CHASS_RF_TURN_CURRENT"),  DOUBLE, (&Chassis_RF_Turn_Current)},            // 0.4000
    {(char*)("B15 dummy:"),                 INT,    (&CrazyEights)},                        // 0.8000
  // Back Left module
    {(char*)("B16 CHASS_LB_DRV_ROTATIONS"), DOUBLE, (&Chassis_LB_Drive_Rotations)},         // 0001.0
    {(char*)("B17 CHASS_LB_DRV_RPS"),       DOUBLE, (&Chassis_LB_Drive_RPS)},               // 0002.0
    {(char*)("B18 CHASS_LB_DRV_CURRENT"),   DOUBLE, (&Chassis_LB_Drive_Current)},           // 0004.0
    {(char*)("B19 dummy:"),                 INT,    (&CrazyEights)},                        // 0008.0
    {(char*)("B20 CHASS_LB_TURN_ROTATIONS"),DOUBLE, (&Chassis_LB_Turn_Rotations)},          // 0010.0
    {(char*)("B21 CHASS_LB_TURN_RPS"),      DOUBLE, (&Chassis_LB_Turn_RPS)},                // 0020.0
    {(char*)("B22 CHASS_LB_TURN_CURRENT"),  DOUBLE, (&Chassis_LB_Turn_Current)},            // 0040.0
    {(char*)("B23 dummy:"),                 INT,    (&CrazyEights)},                        // 0080.0
  // BackRight module
    {(char*)("B24 CHASS_RB_DRV_ROTATIONS"), DOUBLE, (&Chassis_RB_Drive_Rotations)},         // 0100.0
    {(char*)("B25 CHASS_RB_DRV_RPS"),       DOUBLE, (&Chassis_RB_Drive_RPS)},               // 0200.0
    {(char*)("B26 CHASS_RB_DRV_CURRENT"),   DOUBLE, (&Chassis_RB_Drive_Current)},           // 0400.0
    {(char*)("B27 dummy:"),                 INT,    (&CrazyEights)},                        // 0800.0
    {(char*)("B28 CHASS_RB_TURN_ROTATIONS"),DOUBLE, (&Chassis_RB_Turn_Rotations)},          // 1000.0
    {(char*)("B29 CHASS_RB_TURN_RPS"),      DOUBLE, (&Chassis_RB_Turn_RPS)},                // 2000.0
    {(char*)("B30 CHASS_RB_TURN_CURRENT"),  DOUBLE, (&Chassis_RB_Turn_Current)},            // 4000.0
    {(char*)("B31 dummy:"),                 INT,    (&CrazyEights)},                        // 8000.0
    {(char*)("This is the end - The Doors"),INT,    nullptr }
  };

};

