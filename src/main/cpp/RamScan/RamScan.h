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

#include <frc/Preferences.h>
#include <string>

using namespace std;

// include files for data to scan
#include <gamepad/TeleopControl.h>
#include <hw/DragonPigeon.h>
#include <hw/DragonTalon.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/MotorControllerUsage.h>
#include <subsys/SwerveChassis.h>


class RamScan {

    friend class    Robot;

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
        // //STRING
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
    static string           result_STRING;

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

    typedef enum            // TODO: this should be in the Robot class, but how to access it then?
    {
        INIT,
        DISABLED,
        AUTON,
        TELEOP,
        TEST
    } RunModeType;
    static RunModeType      m_RunMode;      // Mode set in Robot.cpp

// pointers to objects containing variables that may be scanned
    static std::shared_ptr<SwerveChassis>   m_chassis;
    static DragonPigeon*                    m_pigeon;
    static TeleopControl*                   m_teleopControl;

//  Arrays of swerve motor controllers, indexed by SwerveModule::ModuleID
    static shared_ptr<IDragonMotorController> SwerveDriveMotorArray[4];
    static shared_ptr<IDragonMotorController> SwerveTurnMotorArray[4];

//  Array of mechanism motor controllers, indexed by MOTOR_CONTROLLER_USAGE
    static shared_ptr<IDragonMotorController>
        MotorControllerArray[static_cast<int>(MotorControllerUsage::MAX_MOTOR_CONTROLLER_USAGES)];

// pointers to motor controllers (nullptr if that motor doesn't exist)
    static shared_ptr<IDragonMotorController>  m_Motor_INTAKE1;
    static shared_ptr<IDragonMotorController>  m_Motor_INTAKE2;
    static shared_ptr<IDragonMotorController>  m_Motor_BALL_TRANSFER;
    static shared_ptr<IDragonMotorController>  m_Motor_TURRET;
    static shared_ptr<IDragonMotorController>  m_Motor_SHOOTER_1;
    static shared_ptr<IDragonMotorController>  m_Motor_SHOOTER_2;
    static shared_ptr<IDragonMotorController>  m_Motor_SHOOTER_HOOD;

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

    static void Chassis_CurrentSpeed(void);

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
    static void Motor_INTAKE1_GetRotations(void);
    static void Motor_INTAKE1_GetRPS(void);
    static void Motor_INTAKE1_GetCurrent(void);
    static void Motor_INTAKE2_GetRotations(void);
    static void Motor_INTAKE2_GetRPS(void);
    static void Motor_INTAKE2_GetCurrent(void);
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
    static void Motor_SHOOTER_HOOD_GetRotations(void);
    static void Motor_SHOOTER_HOOD_GetRPS(void);
    static void Motor_SHOOTER_HOOD_GetCurrent(void);

//  teleop inputs
    static void TeleopAxis_SWERVE_DRIVE_DRIVE(void);
    static void TeleopAxis_SWERVE_DRIVE_ROTATE(void);
    static void TeleopAxis_SWERVE_DRIVE_STEER(void);
    static void TeleopAxis_SWITCH_DRIVE_MODE(void);
    static void TeleopAxis_CURVATURE_DRIVE_QUICK_TURN(void);
// ?? //    static void TeleopAxis_IMPELLER_OFF(void);
// ?? //    static void TeleopAxis_IMPELLER_HOLD(void);
// ?? //    static void TeleopAxis_IMPELLER_TO_SHOOTER(void);
    static void TeleopAxis_INTAKE_ON(void);
    static void TeleopAxis_INTAKE_OFF(void);
    static void TeleopAxis_BALL_TRANSFER_OFF(void);
    static void TeleopAxis_BALL_TRANSFER_TO_SHOOTER(void);
// ?? //    static void TeleopAxis_BALL_TRANSFER_TO_IMPELLER(void);
    static void TeleopAxis_SHOOTER_PREPARE_TO_SHOOT(void);
    static void TeleopAxis_SHOOTER_AUTO_SHOOT(void);
    static void TeleopAxis_SHOOTER_MANUAL_AIM(void);
    static void TeleopAxis_SHOOTER_MANUAL_ADJUST_DISTANCE(void);
    static void TeleopAxis_SHOOTER_MANUAL_SHOOT(void);
    static void TeleopAxis_SHOOTER_OFF(void);
    static void TeleopAxis_SHOOTER_HOOD_MOVE_UP(void);
    static void TeleopAxis_SHOOTER_HOOD_MOVE_DOWN(void);
    static void TeleopAxis_SHOOTER_HOOD_HOLD_POSITION(void);
    static void TeleopAxis_SHOOTER_HOOD_MANUAL_BUTTON(void);
    static void TeleopAxis_SHOOTER_HOOD_MANUAL_AXIS(void);
    static void TeleopAxis_TURRET_MANUAL_AXIS(void);
    static void TeleopAxis_TURRET_MANUAL_BUTTON(void);
    static void TeleopAxis_TURRET_LIMELIGHT_AIM(void);


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
    {(char*)("B01 PacketCntr20ms:"),        INT,    (&Get_PacketCntr20ms)},                 // 0.0002
    {(char*)("B02 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0004
    {(char*)("B03 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0008

    {(char*)("B04 PigeonPitch:"),           DOUBLE, (&Get_Pigeon_Pitch)},                   // 0.0010
    {(char*)("B05 PigeonRoll:"),            DOUBLE, (&Get_Pigeon_Roll)},                    // 0.0020
    {(char*)("B06 PigeonYaw:"),             DOUBLE, (&Get_Pigeon_Yaw)},                     // 0.0040
    {(char*)("B07 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0080

    {(char*)("B08 INTAKE1_Rotations"),      DOUBLE, (&Motor_INTAKE1_GetRotations)},         // 0.0100
    {(char*)("B09 INTAKE1_RPS"),            DOUBLE, (&Motor_INTAKE1_GetRPS)},               // 0.0200
    {(char*)("B10 INTAKE1_Current"),        DOUBLE, (&Motor_INTAKE1_GetCurrent)},           // 0.0400
    {(char*)("B11 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0800

    {(char*)("B08 INTAKE2_Rotations"),      DOUBLE, (&Motor_INTAKE2_GetRotations)},         // 0.1000
    {(char*)("B13 INTAKE2_RPS"),            DOUBLE, (&Motor_INTAKE2_GetRPS)},               // 0.2000
    {(char*)("B14 INTAKE2_Current"),        DOUBLE, (&Motor_INTAKE2_GetCurrent)},           // 0.4000
    {(char*)("B15 dummy:"),                 INT,    (&CrazyEights)},                        // 0.8000

    {(char*)("B08 BALL_TRANSFER_Rotations"),DOUBLE, (&Motor_BALL_TRANSFER_GetRotations)},   // 0001.0
    {(char*)("B17 BALL_TRANSFER_RPS"),      DOUBLE, (&Motor_BALL_TRANSFER_GetRPS)},         // 0002.0
    {(char*)("B18 BALL_TRANSFER_Current"),  DOUBLE, (&Motor_BALL_TRANSFER_GetCurrent)},     // 0004.0
    {(char*)("B19 dummy:"),                 INT,    (&CrazyEights)},                        // 0008.0

    {(char*)("B08 TURRET_Rotations"),       DOUBLE, (&Motor_TURRET_GetRotations)},          // 0010.0
    {(char*)("B21 TURRET_RPS"),             DOUBLE, (&Motor_TURRET_GetRPS)},                // 0020.0
    {(char*)("B22 TURRET_Current"),         DOUBLE, (&Motor_TURRET_GetCurrent)},            // 0040.0
    {(char*)("B23 dummy:"),                 INT,    (&CrazyEights)},                        // 0080.0

    {(char*)("B24 SHOOTER_1_RPS"),          DOUBLE, (&Motor_SHOOTER_1_GetRPS)},             // 0100.0
    {(char*)("B25 SHOOTER_1_Current"),      DOUBLE, (&Motor_SHOOTER_1_GetCurrent)},         // 0200.0
    {(char*)("B26 SHOOTER_2_RPS"),          DOUBLE, (&Motor_SHOOTER_2_GetRPS)},             // 0400.0
    {(char*)("B27 SHOOTER_2_Current"),      DOUBLE, (&Motor_SHOOTER_2_GetCurrent)},         // 0800.0

    {(char*)("B08 SHOOTER_HOOD_Rotations"), DOUBLE, (&Motor_SHOOTER_HOOD_GetRotations)},    // 1000.0
    {(char*)("B25 SHOOTER_HOOD_RPS"),       DOUBLE, (&Motor_SHOOTER_HOOD_GetRPS)},          // 2000.0
    {(char*)("B26 SHOOTER_HOOD_Current"),   DOUBLE, (&Motor_SHOOTER_HOOD_GetCurrent)},      // 4000.0
    {(char*)("B27 dummy:"),                 INT,    (&CrazyEights)},                        // 8000.0

    {(char*)("THE END"), INT, nullptr }    // Great move, Columbus, you just fell off the edge of the world!
  };

  // not used yet - wait for multiple 32-bit arrays
  const ScanElement ScanElementArray2[NUM_OF_SCAN_ELEMENTS+1] =       // Teleop inputs
  {
    {(char*)("B00 SWERVE_DRIVE_DRIVE"),     FLOAT,  (&TeleopAxis_SWERVE_DRIVE_DRIVE)},      // 0.0001
    {(char*)("B01 SWERVE_DRIVE_ROTATE"),    FLOAT,  (&TeleopAxis_SWERVE_DRIVE_ROTATE)},     // 0.0002
    {(char*)("B02 SWERVE_DRIVE_STEER"),     FLOAT,  (&TeleopAxis_SWERVE_DRIVE_STEER)},      // 0.0004
    {(char*)("B03 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0008
    {(char*)("B04 SWITCH_DRIVE_MODE"),      FLOAT,  (&TeleopAxis_SWITCH_DRIVE_MODE)},       // 0.0010
    {(char*)("B05 CURV_DRIVE_QUICK_TURN"),  FLOAT,  (&TeleopAxis_CURVATURE_DRIVE_QUICK_TURN)},//0.0020
    {(char*)("B06 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0040
//    {(char*)("B07 IMPELLER_OFF"),           FLOAT,  (&TeleopAxis_IMPELLER_OFF)},            // 0.0080
    {(char*)("B07 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0080

//    {(char*)("B08 IMPELLER_HOLD"),          FLOAT,  (&TeleopAxis_IMPELLER_HOLD)},           // 0.0100
    {(char*)("B08 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0100
//    {(char*)("B09 IMPELLER_TO_SHOOTER"),    FLOAT,  (&TeleopAxis_IMPELLER_TO_SHOOTER)},     // 0.0200
    {(char*)("B09 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0200
    {(char*)("B10 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0400
    {(char*)("B11 INTAKE_ON"),              FLOAT,  (&TeleopAxis_INTAKE_ON)},               // 0.0800
    {(char*)("B12 INTAKE_OFF"),             FLOAT,  (&TeleopAxis_INTAKE_OFF)},              // 0.1000
    {(char*)("B13 BALL_OFF"),               FLOAT,  (&TeleopAxis_BALL_TRANSFER_OFF)},       // 0.2000
//    {(char*)("B14 BALL_TRANSFER_IMPELLER"), FLOAT,  (&TeleopAxis_BALL_TRANSFER_TO_IMPELLER)},//0.4000
    {(char*)("B14 dummy:"),                 INT,    (&CrazyEights)},                        // 0.4000
    {(char*)("B15 BALL_TRANSFER_SHOOTER"),  FLOAT,  (&TeleopAxis_BALL_TRANSFER_TO_SHOOTER)}, //0.8000

    {(char*)("B16 dummy:"),                 INT,    (&CrazyEights)},                        // 0001.0
    {(char*)("B17 SHOOTER_PREPARE_SHOOT"),  FLOAT,  (&TeleopAxis_SHOOTER_PREPARE_TO_SHOOT)},// 0002.0
    {(char*)("B18 SHOOTER_AUTO_SHOOT"),     FLOAT,  (&TeleopAxis_SHOOTER_AUTO_SHOOT)},      // 0004.0
    {(char*)("B19 SHOOTER_MANUAL_AIM"),     FLOAT,  (&TeleopAxis_SHOOTER_MANUAL_AIM)},      // 0008.0
    {(char*)("B20 SHOOTER_MANUAL_ADJ_DIST"),FLOAT,  (&TeleopAxis_SHOOTER_MANUAL_ADJUST_DISTANCE)},//0010.0
    {(char*)("B21 SHOOTER_MANUAL_SHOOT"),   FLOAT,  (&TeleopAxis_SHOOTER_MANUAL_SHOOT)},    // 0020.0
    {(char*)("B22 SHOOTER_OFF"),            FLOAT,  (&TeleopAxis_SHOOTER_OFF)},             // 0040.0
    {(char*)("B23 dummy:"),                 INT,    (&CrazyEights)},                        // 0080.0

    {(char*)("B24 SHOOTER_HOOD_MOVE_UP"),   FLOAT,  (&TeleopAxis_SHOOTER_HOOD_MOVE_UP)},    // 0100.0
    {(char*)("B25 SHOOTER_HOOD_MOVE_DOWN"), FLOAT,  (&TeleopAxis_SHOOTER_HOOD_MOVE_DOWN)},  // 0200.0
    {(char*)("B26 SHOOTER_HOOD_HOLD_POS"),  FLOAT,  (&TeleopAxis_SHOOTER_HOOD_HOLD_POSITION)},//0400.0
    {(char*)("B27 SHOOTER_HOOD_MANUAL_BUT"),FLOAT,  (&TeleopAxis_SHOOTER_HOOD_MANUAL_BUTTON)},//0800.0
    {(char*)("B28 SHOOTER_HOOD_MANUAL_AXS"),FLOAT,  (&TeleopAxis_SHOOTER_HOOD_MANUAL_AXIS)},// 1000.0
    {(char*)("B29 TURRET_MANUAL_AXIS"),     FLOAT,  (&TeleopAxis_TURRET_MANUAL_AXIS)},      // 2000.0
    {(char*)("B30 TURRET_MANUAL_BUTTON"),   FLOAT,  (&TeleopAxis_TURRET_MANUAL_BUTTON)},    // 4000.0
    {(char*)("B31 TURRET_LIMELIGHT_AIM"),   FLOAT,  (&TeleopAxis_TURRET_LIMELIGHT_AIM)},    // 8000.0
    {(char*)("ANOTHER END"), INT, nullptr }
  };

  // not used yet - wait for multiple 32-bit arrays
  const ScanElement ScanElementArray3[NUM_OF_SCAN_ELEMENTS+1] =       // Swerve motor variables
  {
  // Front Left module
    {(char*)("B00 CHASS_LF_DRV_ROTATIONS"), DOUBLE, (&Chassis_LF_Drive_Rotations)},         // 0.0001
    {(char*)("B01 CHASS_LF_DRV_RPS"),       DOUBLE, (&Chassis_LF_Drive_RPS), },             // 0.0002
    {(char*)("B02 CHASS_LF_DRV_CURRENT"),   DOUBLE, (&Chassis_LF_Drive_Current)},           // 0.0004
    {(char*)("B03 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0008
    {(char*)("B04 CHASS_LF_TURN_ROTATIONS"),DOUBLE, (&Chassis_LF_Turn_Rotations)},          // 0.0010
    {(char*)("B05 CHASS_LF_TURN_RPS"),      DOUBLE, (&Chassis_LF_Turn_RPS), },              // 0.0020
    {(char*)("B06 CHASS_LF_TURN_CURRENT"),  DOUBLE, (&Chassis_LF_Turn_Current)},            // 0.0040
    {(char*)("B07 dummy:"),                 INT,    (&CrazyEights)},                        // 0.0080
  // Front Right module
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

