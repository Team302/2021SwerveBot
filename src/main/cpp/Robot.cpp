
//====================================================================================================================================================
// Copyright 2021 Lake Orion Robotics FIRST Team 302
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

// C++ Includes
#include <memory>

// FRC includes


// Team 302 Includes
#include <RamScan/RamScan.h>
#include <Robot.h>
#include <states/chassis/SwerveDrive.h>
#include <subsys/SwerveChassisFactory.h>
#include <subsys/SwerveChassis.h>
#include <xmlhw/RobotDefn.h>


using namespace std;
using namespace frc;

/// @brief  The main robot code.  The Init methods get called when that state gets entered and then the
///     Periodic methods get called every 20 milliseconds.

/// @brief When the robot gets created this gets called.  It initializes the robot subsystems (hardware).
/// @return void
void Robot::RobotInit()
{
    m_RunMode = INIT;

    // Read the robot definition from the xml configuration files and
    // create the hardware (chassis + mechanisms along with their talons,
    // solenoids, digital inputs, analog inputs, etc.
    unique_ptr<RobotDefn>  robotXml = make_unique<RobotDefn>();
    robotXml->ParseXML();

    // RAM SCAN: This is at the end of RobotInit() so all the target objects are already created.
    m_RamScan = new RamScan();
    m_RamScan->Init();
}


/// @brief This function is called every robot packet, no matter the  mode. This is used for items like diagnostics that run
///        during disabled, autonomous, teleoperated and test modes (states).  THis runs after the specific state periodic
///        methods and before the LiveWindow and SmartDashboard updating.
/// @return void
void Robot::RobotPeriodic()
{
    auto swerveChassis = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis();
    if ( swerveChassis.get() != nullptr )
    {
        swerveChassis.get()->UpdateOdometry();
    }

    m_RamScan->ScanVariables();     // do this here after other things are done
}


/// @brief  Called whenever the robot gets disabled (once when it gets disabled).
/// @return void
void Robot::DisabledInit()
{
    m_RunMode = DISABLED;
}


/// @brief Called every 20 milliseconds when the robot is disabled.
/// @return void
void Robot::DisabledPeriodic()
{
    // Its awfully lonely in here. Could someone write some code for me?
}


/// @brief This initializes the autonomous state
/// @return void
void Robot::AutonomousInit()
{
    m_RunMode = AUTON;

    auto swerveChassis = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis();
    if ( swerveChassis.get() != nullptr )
    {
        swerveChassis.get()->ZeroAlignSwerveModules();
    }
    else
    {
        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::ERROR_ONCE, string("AutonomousInit"), string("no swerve chassis"));
    }
}


/// @brief Runs every 20 milliseconds when the autonomous state is active.
/// @return void
void Robot::AutonomousPeriodic()
{
    //Real auton magic right here:
    auto swerveChassis = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis();
    if ( swerveChassis.get() != nullptr )
    {
        swerveChassis.get()->Drive(1.0, 0.0, 0.0, false);
    }
    else
    {
        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::ERROR_ONCE, string("AutonomousPeriodic"), string("no swerve chassis"));
    }
}


/// @brief This initializes the teleoperated state
/// @return void
void Robot::TeleopInit()
{
    m_RunMode = TELEOP;

    auto swerveChassis = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis();
    if ( swerveChassis.get() != nullptr )
    {
        swerveChassis.get()->ZeroAlignSwerveModules();
    }
    else
    {
        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::ERROR_ONCE, string("TeleopPeriodic"), string("no swerve chassis"));
    }

    m_drive = make_shared<SwerveDrive>();
    m_drive.get()->Init();
}


/// @brief Runs every 20 milliseconds when the teleoperated state is active.
/// @return void
void Robot::TeleopPeriodic()
{
    m_drive.get()->Run();
}


/// @brief This initializes the test state
/// @return void
void Robot::TestInit()
{
    m_RunMode = TEST;
}

/// @brief Runs every 20 milliseconds when the test state is active.
/// @return void
void Robot::TestPeriodic()
{
}


/// @brief Returns the current mode of the robot (AUTON, TELEOP, DISABLED etc.)
/// @return int
int    Robot::GetRunMode(void)
{
    return(static_cast<int> (Robot::m_RunMode));
}

//
//  static variables in Robot
//
    Robot::RunModeType      Robot::m_RunMode;


#ifndef RUNNING_FRC_TESTS
int main()
{
    return StartRobot<Robot>();
}
#endif
