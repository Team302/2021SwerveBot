
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

// C++ Includes
#include <memory>

// FRC includes


// Team 302 Includes
#include <Robot.h>
#include <states/chassis/SwerveDrive.h>
#include <states/shooter/ShooterStateMgr.h>
#include <subsys/MechanismFactory.h>
#include <subsys/Shooter.h>
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
    // Read the robot definition from the xml configuration files and
    // create the hardware (chassis + mechanisms along with their talons,
    // solenoids, digital inputs, analog inputs, etc.
    unique_ptr<RobotDefn>  robotXml = make_unique<RobotDefn>();
    robotXml->ParseXML();
    m_cyclePrims= new CyclePrimitives();
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

}


/// @brief This initializes the autonomous state
/// @return void
void Robot::AutonomousInit() 
{
    m_cyclePrims->Init();
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

    m_cyclePrims->Run();

}


/// @brief This initializes the teleoperated state
/// @return void
void Robot::TeleopInit() 
{
    m_drive = make_shared<SwerveDrive>();
    m_drive.get()->Init();

    auto shooter = MechanismFactory::GetMechanismFactory()->GetShooter();
    m_shooterState = ( shooter.get() != nullptr ) ? ShooterStateMgr::GetInstance() : nullptr;
    if ( m_shooterState != nullptr )
    {
        m_shooterState->SetCurrentState(ShooterStateMgr::SHOOTER_STATE::OFF, true);
    }
    else 
    {
        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::ERROR_ONCE, string("TeleopInit"), string("no shooter state manager"));
    }

}


/// @brief Runs every 20 milliseconds when the teleoperated state is active.
/// @return void
void Robot::TeleopPeriodic() 
{
    m_drive.get()->Run();
    if ( m_shooterState != nullptr )
    {
        m_shooterState->RunCurrentState();
    }
}



/// @brief This initializes the test state
/// @return void
void Robot::TestInit() 
{

}


/// @brief Runs every 20 milliseconds when the test state is active.
/// @return void
void Robot::TestPeriodic() 
{

}

#ifndef RUNNING_FRC_TESTS
int main() 
{
    return StartRobot<Robot>(); 
}
#endif
