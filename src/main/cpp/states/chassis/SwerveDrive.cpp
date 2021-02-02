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

//C++ Inlcudes
#include <memory>
#include <units/velocity.h>
#include <units/angular_velocity.h>

//Team 302 Includes
#include <states/chassis/SwerveDrive.h>
#include <hw/DragonPigeon.h>
#include <gamepad/IDragonGamePad.h>
#include <gamepad/TeleopControl.h>
#include <states/IState.h>
#include <subsys/SwerveChassisFactory.h>
#include <hw/factories/PigeonFactory.h>
#include <utils/Logger.h>


using namespace std;

/// @brief initialize the object and validate the necessary items are not nullptrs
SwerveDrive::SwerveDrive() : IState(),
                             m_chassis( SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis() ),
                             m_controller( TeleopControl::GetInstance() )
{
    if ( m_controller == nullptr )
    {
        Logger::GetLogger()->LogError( string("SwerveDrive::SwerveDrive"), string("TeleopControl is nullptr"));
    }

    if ( m_chassis.get() == nullptr )
    {
        Logger::GetLogger()->LogError( string("SwerveDrive::SwerveDrive"), string("Chassis is nullptr"));
    }
}

/// @brief initialize the profiles for the various gamepad inputs
/// @return void
void SwerveDrive::Init()
{
    auto controller = GetController();
    if ( controller != nullptr )
    {
        controller->SetAxisProfile( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, IDragonGamePad::AXIS_PROFILE::CUBED );
        controller->SetAxisProfile( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, IDragonGamePad::AXIS_PROFILE::CUBED );
        controller->SetAxisProfile( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, IDragonGamePad::AXIS_PROFILE::CUBED );
    }
}



/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void SwerveDrive::Run( )
{
    //Get drive, steer, and rotate values
    units::velocity::meters_per_second_t maxSpeed = m_chassis.get()->GetMaxSpeed();
    units::velocity::meters_per_second_t driveSpeed = maxSpeed * GetDrive();
    units::velocity::meters_per_second_t turnSpeed = maxSpeed * GetSteer();

    units::radians_per_second_t maxRotateSpeed = m_chassis.get()->GetMaxAngularSpeed();
    units::radians_per_second_t rotateSpeed = maxRotateSpeed * GetRotate(); 

    m_chassis.get()->Drive(driveSpeed, turnSpeed, rotateSpeed, true);
}

/// @brief indicates that we are not at our target
/// @return bool
bool SwerveDrive::AtTarget() const
{
    return false;
}

void SwerveDrive::RunCurrentState()
{
    auto controller = TeleopControl::GetInstance();

    auto factory = PigeonFactory::GetFactory();

    auto m_pigeon = factory->GetPigeon();

    if ( controller != nullptr)
    {
        if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::REZERO_PIGEON))
        {
            m_pigeon->ReZeroPigeon( 0, 0);
        }
    }
}

/// @brief get the drive component from the game controller
/// @return double - drive value between -1.0 and 1.0
double SwerveDrive::GetDrive()
{
    auto controller = GetController();
    return ( ( controller != nullptr ) ? controller->GetAxisValue( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE) : 0.0);
}

/// @brief get the steer component from the game controller
/// @return double - steer value between -1.0 and 1.0
double SwerveDrive::GetSteer()
{
    auto controller = GetController();
    return ( ( controller != nullptr ) ? controller->GetAxisValue( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER) : 0.0);
}

/// @brief get the rotate component from the game controller
/// @return double - rotate value between -1.0 and 1.0
double SwerveDrive::GetRotate()
{
    auto controller = GetController();
    return ( ( controller != nullptr ) ? controller->GetAxisValue( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE) : 0.0);
}