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
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);

        controller->SetAxisProfile( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, IDragonGamePad::AXIS_PROFILE::CUBED );
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);

        controller->SetAxisProfile( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, IDragonGamePad::AXIS_PROFILE::CUBED );
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
   }
}



/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void SwerveDrive::Run( )
{
    double drive = 0.0;
    double steer = 0.0;
    double rotate = 0.0;
    auto controller = GetController();
    if ( controller != nullptr )
    {
        drive  = controller->GetAxisValue( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE) ;
        steer  = controller->GetAxisValue( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER);
        rotate =  controller->GetAxisValue( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE);
    }
    m_chassis.get()->Drive(drive, steer, rotate, true);
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
    if ( controller != nullptr)
    {
        if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::REZERO_PIGEON))
        {
            auto factory = PigeonFactory::GetFactory();
            auto m_pigeon = factory->GetPigeon();
            m_pigeon->ReZeroPigeon( 0, 0);
        }
    }
}

