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
#include <algorithm>
#include <memory>
#include <string>
#include <units/angular_velocity.h>

//FRC Includes
#include <frc/Timer.h>
#include <wpi/math>

//Team 302 Includes
#include <auton/PrimitiveParams.h>
#include <auton/primitives/IPrimitive.h>
#include <auton/primitives/TurnAngle.h>
#include <controllers/ControlData.h>
#include <controllers/ControlModes.h>
#include <hw/DragonPigeon.h>
#include <hw/factories/PigeonFactory.h>
#include <subsys/SwerveChassisFactory.h>
#include <subsys/SwerveChassis.h>
#include <utils/Logger.h>

using namespace std;
using namespace frc;

using namespace wpi::math;

TurnAngle::TurnAngle
(   PRIMITIVE_IDENTIFIER        mode,
    units::radians_per_second_t targetAngle
) : m_chassis( SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
                         m_timer( make_unique<Timer>() ),
                         m_maxTime(0.0),
                         m_isDone(false),
                         m_turnRight(true)
{
}

/*
TurnAngle::TurnAngle
(
    PRIMITIVE_IDENTIFIER        mode,
    units::radians_per_second_t targetAngle
) : m_chassis( SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
						 m_timer( make_unique<Timer>() ),
						 m_maxTime(0.0),
						 m_isDone(false),
						 m_turnRight(true)
{
}
*/  //Don't know why this was declared twice in original turn angle code



/*
void TurnAngle::Init
(
    PrimitiveParams* params
)
{
    auto pigeon = PigeonFactory::GetFactory()->GetPigeon();
    auto startHeading = pigeon != nullptr ? pigeon->GetYaw() : 0.0;
    auto angle = params->GetHeading();
}
*/


void TurnAngle::Run()
{      

    auto pigeon = PigeonFactory::GetFactory()->GetPigeon();
    auto heading = ( pigeon != nullptr ) ? pigeon->GetYaw() : 0.0;

    units::meters_per_second_t xSpeed(0);
    units::meters_per_second_t ySpeed(0);
    units::radians_per_second_t relativeAngle(0);
    units::radians_per_second_t radianHeading(heading);

    if( m_mode == TURN_ANGLE_ABS)
    {
       m_chassis->Drive(xSpeed, ySpeed, m_targetAngle, true);
    } else if ( m_mode == TURN_ANGLE_REL)
    {
        relativeAngle = m_targetAngle - radianHeading;

        m_chassis->Drive( xSpeed, ySpeed, relativeAngle, true);
    }
       

}