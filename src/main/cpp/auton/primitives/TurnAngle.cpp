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
#include <frc/geometry/Translation2d.h>

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
    units::degree_t targetAngle
) : m_chassis( SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
                         m_timer( make_unique<Timer>() ),
                         m_maxTime(0.0),
                         m_isDone(false),
                         m_turnRight(true),
                         m_currentChassisPosition(units::meter_t(0), units::meter_t(0), units::radian_t(0))
{

    //How to get targetAngle into rest of program
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




void TurnAngle::Init
(
    PrimitiveParams* params
)
{
   m_maxTime = params->GetTime();
   m_timer->Reset();
   m_timer->Start(); 

    auto pigeon = PigeonFactory::GetFactory()->GetPigeon();
    units::degree_t heading(( pigeon != nullptr ) ? pigeon->GetYaw() : 0.0);

   if(m_mode == TURN_ANGLE_ABS)
   {
       m_targetAngle = targetAngle;
   } else if( m_mode == TURN_ANGLE_REL)
   {
       m_targetAngle = targetAngle + heading;
   }

    //integrate pid loop based on singular wheel pos

    //move position check out of timer expiration

    //check to see if the chassis is at the estimated angle


}



void TurnAngle::Run()
{      
    frc::SwerveDrivePoseEstimator<4> chassisPoseEstimator = m_chassis.get()->GetPoseEstimator();

    m_currentChassisPosition = chassisPoseEstimator.GetEstimatedPosition();

    units::meters_per_second_t xSpeed(0);
    units::meters_per_second_t ySpeed(0);
    units::radians_per_second_t maxAccelerationRadian(m_chassis.get()->GetMaxAcceleration());

    if( m_mode == TURN_ANGLE_ABS)
    {
        if(m_turnRight)
        {
            m_chassis->Drive(xSpeed, ySpeed, -1 * (maxAccelerationRadian / 2), true);
        } else if (!m_turnRight)
        {
            m_chassis->Drive(xSpeed, ySpeed, maxAccelerationRadian / 2, true);
        }
    } else if ( m_mode == TURN_ANGLE_REL)
    {
        m_relativeAngle = m_targetAngle + heading;

        if(m_turnRight)
        {
            m_chassis->Drive( xSpeed, ySpeed, -1 * (maxAccelerationRadian / 2), true);
        } else if (!m_turnRight)
        {
            m_chassis->Drive( xSpeed, ySpeed, maxAccelerationRadian / 2, true);
        }
    } else 
    {
        Logger::GetLogger()->LogError( string("TurnAngle::Run"), string("turning type not absolute or relative"));
    }
}

bool TurnAngle::IsDone()
{

    frc::SwerveDrivePoseEstimator<4> chassisPoseEstimatorDone = m_chassis.get()->GetPoseEstimator();

    Pose2d currentChassisPos = chassisPoseEstimatorDone.GetEstimatedPosition();

    units::degree_t angleDifference = m_targetAngle - currentChassisPos.Rotation().Degrees();

    if()
    {
        m_isDone = true;
    } else 
    {
    }

    if(m_timer->HasPeriodPassed( m_maxTime ))
    {
        m_isDone = true;
    }else 
    {
        m_isDone = true;
    }
    

    return m_isDone;
}