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

TurnAngle::TurnAngle() : m_chassis( SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
                         m_timer( make_unique<Timer>() ),
                         m_targetAngle(0.0),
                         m_maxTime(0.0),
                         m_backLeftPos(0.0),
                         m_backRightPos(0.0),
                         m_frontLeftPos(0.0),
                         m_frontRightPos(0.0),
                         m_isDone(false),
                         m_control(nullptr),
                        m_turnRight(true)
{
        m_control = new ControlData
        ( ControlModes::CONTROL_TYPE::POSITION_INCH,
           ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
           string("TurnAngle"),
           3.0,
           0.0,
		   0.0,
		   0.0,
		   0.0,
		   0.0,
		   0.0,
		   1.0,
		   0.0   );
}

TurnAngle::TurnAngle
(
    ControlData* control
) : m_chassis( SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
						 m_timer( make_unique<Timer>() ),
						 m_targetAngle(0.0),
						 m_maxTime(0.0),
						 m_backLeftPos(0.0),
                         m_backRightPos(0.0),
                         m_frontLeftPos(0.0),
                         m_frontRightPos(0.0),
						 m_isDone(false),
						 m_control(nullptr),
						 m_turnRight(true)
{
}

void TurnAngle::Init
(
    PrimitiveParams* params
)
{
    /*
    m_isDone = false;

    //get the current angle/heading of the robot
    auto pigeon = PigeonFactory::GetFactory()->GetPigeon();
    auto startHeading = pigeon != nullptr ? pigeon->GetYaw() : 0.0;
    auto angle = params->GetHeading();

    auto isRelative = params->GetID() == TURN_ANGLE_REL;
    m_targetAngle = isRelative ? (startHeading + angle) : angle;
    auto delta = isRelative ? angle : (startHeading - m_targetAngle);
    m_turnRight = delta > 0.0;

    // figure out how far each side of the robot needs to move to turn in place (circle/arc) in order to reach
	// the angle.   So, 2 * PI * radius is the distance a wheel would travel to turn 360 degrees.  
	// Thus, to travel 90 degrees, the wheel would need to move a quarter of this distance (90/360).
	// This won't be entirely accurate because of scrub and slippage, the encoder may show a slightly different distance
	// but we'll use this to get approximately there and then use the gyro and a slow speed to finish it up. 
    auto turningRadius = m_chassis->GetTrack() / 2.0;
    auto arcLen = ((2.0 * pi * turningRadius) * delta ) / 360.0;

    //dummy pose estimator and translation2d for right now, will be changed later
    //Original code:
    
        m_leftPos = m_chassis->GetCurrentLeftPosition() + arcLen;
	    m_rightPos = m_chassis->GetCurrentRightPosition() - arcLen;
    
    
    //{
    frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
        frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
        frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
        frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};
        frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};
    frc::SwerveDrivePoseEstimator<4> m_poseEstimatorDummy{  frc::Rotation2d(), 
                                                           frc::Pose2d(), 
                                                           m_kinematics,
                                                           {0.1, 0.1, 0.1},   
                                                           {0.05},        
                                                           {0.1, 0.1, 0.1} };

    m_frontLeftPos = m_poseEstimatorDummy + arcLen;
    m_frontRightPos = m_poseEstimatorDummy - arcLen;
    m_backLeftPos = m_poseEstimatorDummy + arcLen;
    m_backRightPos = m_poseEstimatorDummy - arcLen;
    //}

    //Original code from 2021InfiniteRecharge2
    // Set the output to the wheels
	//m_chassis->SetControlConstants( m_control );
	//m_chassis->SetOutput( m_control->GetMode(), m_leftPos, m_rightPos );

    m_maxTime = params->GetTime();
    m_timer->Reset();
    m_timer->Start();
    */
}



void TurnAngle::Run()
{
       /* //Check the pigeon to see if we've reached that target, if we have set m_isDone to true and the speeds to 0.0
        auto pigeon = PigeonFactory::GetFactory()->GetPigeon();
        auto heading = ( pigeon != nullptr ) ? pigeon->GetYaw() : 0.0;

        auto delta = heading - m_targetAngle;
        m_isDone = ( abs(delta) < ANGLE_THRESH);
        if ( !m_isDone )
        {
            auto currFrontLeftPos =
            auto currFrontRightPos =
            auto currBackLeftPos =
            auto currBackRightPos =
        }
        */

       
       units::radians_per_second_t fullCircle(360);
       units::meters_per_second_t xSpeed(0);
       units::meters_per_second_t ySpeed(0);

       m_chassis->Drive(xSpeed, ySpeed, fullCircle, true);
}