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
#include <units/math.h>
#include <units/acceleration.h>

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

TurnAngle::TurnAngle() : m_chassis( SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
                         m_timer( make_unique<Timer>() ),
                         m_maxTime(0.0),
                         m_currentChassisPosition(units::meter_t(0), units::meter_t(0), units::radian_t(0))
{
}


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

        units::degree_t targetAngle(params->GetHeading());

    if( params->GetID() == TURN_ANGLE_ABS)
    {
       m_targetAngle = targetAngle;
    } else if( params->GetID() == TURN_ANGLE_REL)
    {
       m_targetAngle = targetAngle + heading;
    } else
    {
        Logger::GetLogger()->LogError( string("TurnAngle::Run"), string("turning type not absolute or relative"));
    }

    m_driveSpeed = params->GetDriveSpeed();

    double targetAngleDouble = m_targetAngle.to<double>();

    //Turning Optimization
    TurnAngle::IsGreaterThan(targetAngleDouble, m_driveSpeed);
    TurnAngle::IsLessThan(targetAngleDouble, m_driveSpeed);
    TurnAngle::ReduceCases(targetAngleDouble, m_driveSpeed);

    //Create targetAngleDegree which is targetAngleDouble as units::degree_t
    units::degree_t targetAngleDegree(targetAngleDouble);
    //Set the targetAngle to the optimized angle
    m_targetAngle = targetAngleDegree;

   //Calculate diameter of robot to find arc length
     double trackSquared = m_chassis.get()->GetTrack().to<double>() * m_chassis.get()->GetTrack().to<double>();
    double wheelbaseSquared = m_chassis.get()->GetWheelBase().to<double>() * m_chassis.get()->GetWheelBase().to<double>();

   double hypBeforeSqrt(trackSquared * wheelbaseSquared);
     units::length::inch_t hypAfterSqrt(sqrt(hypBeforeSqrt));
   
     units::length::inch_t m_circumference = pi * hypAfterSqrt;

   //possiblly get absolute valie after heading - m_targetAngle, if ABS and want to go to 90 and heading is 0, we go -90
   //does a negative distance mean we go in an opposite direction
     units::degree_t arcAngle = heading - m_targetAngle;

     units::length::inch_t m_goalDistance = ( units::math::abs(arcAngle) / units::degree_t(360)) * m_circumference;

    units::acceleration::meters_per_second_squared_t maxAccel(m_chassis.get()->GetMaxAcceleration());
    units::meters_per_second_t                       maxSpeed = m_chassis.get()->GetMaxSpeed();

    frc::TrapezoidProfile<units::meters>::Constraints m_constraints{ maxSpeed, maxAccel};

    m_goal = {m_goalDistance, units::velocity::meters_per_second_t( m_driveSpeed) };

    units::length::meter_t circumferenceMeter(m_circumference);

    m_distanceToAngleConversion = (2 * pi ) / circumferenceMeter.to<double>();

}

void TurnAngle::Run()
{
    units::meters_per_second_t      xSpeed(0);
    units::meters_per_second_t      ySpeed(0);

    static constexpr units::second_t kDt = 20_ms;

    frc::TrapezoidProfile<units::meters> profile{m_constraints, m_goal, m_setpoint};

    m_setpoint = profile.Calculate(kDt);

    units::angular_velocity::radians_per_second_t radianTrapezoidSpeed(m_setpoint.velocity.to<double>() * m_distanceToAngleConversion);

    //Angular speed calculation
    if (m_reverse)
    {
        m_chassis->Drive(xSpeed, ySpeed, -1 * radianTrapezoidSpeed, true);
    } else if (!m_reverse)
    {
        m_chassis->Drive(xSpeed, ySpeed, radianTrapezoidSpeed, true);
    }        
}

bool TurnAngle::IsDone()
{

    frc::SwerveDrivePoseEstimator<4> chassisPoseEstimatorDone = m_chassis.get()->GetPose();

    Pose2d currentChassisPos = chassisPoseEstimatorDone.GetEstimatedPosition();

    units::degree_t tolerance(2);

    if( units::math::abs(currentChassisPos.Rotation().Degrees() - m_targetAngle) <= tolerance)
    {
        m_isDone = true;
    } else 
    {
        m_isDone = false;
    }

    if(m_timer->HasPeriodPassed( m_maxTime ))
    {
        m_isDone = true;
    }else 
    {
        m_isDone = false;
    }
    

    return m_isDone;
}

void TurnAngle::IsGreaterThan( double& angle, float& speed)
{
    if ( angle > 180)
    {
        angle = 360 - angle;
        speed = speed * -1;
    }
}

void TurnAngle::IsLessThan( double& angle, float& speed)
{
    if ( angle < -180)
    {
        angle = 360 + angle;
        speed = speed * -1;
    }
}

void TurnAngle::ReduceCases( double& angle, float& speed)
{
    angle = abs(angle);

}