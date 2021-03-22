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

// FRC includes
#include <frc/smartdashboard/SmartDashboard.h>

// Team 302 includes
#include <controllers/ControlData.h>
#include <hw/DragonLimelight.h>
#include <hw/factories/LimelightFactory.h>
#include <states/turret/LimelightAim.h>
#include <states/Mech1MotorState.h>
#include <subsys/MechanismFactory.h>
#include <subsys/Turret.h>
#include <utils/GoalDetection.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace std;

LimelightAim::LimelightAim
(
    ControlData*    control, 
    double          target
) : IState(),
    m_motorState( make_unique<Mech1MotorState>(MechanismFactory::GetMechanismFactory()->GetTurret().get(), control, target)),
    m_turret(MechanismFactory::GetMechanismFactory()->GetTurret()),
    m_controlData(control),
    m_atTarget(false),
    m_target(target),
    m_targetPosition(0.0),
    m_start(false)
{
}

void LimelightAim::Init()
{
    m_turret.get()->SetControlConstants(0, m_controlData);
}

void LimelightAim::Run()
{
    auto target = m_target;
    if ( abs(target) < 0.1 )
    {
        m_target = 0.15;
        target = 0.15;
    }
    auto angle = units::angle::degree_t(360.0);
    auto goal = GoalDetection::GetInstance();
    if ( goal->SeeInnerGoal() )
    {
        angle = goal->GetHorizontalAngleToInnerGoal();
        if (abs(angle.to<double>()) < 1.0)
        {
            target = 0.0;
        }
    }

    auto pos = m_turret.get()->GetPosition() / 360.0;
    if (pos > m_max && target > 0.0)
    {
        target *= -1.0;
    }
    else if (pos < m_min && target < 0.0)
    {
        target *= -1.0;
    }


    target = (angle.to<double>() > 0.0) ? -1.0 * target : target;

    Logger::GetLogger()->ToNtTable("LimelightAim", "angle", angle.to<double>());
    Logger::GetLogger()->ToNtTable("LimelightAim", "output", target);

    m_turret.get()->UpdateTarget(target);
    m_turret.get()->Update();

    /**
   auto targetHorizontalOffset = GoalDetection::GetInstance()->GetHorizontalAngleToInnerGoal();
   double currentPosition = m_turret.get()->GetPosition();
   //m_turret.get()->UpdateTarget((currentPosition + targetHorizontalOffset + 2.0));
   m_turret.get()->UpdateTarget((currentPosition + targetHorizontalOffset.to<double>()));
   m_turret.get()->Update();
   **/
}

bool LimelightAim::AtTarget() const
{
    return m_atTarget;
}

void LimelightAim::UpdateTarget
( 
    double target
)
{
    m_target = target;
}