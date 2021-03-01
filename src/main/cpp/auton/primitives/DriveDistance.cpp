
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

// C++ includes
#include <memory>

// FRC includes
#include <units/velocity.h>

// 302 includes
#include <auton/PrimitiveParams.h>
#include <subsys/SwerveChassisFactory.h>
#include <auton/primitives/DriveDirection.h>
#include <auton/primitives/DriveDistance.h>



using namespace std;
using namespace frc;

DriveDistance::DriveDistance() :  DriveDirection(), m_startPose(), m_distance(), m_startSpeed(), m_percentToTarget(0.0)
{
}

void DriveDistance::Init(PrimitiveParams* params)
{
    m_distance = units::length::inch_t(params->GetDistance());
    m_startSpeed = units::velocity::feet_per_second_t(params->GetDriveSpeed()/12.0);
    m_startPose = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis().get()->GetPose().GetEstimatedPosition();
}

void DriveDistance::Run()
{
    // TODO:: add PID to slow down as we get close instead of this ramp
    auto percent = PercentToTarget();
    if (percent > 0.75)
    {
        UpdateSpeed((1.0-percent)*m_startSpeed);
    }
    DriveDirection::Run();
}


double DriveDistance::PercentToTarget()
{
    auto currentPose = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis().get()->GetPose().GetEstimatedPosition();
    Translation2d transStart { m_startPose.X(), m_startPose.Y() };
    Translation2d transCurrent{ currentPose.X(), currentPose.Y() };
    units::length::inch_t dist = transCurrent.Distance(transStart);
    return (dist/m_distance);
}

bool DriveDistance::IsDone()
{
    return (PercentToTarget() > 0.99);  // TODO:  is this close enough or should we do actual distance calculation
}