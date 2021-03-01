
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
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

// Team 302 includes
#include <subsys/SwerveChassis.h>

// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>

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

// Team 302 includes
#include <hw/DragonLimelight.h>
#include <hw/factories/LimeLightFactory.h>
#include <utils/GoalDetection.h>

// Third Party Includes

using namespace std;


GoalDetection* GoalDetection::m_instance = nullptr;
GoalDetection* GoalDetection::GetInstance()
{
    if (GoalDetection::m_instance == nullptr)
    {
        GoalDetection::m_instance = new GoalDetection();
    }
    return GoalDetection::m_instance;
}

GoalDetection::GoalDetection()
{
    m_camera = LimelightFactory::GetLimelightFactory()->GetLimelight();
    m_camera->SetLEDMode(DragonLimelight::LED_ON);
}

bool GoalDetection::SeeOuterGoal() const
{
    auto cam = m_camera;
    return (cam != nullptr && cam->HasTarget() );
}
bool GoalDetection::SeeInnerGoal() const
{
    return SeeOuterGoal();
}

units::angle::degree_t GoalDetection::GetHorizontalAngleToOuterGoal() const
{
    units::angle::degree_t angle = 0_deg;
    auto cam = m_camera;
    if (cam != nullptr && cam->HasTarget() )
    {
        angle = cam->GetTargetHorizontalOffset();
    }
    return angle;
}
units::angle::degree_t GoalDetection::GetHorizontalAngleToInnerGoal() const
{
    return GetHorizontalAngleToOuterGoal();
}

units::angle::degree_t GoalDetection::GetVerticalAngleToOuterGoal() const
{
    units::angle::degree_t angle = 0_deg;
    auto cam = m_camera;
    if (cam != nullptr && cam->HasTarget() )
    {
        angle = cam->GetTargetHorizontalOffset();
    }
    return angle;
}
units::angle::degree_t GoalDetection::GetVerticalAngleToInnerGoal() const
{
    /**  If we align with center of outer they are the same on flat goal
    auto angle = GetVerticalAngleToOuterGoal();
    auto dist  = GetDistanceToOuterGoal();
    auto deltaOuter = m_camera->GetTargetHeight() - m_camera->GetMountingHeight();
    auto deltaInner = delta + units::length::inch_t(15.0);
    **/
    return GetVerticalAngleToOuterGoal();
}

units::length::inch_t GoalDetection::GetDistanceToOuterGoal() const
{
    units::length::inch_t dist = units::length::inch_t(0.0);
    if (m_camera != nullptr && m_camera->HasTarget() )
    {
        dist = m_camera->EstimateTargetDistance();
    }
    return dist;
}

units::length::inch_t GoalDetection::GetDistanceToInnerGoal() const
{
    return GetDistanceToInnerGoal();
}
