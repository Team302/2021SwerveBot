
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

GoalDetection::GoalDetection() : m_camera(LimelightFactory::GetLimelightFactory()->GetLimelight()),
                                 m_queriesSinceLastSeen(100),
                                 m_lastHor(360_deg),
                                 m_lastVert(360_deg),
                                 m_lastDist(units::length::inch_t(360.0))
{
    m_camera->SetPipeline(1);
}

bool GoalDetection::SeeOuterGoal() const
{
    Logger::GetLogger()->ToNtTable(string("Goal Detection"), string("camera"), m_camera != nullptr ? "true" : "false");
    Logger::GetLogger()->ToNtTable(string("Goal Detection"), string("target"), m_camera != nullptr ? (m_camera->HasTarget() ? "true" : "false") : "false");
    
    auto seen =  (m_camera != nullptr && m_camera->HasTarget() );
    m_queriesSinceLastSeen = seen ? 0 : m_queriesSinceLastSeen+1;

    seen = m_queriesSinceLastSeen < 5 && !seen ? true : seen;
    return seen;
}
bool GoalDetection::SeeInnerGoal() const
{
    return SeeOuterGoal();
}

units::angle::degree_t GoalDetection::GetHorizontalAngleToOuterGoal() const
{
    auto seen =  (m_camera != nullptr && m_camera->HasTarget() );
    auto angle = m_lastHor;
    if ( seen )
    {
        angle = -1.0 * m_camera->GetTargetHorizontalOffset();
        m_lastHor = angle;
    }
    return angle;   
}
units::angle::degree_t GoalDetection::GetHorizontalAngleToInnerGoal() const
{
    return GetHorizontalAngleToOuterGoal();
}

units::angle::degree_t GoalDetection::GetVerticalAngleToOuterGoal() const
{
    auto seen =  (m_camera != nullptr && m_camera->HasTarget() );
    auto angle = m_lastVert;
    if ( seen )
    {
        angle = m_camera->GetTargetVerticalOffset();
        m_lastVert = angle;
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
    auto seen =  (m_camera != nullptr && m_camera->HasTarget() );
    auto dist = m_lastDist;
    if ( seen )
    {
        dist = m_camera->EstimateTargetDistance();
        m_lastDist = dist;
    }
    return dist;   
}

units::length::inch_t GoalDetection::GetDistanceToInnerGoal() const
{
    return GetDistanceToOuterGoal();
}
