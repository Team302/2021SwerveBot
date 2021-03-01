
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
#include <units/time.h>
#include <units/velocity.h>

// 302 includes
#include <auton/PrimitiveParams.h>
#include <auton/primitives/DriveDirection.h>
#include <auton/primitives/DriveTime.h>



using namespace std;
using namespace frc;

DriveTime::DriveTime() :  DriveDirection(), m_time(), m_startSpeed()
{
}

void DriveTime::Init(PrimitiveParams* params)
{
    m_time = units::time::second_t(params->GetTime());
    m_startSpeed = units::velocity::feet_per_second_t(params->GetDriveSpeed()/12.0);
    m_timer = make_shared<Timer>();
    m_timer.get()->Start();
}

void DriveTime::Run()
{
    // TODO:: add PID to slow down as we get close instead of this ramp
    auto currTime = units::time::second_t(m_timer.get()->Get());
    auto percent = (currTime / m_time);
    if (percent > 0.75)
    {
        UpdateSpeed((1.0-percent)*m_startSpeed);
    }
    DriveDirection::Run();
}


bool DriveTime::IsDone()
{
    auto currTime = units::time::second_t(m_timer.get()->Get());
    return (currTime > m_time);
}