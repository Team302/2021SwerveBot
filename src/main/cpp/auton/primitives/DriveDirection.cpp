
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
#include <cmath>
#include <memory>
#include <string>

// FRC includes
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/velocity.h>

// 302 includes
#include <auton/PrimitiveParams.h>
#include <auton/primitives/IPrimitive.h>
#include <subsys/SwerveChassisFactory.h>
#include <utils/Logger.h>
#include <auton/primitives/DriveDirection.h>
#include <hw/factories/PigeonFactory.h>
#include <hw/DragonPigeon.h>



using namespace std;
using namespace frc;

DriveDirection::DriveDirection() :  IPrimitive(),
                                    m_chassis(SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
                                    m_heading(0.0),
                                    m_speed(0.0),
                                    m_frontLeft(m_chassis->GetFrontLeft()),
                                    m_frontRight(m_chassis->GetFrontRight()),
                                    m_backLeft(m_chassis->GetBackLeft()),
                                    m_backRight(m_chassis->GetBackRight())
{
}

void DriveDirection::Init(PrimitiveParams* params)
{
    m_speed     = units::velocity::feet_per_second_t(params->GetDriveSpeed()/12.0);
    m_heading   = units::angle::degree_t(params->GetHeading());

    SwerveModuleState state;
    state.speed = 0_mps;
    state.angle = Rotation2d(m_heading);
    m_frontLeft.get()->SetDesiredState(state);
    m_frontRight.get()->SetDesiredState(state);
    m_backLeft.get()->SetDesiredState(state);
    m_backRight.get()->SetDesiredState(state);
}

void DriveDirection::Run()
{
    SwerveModuleState state;
    state.speed = m_speed;
    state.angle = Rotation2d(m_heading);

    m_frontLeft.get()->SetDesiredState(state);
    m_frontRight.get()->SetDesiredState(state);
    m_backLeft.get()->SetDesiredState(state);
    m_backRight.get()->SetDesiredState(state);
}

bool DriveDirection::IsDone()
{
    bool isDone = false;
    return isDone;
}