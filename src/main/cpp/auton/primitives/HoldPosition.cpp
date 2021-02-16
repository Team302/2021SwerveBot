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

//C++ Includes
#include <memory>
#include <string>
#include <units/velocity.h>

//FRC Includes
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>

//Team 302 Includes
#include <auton/PrimitiveParams.h>
#include <auton/primitives/HoldPosition.h>
#include <auton/primitives/IPrimitive.h>
#include <subsys/SwerveChassisFactory.h>

using namespace std;


HoldPosition::HoldPosition() :
                    m_chassis( SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
                    m_timeRemaining( 0.0 )//Will be changed in init
{
}

void HoldPosition::Init(PrimitiveParams* params) 
{
    //Get timeRemaining from params
    m_timeRemaining = params->GetTime();

    /*
    SwerveModuleState m_frontLeftState = m_chassis.get()->GetFrontLeft().get()->GetState();
    SwerveModuleState m_frontRightState = m_chassis.get()->GetFrontRight().get()->GetState();
    SwerveModuleState m_backLeftState = m_chassis.get()->GetBackLeft().get()->GetState();
    SwerveModuleState m_backRightState = m_chassis.get()->GetBackRight().get()->GetState();
    */

    m_frontLeft = m_chassis.get()->GetFrontLeft();
    m_frontRight = m_chassis.get()->GetFrontRight();
    m_backLeft = m_chassis.get()->GetBackLeft();
    m_backRight = m_chassis.get()->GetBackRight();

    //Probably have to do 45, 135, -45, -135 for each wheel
    m_frontLeftRadian = units::degree_t(-45);
    m_frontRightRadian = units::degree_t(45);
    m_backLeftRadian = units::degree_t(-135);
    m_backRightRadian = units::degree_t(135);

    m_frontLeftRotation = m_frontLeftRadian;
    m_frontRightRotation = m_frontRightRadian;
    m_backLeftRotation = m_backLeftRadian;
    m_backRightRotation = m_backRightRadian;

    units::meters_per_second_t speedMPS(-0.05); //Speed the wheels will be going facing inward toward the robot, about 2 inches per second

    m_frontLeftState = {speedMPS, m_frontLeftRotation};
    m_frontRightState = {speedMPS, m_frontRightRotation};
    m_backLeftState = {speedMPS, m_backLeftRotation};
    m_backRightState = {speedMPS, m_backRightRotation};

}

void HoldPosition::Run() 
{
    //Decrement time remaining
    m_timeRemaining -= IPrimitive::LOOP_LENGTH;

    //Turn all wheels to face inward and apply very little speed
    m_frontLeft.get()->SetDesiredState(m_frontLeftState);
    m_frontRight.get()->SetDesiredState(m_frontRightState);
    m_backLeft.get()->SetDesiredState(m_backLeftState);
    m_backRight.get()->SetDesiredState(m_backRightState);
}

bool HoldPosition::IsDone()
{
    //Return true when time runs out
    bool done = ((m_timeRemaining <= (IPrimitive::LOOP_LENGTH / 2.0)));
    return done;
}