
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
#include <map>
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <hw/usages/MotorControllerUsage.h>

// Third Party Includes

using namespace std;

MotorControllerUsage* MotorControllerUsage::m_instance = nullptr;
MotorControllerUsage* MotorControllerUsage::GetInstance()
{
    if ( m_instance == nullptr )
    {
        m_instance = new MotorControllerUsage();
    }
    return m_instance;
}

MotorControllerUsage::MotorControllerUsage()
{
    m_usageMap["DRIVE"]  = MOTOR_CONTROLLER_USAGE::DRIVE;
    m_usageMap["TURN"]   = MOTOR_CONTROLLER_USAGE::TURN;

    m_usageMap["INTAKE"] = MOTOR_CONTROLLER_USAGE::INTAKE;
    m_usageMap["INTAKE2"] = MOTOR_CONTROLLER_USAGE::INTAKE2;
    m_usageMap["BALL_TRANSFER"] = MOTOR_CONTROLLER_USAGE::BALL_TRANSFER;
    m_usageMap["TURRET"] = MOTOR_CONTROLLER_USAGE::TURRET;
    m_usageMap["SHOOTER_1"] = MOTOR_CONTROLLER_USAGE::SHOOTER_1;
    m_usageMap["SHOOTER_2"] = MOTOR_CONTROLLER_USAGE::SHOOTER_2;
    
    m_usageMap["BALL_HOPPER"] = MOTOR_CONTROLLER_USAGE::BALL_HOPPER;
}

MotorControllerUsage::~MotorControllerUsage()
{
    m_usageMap.clear();
}

MotorControllerUsage::MOTOR_CONTROLLER_USAGE MotorControllerUsage::GetUsage
(
    string              usageString
)
{
    return m_usageMap.find(usageString)->second;
}

