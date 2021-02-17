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
#include <string>

// frc includes
#include <units/angular_velocity.h>

// Team 302 includes
#include <controllers/AngularVelocityControllerMC.h>
#include <controllers/ControlData.h>
#include <utils/ConversionUtils.h>
#include <utils/Logger.h>

using namespace std;

AngularVelocityControllerMC::AngularVelocityControllerMC
(
    shared_ptr<IDragonMotorController>                  motor,
    units::angular_velocity::revolutions_per_minute_t   target,
    shared_ptr<ControlData>                             controlData
) : m_motor( motor ),
    m_target( target ),
    m_control( controlData )
{
    if (m_motor.get() == nullptr )
    {
        Logger::GetLogger()->LogError( Logger::LOGGER_LEVEL::ERROR_ONCE, 
                                       string( "AngularVelocityControllerMC constructor" ), 
                                       string( "motorController is nullptr" ) );
    }    
    if (m_control.get() == nullptr )
    {
        Logger::GetLogger()->LogError( Logger::LOGGER_LEVEL::ERROR_ONCE, 
                                       string( "AngularVelocityControllerMC constructor" ), 
                                       string( "ControlData is nullptr" ) );
    }

    UpdateTarget(target);
    Init();
}

void AngularVelocityControllerMC::Init()
{
    if ( m_motor.get() != nullptr && m_control.get() != nullptr )
    {
        m_motor.get()->SetControlConstants( m_control.get() );
        m_motor.get()->SetControlMode( m_control.get()->GetMode() );
    }
}

void AngularVelocityControllerMC::UpdateTarget
(
    units::angular_velocity::revolutions_per_minute_t target
)
{
    m_target = target;
	m_unitsPer100MS = ConversionUtils::RPSToCounts100ms( target.to<double>()/60.0, m_motor.get()->GetCountsPerRev() );
}

void AngularVelocityControllerMC::Run()
{
    if ( m_motor.get() != nullptr )
    {
        m_motor.get()->Set( m_unitsPer100MS );
    }
}

units::angular_velocity::revolutions_per_minute_t AngularVelocityControllerMC::GetAngularVelocity()
{
    auto rps = m_motor.get() != nullptr ? m_motor.get()->GetRPS() : 0.0;
    units::angular_velocity::revolutions_per_minute_t rpm(rps * 60.0);
    return rpm;
}

