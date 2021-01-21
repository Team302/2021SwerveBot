
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
#include <hw/usages/IDragonMotorControllerMap.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <subsys/DragonSwerveModule.h>
#include <subsys/SwerveChassis.h>
#include <subsys/SwerveChassisFactory.h>


// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>

using namespace std;
using namespace ctre::phoenix::sensors;



SwerveChassisFactory* SwerveChassisFactory::m_swerveChassisFactory = nullptr;
SwerveChassisFactory* SwerveChassisFactory::GetSwerveChassisFactory()
{
	if ( SwerveChassisFactory::m_swerveChassisFactory == nullptr )
	{
		SwerveChassisFactory::m_swerveChassisFactory = new SwerveChassisFactory();
	}
	return SwerveChassisFactory::m_swerveChassisFactory;
}

//=====================================================================================
/// Method:         CreateSwerveModule
/// Description:    Find or create the swerve module
/// Returns:        DragonSwerveModule *    pointer to the swerve module or nullptr if it 
///                                         doesn't exist and cannot be created.
//=====================================================================================
std::shared_ptr<DragonSwerveModule> SwerveChassisFactory::CreateSwerveModule
(
    DragonSwerveModule::ModuleID                            type, 
    const IDragonMotorControllerMap&        				motorControllers,   // <I> - Motor Controllers
    std::shared_ptr<ctre::phoenix::sensors::CANCoder>		canCoder,
    units::degree_t                                         turnOffset
)
{
    std::shared_ptr<DragonSwerveModule> swerve = nullptr;
	auto driveMotor = GetMotorController(motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::DRIVE);
	auto turnMotor  = GetMotorController(motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::TURN);

    switch (type)
    {
        case DragonSwerveModule::ModuleID::LEFT_FRONT:
            if ( m_leftFront.get() == nullptr )
            {
                m_leftFront = make_shared<DragonSwerveModule>(type, driveMotor, turnMotor, canCoder, turnOffset );
            }
            swerve = m_leftFront;
            break;

        case DragonSwerveModule::ModuleID::LEFT_BACK:
            if ( m_leftBack.get() == nullptr )
            {
                m_leftBack = make_shared<DragonSwerveModule>(type, driveMotor, turnMotor, canCoder, turnOffset );
            }
            swerve = m_leftBack;

            break;

        case DragonSwerveModule::ModuleID::RIGHT_FRONT:
            if ( m_rightFront.get() == nullptr )
            {
                m_rightFront = make_shared<DragonSwerveModule>(type, driveMotor, turnMotor, canCoder, turnOffset );
            }
            swerve = m_rightFront;
            break;

        case DragonSwerveModule::ModuleID::RIGHT_BACK:
            if ( m_rightBack.get() == nullptr )
            {
                m_rightBack = make_shared<DragonSwerveModule>(type, driveMotor, turnMotor, canCoder, turnOffset );
            }            
            swerve = m_rightBack;
            break;

        default:
            break;
    }

    return swerve;
}

//=====================================================================================
/// Method:         CreateSwerveChassis
/// Description:    Find or create the swerve chassis
/// Returns:        SwerveChassis*      pointer to the swerve chassis or nullptr if it 
///                                     doesn't exist and cannot be created.
//=====================================================================================
shared_ptr<SwerveChassis> SwerveChassisFactory::CreateSwerveChassis
(
    std::shared_ptr<DragonSwerveModule> frontLeft, 
    std::shared_ptr<DragonSwerveModule> frontRight,
    std::shared_ptr<DragonSwerveModule> backLeft, 
    std::shared_ptr<DragonSwerveModule> backRight, 
    units::length::inch_t wheelDiameter,
    units::length::inch_t wheelBase,
    units::length::inch_t track,
    units::velocity::meters_per_second_t maxSpeed,
    double maxAcceleration
)
{
    if ( m_chassis.get() == nullptr )
    {
        m_chassis = make_shared<SwerveChassis>(frontLeft, frontRight, backLeft, backRight, wheelDiameter, wheelBase, track, maxSpeed, maxAcceleration );
    }

    return m_chassis;
}


shared_ptr<IDragonMotorController> SwerveChassisFactory::GetMotorController
(
	const IDragonMotorControllerMap&				motorControllers,
	MotorControllerUsage::MOTOR_CONTROLLER_USAGE	usage
)
{
	shared_ptr<IDragonMotorController> motor;
	auto it = motorControllers.find( usage );
	if ( it != motorControllers.end() )  // found it
	{
		motor = it->second;
	}
	else
	{
		string msg = "motor not found; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "SwerveChassisFactory::GetMotorController" ), msg );
	}
	
	if ( motor.get() == nullptr )
	{
		string msg = "motor is nullptr; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "SwerveChassisFactory::GetMotorController" ), msg );
	}
	return motor;
}
