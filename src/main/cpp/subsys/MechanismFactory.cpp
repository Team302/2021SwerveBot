
//====================================================================================================================================================
/// Copyright 2020 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

//========================================================================================================
/// MechanismFactory.cpp
//========================================================================================================
///
/// File Description:
///     This controls the creation of mechanisms/subsystems
///
//========================================================================================================

// C++ Includes
#include <map>
#include <memory>

// FRC includes

// Team 302 includes
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/IDragonMotorControllerMap.h>
#include <hw/usages/DigitalInputMap.h>
#include <hw/usages/ServoMap.h>
#include <hw/DragonServo.h>
#include <hw/DragonDigitalInput.h>
#include <subsys/BallTransfer.h>
#include <subsys/Intake.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <utils/Logger.h>
#include <subsys/Shooter.h>
#include <subsys/Turret.h>

// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>
#include <ctre/phoenix/motorcontrol/StatusFrame.h>


using namespace std;
using namespace ctre::phoenix::sensors;
using namespace ctre::phoenix::motorcontrol;

//=====================================================================================
/// Method:         GetMechanismFactory
/// Description:    Find or create the mechanism factory
/// Returns:        MechanismFactory* pointer to the mechanism factory
//=====================================================================================
MechanismFactory* MechanismFactory::m_mechanismFactory = nullptr;
MechanismFactory* MechanismFactory::GetMechanismFactory()
{
	if ( MechanismFactory::m_mechanismFactory == nullptr )
	{
		MechanismFactory::m_mechanismFactory = new MechanismFactory();
	}
	return MechanismFactory::m_mechanismFactory;
}

MechanismFactory::MechanismFactory()  : m_balltransfer(),
										m_ballhopper(),
										m_intake1(),
										m_intake2(),
										m_shooter(),
										m_turret()   
{
	Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Intake1"), string("not created"));
	Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Intake2"), string("not created"));
	Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Ball Hopper"), string("not created"));
	Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Ball Transfer"), string("not created"));
	Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Shooter"), string("not created"));
	Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Turret"), string("not created"));
}

/// @brief      create the requested mechanism
/// @param [in] MechanismTypes::MECHANISM_TYPE  type - the type of mechanism to create
/// @param [in] const IDragonMotorControllerMap& map of the motor usage to the motor controller
/// @param [in] 
/// @param [in] 
/// @param [in] 
/// @param [in] 
/// @return  IMech*  pointer to the mechanism or nullptr if mechanism couldn't be created.
void  MechanismFactory::CreateIMechanism
(
	MechanismTypes::MECHANISM_TYPE			type,
	const IDragonMotorControllerMap&        motorControllers,   // <I> - Motor Controllers
	const ServoMap&						    servos,
	const DigitalInputMap&					digitalInputs,
	shared_ptr<CANCoder>					canCoder
)
{
	bool found = false;

    auto nt = nt::NetworkTableInstance::GetDefault().GetTable(string("MechanismFactory"));

	// Create the mechanism
	switch ( type )
	{
		case MechanismTypes::MECHANISM_TYPE::INTAKE:
		{
			auto motor = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE );
			if ( motor.get() != nullptr )
			{
				motor.get()->UpdateFramePeriods(StatusFrameEnhanced::Status_1_General, 120);
				motor.get()->UpdateFramePeriods(StatusFrameEnhanced::Status_1_General, 120);

				if ( m_intake1.get() == nullptr )
				{
					m_intake1 = make_shared<Intake>( motor );
					Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Intake1"), string("created"));
				}
				else if ( m_intake2.get() == nullptr )
				{
					m_intake2 = make_shared<Intake>( motor );
					Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Intake2"), string("created"));
				}
				else
				{
					found = true;
				}
			}
		}
		break;

		case MechanismTypes::MECHANISM_TYPE::BALL_HOPPER:
		{
			if(m_ballhopper.get() == nullptr )
			{
				auto motor = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::BALL_HOPPER );
				auto bannerSensor = GetDigitalInput( digitalInputs, DigitalInputUsage::DIGITAL_SENSOR_USAGE::HOPPER_BANNER_SENSOR );
				if( motor.get() != nullptr && bannerSensor.get() != nullptr)
				{
					motor.get()->UpdateFramePeriods(StatusFrameEnhanced::Status_1_General, 80);
					motor.get()->UpdateFramePeriods(StatusFrameEnhanced::Status_1_General, 120);
		
					m_ballhopper = make_shared<BallHopper>(motor, bannerSensor);
					Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Ball Hopper"), string("created"));
				}
			}
			else
			{
				found = true;
			}
			
		}
		break;					

		case MechanismTypes::MECHANISM_TYPE::BALL_TRANSFER:
		{
			if ( m_balltransfer.get() == nullptr )
			{
				auto motor = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::BALL_TRANSFER );
				if ( motor.get() != nullptr )
				{
					motor.get()->UpdateFramePeriods(StatusFrameEnhanced::Status_1_General, 60);
					motor.get()->UpdateFramePeriods(StatusFrameEnhanced::Status_1_General, 120);
					m_balltransfer = make_shared<BallTransfer>( motor );
					Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Ball Transfer"), string("created"));
				}
			}
			else
			{
				found = true;
			}
		}
		break;

	
		case MechanismTypes::MECHANISM_TYPE::SHOOTER:
		{
			if ( m_shooter.get() == nullptr )
			{
				auto motor1 = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER_1 );
				auto motor2 = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER_2 );
				if ( motor1.get() != nullptr && motor2.get() != nullptr )
				{
					m_shooter = make_shared<Shooter>(motor1, motor2);
					Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Shooter"), string("created"));
				}
			}
			else
			{
				found = true;
			}

		}
		break;		
		
		case MechanismTypes::MECHANISM_TYPE::TURRET:
		{
			if ( m_turret.get() == nullptr )
			{
				auto motor = GetMotorController(motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::TURRET);
				if(motor.get() != nullptr)
				{
					motor.get()->UpdateFramePeriods(StatusFrameEnhanced::Status_1_General, 60);
					m_turret = make_shared<Turret>(motor);
					Logger::GetLogger()->ToNtTable(string("MechanismFactory"), string("Turret"), string("created"));
				}
			}
			else
			{
				found = true;
			}
		}
		break;

		default:
		{
			string msg = "unknown Mechanism type ";
			msg += to_string( type );
			Logger::GetLogger()->LogError( "MechanismFactory::CreateIMechanism", msg );
		}
		break;
	}
    // See if the mechanism was created already, if it wasn't create it
	if ( found )
    {
        string msg = "mechanism already exists";
        msg += to_string( type );
        Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism " ), msg );
    }
}


shared_ptr<IDragonMotorController> MechanismFactory::GetMotorController
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
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetMotorController" ), msg );
	}
	
	if ( motor.get() == nullptr )
	{
		string msg = "motor is nullptr; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetMotorController" ), msg );
	}
	return motor;
}


shared_ptr<DragonServo> MechanismFactory::GetServo
(
	const ServoMap&									servos,
	ServoUsage::SERVO_USAGE							usage
)
{
	shared_ptr<DragonServo> servo;
	auto it = servos.find( usage );
	if ( it != servos.end() )  // found it
	{
		servo = it->second;
	}
	else
	{
		string msg = "servo not found; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetServo" ), msg );
	}
	
	if ( servo.get() == nullptr )
	{
		string msg = "servo is nullptr; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetServo" ), msg );
	}
	return servo;

}
shared_ptr<DragonDigitalInput> MechanismFactory::GetDigitalInput
(
	const DigitalInputMap&							digitaInputs,
	DigitalInputUsage::DIGITAL_SENSOR_USAGE			usage
)
{
	shared_ptr<DragonDigitalInput> dio;
	auto it = digitaInputs.find( usage );
	if ( it != digitaInputs.end() )  // found it
	{
		dio = it->second;
	}
	else
	{
		string msg = "digital input not found; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetDigitalInput" ), msg );
	}
	
	if ( dio.get() == nullptr )
	{
		string msg = "digital input is nullptr; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetDigitalInput" ), msg );
	}
	return dio;
}



