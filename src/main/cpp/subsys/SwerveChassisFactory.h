
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

#pragma once

// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <hw/usages/IDragonMotorControllerMap.h>
#include <subsys/DragonSwerveModule.h>
#include <subsys/SwerveChassis.h>


// Third Party Includes

// forward declares
class IDragonMotorController;

namespace ctre
{
	namespace phoenix
	{
		namespace sensors
		{
			class CANCoder;
		}
	}
}

class SwerveChassisFactory
{
	public:

		static SwerveChassisFactory* GetSwerveChassisFactory();


		//=====================================================================================
		/// Method:         CreateSwerveModule
		/// Description:    Find or create the swerve module
		/// Returns:        DragonSwerveModule *    pointer to the swerve module or nullptr if it 
		///                                         doesn't exist and cannot be created.
		//=====================================================================================
		std::shared_ptr<DragonSwerveModule> CreateSwerveModule
		(
            DragonSwerveModule::ModuleID                            type,
			const IDragonMotorControllerMap&        				motorControllers,   // <I> - Motor Controllers
			std::shared_ptr<ctre::phoenix::sensors::CANCoder>		turnSensor,
			units::degree_t											turnOffset
		);

		//=====================================================================================
		/// Method:         CreateSwerveChassis
		/// Description:    Find or create the swerve chassis
		/// Returns:        SwerveChassis*      pointer to the swerve chassis or nullptr if it 
		///                                     doesn't exist and cannot be created.
		//=====================================================================================
		std::shared_ptr<SwerveChassis> CreateSwerveChassis
		(
            std::shared_ptr<DragonSwerveModule> frontleft, 
            std::shared_ptr<DragonSwerveModule> frontright,
            std::shared_ptr<DragonSwerveModule> backleft, 
            std::shared_ptr<DragonSwerveModule> backright,
			units::length::inch_t wheelDiameter,
			units::length::inch_t wheelBase,
			units::length::inch_t track,
            units::velocity::meters_per_second_t maxSpeed,
			double maxAcceleration
		);

        std::shared_ptr<SwerveChassis> GetSwerveChassis() { return m_chassis; }
        std::shared_ptr<DragonSwerveModule>	GetLeftFrontSwerveModule() { return m_leftFront; }
        std::shared_ptr<DragonSwerveModule> GetLeftBackSwerveModule() { return m_leftBack; }
        std::shared_ptr<DragonSwerveModule>	GetRightFrontSwerveModule() { return m_rightFront; }
        std::shared_ptr<DragonSwerveModule>	GetRightBackSwerveModule() { return m_rightBack; }

	private:
		std::shared_ptr<IDragonMotorController> GetMotorController
		(
			const IDragonMotorControllerMap&				motorControllers,
			MotorControllerUsage::MOTOR_CONTROLLER_USAGE	usage
		);


		SwerveChassisFactory() = default;
		virtual ~SwerveChassisFactory() = default;

		static SwerveChassisFactory*	        m_swerveChassisFactory;

        std::shared_ptr<SwerveChassis>			m_chassis;
        std::shared_ptr<DragonSwerveModule>	    m_leftFront;
        std::shared_ptr<DragonSwerveModule>	    m_leftBack;
        std::shared_ptr<DragonSwerveModule>	    m_rightFront;
        std::shared_ptr<DragonSwerveModule>	    m_rightBack;



};