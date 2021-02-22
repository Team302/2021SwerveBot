
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
#include <string>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/SpeedController.h>

// Team 302 includes
#include <hw/DragonTalon.h>
#include <hw/DragonPDP.h>
#include <hw/usages/MotorControllerUsage.h>
#include <utils/ConversionUtils.h>
#include <utils/Logger.h>

// Third Party Includes
#include <ctre/phoenix/motorcontrol/LimitSwitchType.h>


using namespace frc;
using namespace std;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix;


DragonTalon::DragonTalon
(
	MotorControllerUsage::MOTOR_CONTROLLER_USAGE deviceType, 
	int deviceID, 
    int pdpID, 
	int countsPerRev, 
	double gearRatio 
) : m_talon( make_shared<WPI_TalonSRX>(deviceID)),
	m_controlMode(ControlModes::CONTROL_TYPE::PERCENT_OUTPUT),
	m_type(deviceType),
	m_id(deviceID),
	m_pdp( pdpID ),
	m_countsPerRev(countsPerRev),
	m_tickOffset(0),
	m_gearRatio(gearRatio),
	m_diameter( 1.0 )
{
	m_talon->ConfigFactoryDefault();
	//m_tickOffset = m_talon.get()->GetSelectedSensorPosition();
}

double DragonTalon::GetRotations() const
{
	return (ConversionUtils::CountsToRevolutions( (m_talon.get()->GetSelectedSensorPosition()), m_countsPerRev) * m_gearRatio);
}

double DragonTalon::GetRPS() const
{
	return (ConversionUtils::CountsPer100msToRPS( m_talon.get()->GetSelectedSensorVelocity(), m_countsPerRev) * m_gearRatio);
}

void DragonTalon::SetControlMode(ControlModes::CONTROL_TYPE mode)
{ 
	m_controlMode = mode;
}

shared_ptr<SpeedController> DragonTalon::GetSpeedController() const
{
	return m_talon;
}

double DragonTalon::GetCurrent() const
{
	PowerDistributionPanel* pdp = DragonPDP::GetInstance()->GetPDP();
    return ( pdp != nullptr ) ? pdp->GetCurrent( m_pdp ) : 0.0;
}


void DragonTalon::Set(std::shared_ptr<nt::NetworkTable> nt, double value)
{
	auto output = value;
	ctre::phoenix::motorcontrol::ControlMode ctreMode = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;
    switch (m_controlMode)
    {
        case ControlModes::CONTROL_TYPE::PERCENT_OUTPUT:
			ctreMode = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;
			output = value;
			break;
			
			
		case ControlModes::CONTROL_TYPE::VOLTAGE:
			ctreMode = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;
			output = value;
			break;

        case ControlModes::CONTROL_TYPE::POSITION_DEGREES:
			ctreMode = ctre::phoenix::motorcontrol::ControlMode::Position;
			output = (ConversionUtils::DegreesToCounts(value,m_countsPerRev) / m_gearRatio);
			break;

		case ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE:
			ctreMode = ctre::phoenix::motorcontrol::ControlMode::MotionMagic;
			output = value;
			break;

        case ControlModes::CONTROL_TYPE::POSITION_INCH:
            ctreMode = ctre::phoenix::motorcontrol::ControlMode::Position;
			output = (ConversionUtils::ConversionUtils::InchesToCounts(value, m_countsPerRev, m_diameter) / m_gearRatio);
        	break;
        
        case ControlModes::CONTROL_TYPE::VELOCITY_DEGREES:
            ctreMode = ctre::phoenix::motorcontrol::ControlMode::Velocity;
			output = (ConversionUtils::DegreesPerSecondToCounts100ms( value, m_countsPerRev ) / m_gearRatio);
        	break;

        case ControlModes::CONTROL_TYPE::VELOCITY_INCH:
            ctreMode = ctre::phoenix::motorcontrol::ControlMode::Velocity;
			output = (ConversionUtils::InchesPerSecondToCounts100ms( value, m_countsPerRev, m_diameter ) / m_gearRatio);
        	break;

		case ControlModes::CONTROL_TYPE::VELOCITY_RPS:
            ctreMode = ctre::phoenix::motorcontrol::ControlMode::Velocity;
			output = (ConversionUtils::RPSToCounts100ms( value, m_countsPerRev ) / m_gearRatio);
        	break;

		case ControlModes::CONTROL_TYPE::CURRENT:
			ctreMode = ctre::phoenix::motorcontrol::ControlMode::Current;
			output = value;
			break;

		case ControlModes::CONTROL_TYPE::MOTION_PROFILE:
			ctreMode = ctre::phoenix::motorcontrol::ControlMode::MotionProfile;
			output = value;
			break;

		case ControlModes::CONTROL_TYPE::MOTION_PROFILE_ARC:
			ctreMode = ctre::phoenix::motorcontrol::ControlMode::MotionProfileArc;
			output = value;
			break;

		case ControlModes::CONTROL_TYPE::TRAPEZOID:
			ctreMode = ctre::phoenix::motorcontrol::ControlMode::MotionMagic;
			output = (ConversionUtils::InchesToCounts(value, m_countsPerRev, m_diameter) / m_gearRatio);
			break;

        default:
            Logger::GetLogger()->LogError( string("DragonTalon::SetControlMode"), string("Invalid Control Mode"));
        	ctreMode = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;
			output = value;
        	break;
    }	

	if ( nt.get() != nullptr )
	{
		nt->PutString("motor id ", to_string(m_talon.get()->GetDeviceID()) );
		nt->PutNumber("motor output ", output );
	}

	m_talon.get()->Set( ctreMode, output );
}

void DragonTalon::Set(double value)
{
	auto id = m_talon.get()->GetDeviceID();
	auto ntName = std::string("MotorOutput");
	ntName += to_string(id);
	auto nt = nt::NetworkTableInstance::GetDefault().GetTable(ntName);
	Set(nt, value);
}
void DragonTalon::SetRotationOffset(double rotations)
{
//	double newRotations = -rotations + DragonTalon::GetRotations();
//	m_tickOffset += (int) (newRotations * m_countsPerRev / m_gearRatio);
}

void DragonTalon::SetVoltageRamping(double ramping, double rampingClosedLoop)
{
    m_talon.get()->ConfigOpenloopRamp(ramping);

	if (rampingClosedLoop >= 0)
	{
		m_talon.get()->ConfigClosedloopRamp(rampingClosedLoop);
	}
}


void DragonTalon::EnableCurrentLimiting(bool enabled)
{
    m_talon.get()->EnableCurrentLimit(enabled);
}

void DragonTalon::EnableBrakeMode(bool enabled)
{
    m_talon.get()->SetNeutralMode(enabled ? ctre::phoenix::motorcontrol::NeutralMode::Brake : ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void DragonTalon::Invert(bool inverted)
{
    m_talon.get()->SetInverted(inverted);
}

void DragonTalon::SetSensorInverted(bool inverted)
{
    m_talon.get()->SetSensorPhase(inverted);
}

MotorControllerUsage::MOTOR_CONTROLLER_USAGE DragonTalon::GetType() const
{
	return m_type;
}

int DragonTalon::GetID() const
{
	return m_id;
}

//------------------------------------------------------------------------------
// Method:		SelectClosedLoopProfile
// Description:	Selects which profile slot to use for closed-loop control
// Returns:		void
//------------------------------------------------------------------------------
void DragonTalon::SelectClosedLoopProfile
(
	int	   slot,			// <I> - profile slot to select
	int    pidIndex			// <I> - 0 for primary closed loop, 1 for cascaded closed-loop
)
{
	m_talon.get()->SelectProfileSlot( slot, pidIndex );
}


int DragonTalon::ConfigSelectedFeedbackSensor
(
	FeedbackDevice feedbackDevice,
	int pidIdx,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigSelectedFeedbackSensor( feedbackDevice, pidIdx, timeoutMs );
	}
	else
	{
//		printf( "DragonTalon::ConfigSelectedFeedbackSensor: m_talon.get() not initialized \n" );
	}
	return error;
}

int DragonTalon::ConfigSelectedFeedbackSensor
(
	RemoteFeedbackDevice feedbackDevice,
	int pidIdx,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigSelectedFeedbackSensor( feedbackDevice, pidIdx, timeoutMs );
	}
	else
	{
//		printf( "DragonTalon::ConfigSelectedFeedbackSensor: m_talon.get() not initialized \n" );
	}
	return error;
}

int DragonTalon::ConfigPeakCurrentLimit
(
	int amps,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigPeakCurrentLimit( amps, timeoutMs );
	}
	else
	{
//		printf( "DragonTalon::ConfigPeakCurrentLimit: m_talon.get() not initialized \n" );
	}
	return error;
}

int DragonTalon::ConfigPeakCurrentDuration
(
	int milliseconds,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigPeakCurrentDuration( milliseconds, timeoutMs );
	}
	else
	{
//		printf( "DragonTalon::ConfigPeakCurrentDuration: m_talon.get() not initialized \n" );
	}
	return error;
}

int DragonTalon::ConfigContinuousCurrentLimit
(
	int amps,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		error = m_talon.get()->ConfigContinuousCurrentLimit( amps, timeoutMs );
	}
	else
	{
//		printf( "DragonTalon::ConfigContinuousCurrentLimit: m_talon.get() not initialized \n" );
	}
	return error;
}

void DragonTalon::SetAsFollowerMotor
(
    int         masterCANID         // <I> - master motor
)
{
    m_talon.get()->Set( ControlMode::Follower, masterCANID );
}


/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] int             slot - hardware slot to use
/// @param [in] ControlData*    pid - the control constants
/// @return void
void DragonTalon::SetControlConstants(int slot, ControlData* controlInfo)
{
	SetControlMode(controlInfo->GetMode());

	auto id = m_talon.get()->GetDeviceID();
	auto ntName = std::string("MotorOutput");
	ntName += to_string(id);
	Logger::GetLogger()->ToNtTable(ntName, string("P"), to_string(controlInfo->GetP()));
	Logger::GetLogger()->ToNtTable(ntName, string("I"), to_string(controlInfo->GetI()));
	Logger::GetLogger()->ToNtTable(ntName, string("D"), to_string(controlInfo->GetD()));
	Logger::GetLogger()->ToNtTable(ntName, string("F"), to_string(controlInfo->GetF()));

	auto peak = controlInfo->GetPeakValue();
	auto error = m_talon.get()->ConfigPeakOutputForward(peak);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogError(string("DragonTalon"), string("ConfigPeakOutputForward error"));
	}
	error = m_talon.get()->ConfigPeakOutputReverse(-1.0*peak);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogError(string("DragonTalon"), string("ConfigPeakOutputReverse error"));
	}

	auto nom = controlInfo->GetNominalValue();
	error = m_talon.get()->ConfigNominalOutputForward(nom);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogError(string("DragonTalon"), string("ConfigNominalOutputForward error"));
	}
	error = m_talon.get()->ConfigNominalOutputReverse(-1.0*nom);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogError(string("DragonTalon"), string("ConfigNominalOutputReverse error"));
	}

	if ( controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_ABSOLUTE ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_DEGREES ||
	     controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_INCH ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::VELOCITY_DEGREES ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::VELOCITY_INCH ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::VELOCITY_RPS  ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::VOLTAGE ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::CURRENT ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::TRAPEZOID )
	{
		error = m_talon.get()->Config_kP(slot, controlInfo->GetP());
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogError(string("DragonTalon"), string("Config_kP error"));
		}
		error = m_talon.get()->Config_kI(slot, controlInfo->GetI());
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogError(string("DragonTalon"), string("Config_kI error"));
		}
		error = m_talon.get()->Config_kD(slot, controlInfo->GetD());
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogError(string("DragonTalon"), string("Config_kD error"));
		}
		error = m_talon.get()->Config_kF(slot, controlInfo->GetF());
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogError(string("DragonTalon"), string("Config_kF error"));
		}
		error = m_talon.get()->SelectProfileSlot(slot, 0);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogError(string("DragonTalon"), string("SelectProfileSlot error"));
		}
	}

	
	if ( //controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_ABSOLUTE ||
		 controlInfo->GetMode() == ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE ||
	     controlInfo->GetMode() == ControlModes::CONTROL_TYPE::TRAPEZOID  )
	{
		error = m_talon.get()->ConfigMotionAcceleration( controlInfo->GetMaxAcceleration() );
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogError(string("DragonTalon"), string("ConfigMotionAcceleration error"));
		}
		error = m_talon.get()->ConfigMotionCruiseVelocity( controlInfo->GetCruiseVelocity(), 0);
		if ( error != ErrorCode::OKAY )
		{
			Logger::GetLogger()->LogError(string("DragonTalon"), string("ConfigMotionCruiseVelocity error"));
		}

	}
}

void DragonTalon::SetForwardLimitSwitch
( 
	bool normallyOpen
)
{
	LimitSwitchNormal type = normallyOpen ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen : LimitSwitchNormal::LimitSwitchNormal_NormallyClosed;
	m_talon.get()->ConfigForwardLimitSwitchSource( LimitSwitchSource::LimitSwitchSource_FeedbackConnector, type, 0  );
}

void DragonTalon::SetReverseLimitSwitch
(
	bool normallyOpen
)
{
	LimitSwitchNormal type = normallyOpen ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen : LimitSwitchNormal::LimitSwitchNormal_NormallyClosed;
	m_talon.get()->ConfigReverseLimitSwitchSource( LimitSwitchSource::LimitSwitchSource_FeedbackConnector, type, 0  );
}


void DragonTalon::SetRemoteSensor
(
    int                                             canID,
    ctre::phoenix::motorcontrol::RemoteSensorSource deviceType
)
{
	m_talon.get()->ConfigRemoteFeedbackFilter( canID, deviceType, 0, 0.0 );
	m_talon.get()->ConfigSelectedFeedbackSensor( RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor0, 0, 0 );
}

void DragonTalon::SetDiameter
(
	double 	diameter
)
{
	m_diameter = diameter;
}