
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
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/SpeedController.h>

// Team 302 includes
#include <hw/DragonFalcon.h>
#include <hw/DragonPDP.h>
#include <hw/usages/MotorControllerUsage.h>
#include <utils/Logger.h>
#include <utils/ConversionUtils.h>

// Third Party Includes
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>


using namespace frc;
using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

DragonFalcon::DragonFalcon
(
	MotorControllerUsage::MOTOR_CONTROLLER_USAGE deviceType, 
	int deviceID, 
    int pdpID, 
	int countsPerRev, 
	double gearRatio 
) : m_talon( make_shared<WPI_TalonFX>(deviceID)),
	m_controlMode(ControlModes::CONTROL_TYPE::PERCENT_OUTPUT),
	m_type(deviceType),
	m_id(deviceID),
	m_pdp( pdpID ),
	m_countsPerRev(countsPerRev),
	m_tickOffset(0),
	m_gearRatio(gearRatio),
	m_diameter( 1.0 )
{
	m_talon.get()->ConfigFactoryDefault();

	m_talon.get()->ConfigNeutralDeadband(0.01, 0);
	m_talon.get()->SetNeutralMode(NeutralMode::Brake);
	m_talon.get()->ConfigNominalOutputForward(0.0, 10);
	m_talon.get()->ConfigNominalOutputReverse(0.0, 10);
	m_talon.get()->ConfigOpenloopRamp(1.0);
	m_talon.get()->ConfigPeakOutputForward(1.0, 10);
	m_talon.get()->ConfigPeakOutputReverse(-1.0, 10);

	SupplyCurrentLimitConfiguration climit;
	climit.enable = false;
	climit.currentLimit = 1.0;
	climit.triggerThresholdCurrent = 1.0;
	climit.triggerThresholdTime = 0.001;
	m_talon.get()->ConfigSupplyCurrentLimit(climit, 50);

	StatorCurrentLimitConfiguration climit2;
	climit2.enable = false;
	climit2.currentLimit = 1.0;
	climit2.triggerThresholdCurrent = 1.0;
	climit2.triggerThresholdTime = 0.001;
	m_talon.get()->ConfigStatorCurrentLimit( climit2, 50);

	m_talon.get()->ConfigVoltageCompSaturation(12.0, 0);

	m_talon.get()->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_Disabled, 0);
	m_talon.get()->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_Disabled, 0);

	m_talon.get()->ConfigForwardSoftLimitEnable(false, 0);
	m_talon.get()->ConfigForwardSoftLimitThreshold(0.0, 0);

	m_talon.get()->ConfigReverseSoftLimitEnable(false, 0);
	m_talon.get()->ConfigReverseSoftLimitThreshold(0.0, 0);
	
	
	m_talon.get()->ConfigMotionAcceleration(1500.0, 10);
	m_talon.get()->ConfigMotionCruiseVelocity(1500.0, 10);
	m_talon.get()->ConfigMotionSCurveStrength(0, 0);

	m_talon.get()->ConfigMotionProfileTrajectoryPeriod(0, 0);
	m_talon.get()->ConfigMotionProfileTrajectoryInterpolationEnable(true, 0);

	m_talon.get()->ConfigAllowableClosedloopError(0.0, 0);

	for ( auto inx=0; inx<4; ++inx )
	{
		m_talon.get()->ConfigClosedLoopPeakOutput(0, 1.0, 10);
		m_talon.get()->ConfigClosedLoopPeriod(0, 1, 10 );
		m_talon.get()->Config_kP(0, 0.01, 0);
		m_talon.get()->Config_kI(0, 0.0, 0);
		m_talon.get()->Config_kD(0, 0.0, 0);
		m_talon.get()->Config_kF(0, 1.0, 0);
		m_talon.get()->Config_IntegralZone(0, 0.0, 0);
	}

	m_talon.get()->ConfigRemoteFeedbackFilter(60, RemoteSensorSource::RemoteSensorSource_Off, 0, 0 );
	m_talon.get()->ConfigRemoteFeedbackFilter(60, RemoteSensorSource::RemoteSensorSource_Off, 1, 0 );

	m_talon.get()->ConfigVoltageCompSaturation(12.0, 0);

	m_talon.get()->ConfigForwardSoftLimitEnable(false, 0);
	m_talon.get()->ConfigReverseSoftLimitEnable(false, 0);

	/**
	TalonFXConfiguration configs;
	m_talon.get()->GetAllConfigs(configs, 50);
	string key = string("talonFx") + to_string(deviceID);
	Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::PRINT_ONCE, key, configs.toString());
	**/
}

double DragonFalcon::GetRotations() const
{
	return (ConversionUtils::CountsToRevolutions( (m_talon.get()->GetSelectedSensorPosition()), m_countsPerRev) / m_gearRatio);
}

double DragonFalcon::GetRPS() const
{
	return (ConversionUtils::CountsPer100msToRPS( m_talon.get()->GetSelectedSensorVelocity(), m_countsPerRev) * m_gearRatio);
}

void DragonFalcon::SetControlMode(ControlModes::CONTROL_TYPE mode)
{ 
	m_controlMode = mode;
}

shared_ptr<SpeedController> DragonFalcon::GetSpeedController() const
{
	return m_talon;
}

double DragonFalcon::GetCurrent() const
{
	PowerDistributionPanel* pdp = DragonPDP::GetInstance()->GetPDP();
    return ( pdp != nullptr ) ? pdp->GetCurrent( m_pdp ) : 0.0;
}


void DragonFalcon::Set(std::shared_ptr<nt::NetworkTable> nt, double value)
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
			ctreMode =:: ctre::phoenix::motorcontrol::ControlMode::Position;
			output = (ConversionUtils::DegreesToCounts(value,m_countsPerRev) / m_gearRatio);
			break;

        case ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE:
			ctreMode =:: ctre::phoenix::motorcontrol::ControlMode::MotionMagic;
			output = value;
			break;

        case ControlModes::CONTROL_TYPE::POSITION_INCH:
            ctreMode = ctre::phoenix::motorcontrol::ControlMode::Position;
			output = (ConversionUtils::InchesToCounts(value, m_countsPerRev, m_diameter) / m_gearRatio);
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
			output = value / m_gearRatio;
			output = ConversionUtils::RPSToCounts100ms( output, m_countsPerRev );
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
            Logger::GetLogger()->LogError( string("DragonFalcon::SetControlMode"), string("Invalid Control Mode"));
        	ctreMode = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;
			output = value;
        	break;
    }	

	if ( nt.get() != nullptr )
	{
		nt->PutString("motor id ", to_string(m_talon.get()->GetDeviceID()) );
		nt->PutNumber("motor output ", output );
	}

	m_talon.get()->Set( ctreMode, output );}

void DragonFalcon::Set(double value)
{
	auto id = m_talon.get()->GetDeviceID();
	auto ntName = std::string("MotorOutput");
	ntName += to_string(id);
	auto nt = nt::NetworkTableInstance::GetDefault().GetTable(ntName);
	Set(nt, value);
}

void DragonFalcon::SetRotationOffset(double rotations)
{
//	double newRotations = -rotations + DragonFalcon::GetRotations();
//	m_tickOffset += (int) (newRotations * m_countsPerRev / m_gearRatio);
}

void DragonFalcon::SetVoltageRamping(double ramping, double rampingClosedLoop)
{
    m_talon.get()->ConfigOpenloopRamp(ramping);

	if (rampingClosedLoop >= 0)
	{
		m_talon.get()->ConfigClosedloopRamp(rampingClosedLoop);
	}
}


void DragonFalcon::EnableCurrentLimiting(bool enabled)
{
		SupplyCurrentLimitConfiguration limit;
		int timeout = 50.0;
		m_talon.get()->ConfigGetSupplyCurrentLimit( limit, timeout );
		limit.enable = enabled;
		m_talon.get()->ConfigSupplyCurrentLimit( limit, timeout );
}

void DragonFalcon::EnableBrakeMode(bool enabled)
{
    m_talon.get()->SetNeutralMode(enabled ? ctre::phoenix::motorcontrol::NeutralMode::Brake : ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void DragonFalcon::Invert(bool inverted)
{
    m_talon.get()->SetInverted(inverted);
}

void DragonFalcon::SetSensorInverted(bool inverted)
{
    m_talon.get()->SetSensorPhase(inverted);
}

MotorControllerUsage::MOTOR_CONTROLLER_USAGE DragonFalcon::GetType() const
{
	return m_type;
}

int DragonFalcon::GetID() const
{
	return m_id;
}

//------------------------------------------------------------------------------
// Method:		SelectClosedLoopProfile
// Description:	Selects which profile slot to use for closed-loop control
// Returns:		void
//------------------------------------------------------------------------------
void DragonFalcon::SelectClosedLoopProfile
(
	int	   slot,			// <I> - profile slot to select
	int    pidIndex			// <I> - 0 for primary closed loop, 1 for cascaded closed-loop
)
{
	m_talon.get()->SelectProfileSlot( slot, pidIndex );
}

int DragonFalcon::ConfigSelectedFeedbackSensor
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
        Logger::GetLogger()->LogError( string("DragonFalcon::ConfigSelectedFeedbackSensor"), string("m_talon is a nullptr"));
	}
	return error;
}

int DragonFalcon::ConfigSelectedFeedbackSensor
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
        Logger::GetLogger()->LogError( string("DragonFalcon::ConfigSelectedFeedbackSensor"), string("m_talon is a nullptr"));
	}
	return error;
}

int DragonFalcon::ConfigPeakCurrentLimit
(
	int amps,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		SupplyCurrentLimitConfiguration limit;
		m_talon.get()->ConfigGetSupplyCurrentLimit( limit, timeoutMs );
		limit.triggerThresholdCurrent = amps;
		m_talon.get()->ConfigSupplyCurrentLimit( limit, timeoutMs );
	}
	else
	{
        Logger::GetLogger()->LogError( string("DragonFalcon::ConfigPeakCurrentLimit"), string("m_talon is a nullptr"));
	}
	return error;
}

int DragonFalcon::ConfigPeakCurrentDuration
(
	int milliseconds,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		SupplyCurrentLimitConfiguration limit;
		m_talon.get()->ConfigGetSupplyCurrentLimit( limit, timeoutMs );
		limit.triggerThresholdTime = milliseconds;
		m_talon.get()->ConfigSupplyCurrentLimit( limit, timeoutMs );
	}
	else
	{
        Logger::GetLogger()->LogError( string("DragonFalcon::ConfigPeakCurrentDuration"), string("m_talon is a nullptr"));
	}
	return error;
}

int DragonFalcon::ConfigContinuousCurrentLimit
(
	int amps,
	int timeoutMs
)
{
	int error = 0;
	if ( m_talon.get() != nullptr )
	{
		SupplyCurrentLimitConfiguration limit;
		m_talon.get()->ConfigGetSupplyCurrentLimit( limit, timeoutMs );
		limit.currentLimit = amps;
		m_talon.get()->ConfigSupplyCurrentLimit( limit, timeoutMs );
	}
	else
	{
        Logger::GetLogger()->LogError( string("DragonFalcon::ConfigContinuousCurrentLimit"), string("m_talon is a nullptr"));
	}
	return error;
}

void DragonFalcon::SetAsFollowerMotor
(
    int         masterCANID         // <I> - master motor
)
{
    m_talon.get()->Set( ControlMode::Follower, masterCANID );
}


/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData*   pid - the control constants
/// @return void
void DragonFalcon::SetControlConstants(ControlData* controlInfo)
{
	SetControlMode(controlInfo->GetMode());

	auto peak = controlInfo->GetPeakValue();
	m_talon.get()->ConfigPeakOutputForward(peak);
	m_talon.get()->ConfigPeakOutputReverse(-1.0*peak);

	auto nom = controlInfo->GetNominalValue();
	m_talon.get()->ConfigPeakOutputForward(nom);
	m_talon.get()->ConfigPeakOutputReverse(-1.0*nom);

	switch ( controlInfo->GetMode() )
	{
		case ControlModes::CONTROL_TYPE::PERCENT_OUTPUT:
		{
		}
		break;

		case ControlModes::CONTROL_TYPE::POSITION_DEGREES:
		{
			m_talon.get()->Config_kP(0, controlInfo->GetP());
			m_talon.get()->Config_kI(0, controlInfo->GetI());
			m_talon.get()->Config_kD(0, controlInfo->GetD());
			m_talon.get()->Config_kF(0, controlInfo->GetF());
			m_talon.get()->SelectProfileSlot(0, 0);
		}
		break;

		case ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE:
		{
			m_talon.get()->Config_kP(0, controlInfo->GetP());
			m_talon.get()->Config_kI(0, controlInfo->GetI());
			m_talon.get()->Config_kD(0, controlInfo->GetD());
			m_talon.get()->Config_kF(0, controlInfo->GetF());
			m_talon.get()->ConfigMotionAcceleration( controlInfo->GetMaxAcceleration() );
			m_talon.get()->ConfigMotionCruiseVelocity( controlInfo->GetCruiseVelocity(), 0);
			m_talon.get()->SelectProfileSlot(0, 0);
		}
		break;

		case ControlModes::CONTROL_TYPE::POSITION_INCH:
		{
			m_talon.get()->Config_kP(0, controlInfo->GetP());
			m_talon.get()->Config_kI(0, controlInfo->GetI());
			m_talon.get()->Config_kD(0, controlInfo->GetD());
			m_talon.get()->Config_kF(0, controlInfo->GetF());
			m_talon.get()->SelectProfileSlot(0, 0);
		}
		break;

		case ControlModes::CONTROL_TYPE::VELOCITY_DEGREES:
		{
			m_talon.get()->Config_kP(0, controlInfo->GetP());
			m_talon.get()->Config_kI(0, controlInfo->GetI());
			m_talon.get()->Config_kD(0, controlInfo->GetD());
			m_talon.get()->Config_kF(0, controlInfo->GetF());
			m_talon.get()->SelectProfileSlot(0, 0);
		}
		break;

		case ControlModes::CONTROL_TYPE::VELOCITY_INCH:
		{
			m_talon.get()->Config_kP(0, controlInfo->GetP());
			m_talon.get()->Config_kI(0, controlInfo->GetI());
			m_talon.get()->Config_kD(0, controlInfo->GetD());
			m_talon.get()->Config_kF(0, controlInfo->GetF());
			m_talon.get()->SelectProfileSlot(0, 0);
		}
		break;

		case ControlModes::CONTROL_TYPE::VELOCITY_RPS:
		{
			m_talon.get()->Config_kP(0, controlInfo->GetP());
			m_talon.get()->Config_kI(0, controlInfo->GetI());
			m_talon.get()->Config_kD(0, controlInfo->GetD());
			m_talon.get()->Config_kF(0, controlInfo->GetF());
			m_talon.get()->SelectProfileSlot(0, 0);
		}
		break;

		case ControlModes::CONTROL_TYPE::VOLTAGE:
		{
			m_talon.get()->Config_kP(0, controlInfo->GetP());
			m_talon.get()->Config_kI(0, controlInfo->GetI());
			m_talon.get()->Config_kD(0, controlInfo->GetD());
			m_talon.get()->Config_kF(0, controlInfo->GetF());
			m_talon.get()->SelectProfileSlot(0, 0);
		}
		break;

		case ControlModes::CONTROL_TYPE::CURRENT:
		{
			m_talon.get()->Config_kP(0, controlInfo->GetP());
			m_talon.get()->Config_kI(0, controlInfo->GetI());
			m_talon.get()->Config_kD(0, controlInfo->GetD());
			m_talon.get()->Config_kF(0, controlInfo->GetF());
			m_talon.get()->SelectProfileSlot(0, 0);
		}
		break;

		case ControlModes::CONTROL_TYPE::TRAPEZOID:
		{
			m_talon.get()->Config_kP(0, controlInfo->GetP());
			m_talon.get()->Config_kI(0, controlInfo->GetI());
			m_talon.get()->Config_kD(0, controlInfo->GetD());
			m_talon.get()->Config_kF(0, controlInfo->GetF());

			auto accel = controlInfo->GetMaxAcceleration() / m_gearRatio;
			m_talon.get()->ConfigMotionAcceleration( accel );

			auto vel = controlInfo->GetCruiseVelocity() / m_gearRatio;
			m_talon.get()->ConfigMotionCruiseVelocity( vel, 0);
			m_talon.get()->SelectProfileSlot(0, 0);
		}
		break;

		case ControlModes::CONTROL_TYPE::MOTION_PROFILE:
		{

		}
		break;

		case ControlModes::CONTROL_TYPE::MOTION_PROFILE_ARC:
		{

		}
		break;

		default:
		{

		}
		break;
	}
	/**
	TalonFXPIDSetConfiguration  configs;
	m_talon.get()->GetPIDConfigs(configs, 50);
	string key = string("talonFx") + to_string(GetID()) + string(" - PIDS ");
	Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::PRINT_ONCE, key, configs.toString());
	**/

}

void DragonFalcon::SetForwardLimitSwitch
( 
	bool normallyOpen
)
{
	LimitSwitchNormal type = normallyOpen ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen : LimitSwitchNormal::LimitSwitchNormal_NormallyClosed;
	m_talon.get()->ConfigForwardLimitSwitchSource( LimitSwitchSource::LimitSwitchSource_FeedbackConnector, type, 0  );
}

void DragonFalcon::SetReverseLimitSwitch
(
	bool normallyOpen
)
{
	LimitSwitchNormal type = normallyOpen ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen : LimitSwitchNormal::LimitSwitchNormal_NormallyClosed;
	m_talon.get()->ConfigReverseLimitSwitchSource( LimitSwitchSource::LimitSwitchSource_FeedbackConnector, type, 0  );
}

void DragonFalcon::SetRemoteSensor
(
    int                                             canID,
    ctre::phoenix::motorcontrol::RemoteSensorSource deviceType
)
{
	m_talon.get()->ConfigRemoteFeedbackFilter( canID, deviceType, 0, 0.0 );
	m_talon.get()->ConfigSelectedFeedbackSensor( RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor0, 0, 0 );
}

void DragonFalcon::SetDiameter
(
	double 	diameter
)
{
	m_diameter = diameter;
}

void DragonFalcon::SetVoltage
(
	units::volt_t output
)
{
	m_talon -> SetVoltage(output);
}