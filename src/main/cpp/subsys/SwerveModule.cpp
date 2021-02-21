
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
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <wpi/math>

// Team 302 includes
#include <controllers/ControlData.h>
#include <controllers/ControlModes.h>

#include <subsys/SwerveChassis.h>
#include <subsys/SwerveChassisFactory.h>
#include <subsys/SwerveModule.h>
#include <utils/Logger.h>

// Third Party Includes
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/sensors/CANCoder.h>


using namespace std;
using namespace frc;
using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::sensors;
using namespace wpi::math;

/// @brief Constructs a Swerve Module.  This is assuming 2 TalonFX (Falcons) with a CanCoder for the turn angle
/// @param [in] ModuleID                                                type:           Which Swerve Module is it
/// @param [in] shared_ptr<IDragonMotorController>                      driveMotor:     Motor that makes the robot move  
/// @param [in] shared_ptr<IDragonMotorController>                      turnMotor:      Motor that turns the swerve module 
/// @param [in] std::shared_ptr<ctre::phoenix::sensors::CANCoder>		canCoder:       Sensor for detecting the angle of the wheel
/// @param [in] units::length::inch_t                                   wheelDiameter   Diameter of the wheel
SwerveModule::SwerveModule
(
    ModuleID                                                    type, 
    shared_ptr<IDragonMotorController>                          driveMotor, 
    shared_ptr<IDragonMotorController>                          turnMotor, 
    std::shared_ptr<ctre::phoenix::sensors::CANCoder>		    canCoder,
    units::length::inch_t                                       wheelDiameter,
    double                                                      turnP,
    double                                                      turnI,
    double                                                      turnD,
    double                                                      turnF,
    double                                                      turnNominalVal,
    double                                                      turnPeakVal,
    double                                                      turnMaxAcc,
    double                                                      turnCruiseVel
) : m_type(type), 
    m_driveMotor(driveMotor), 
    m_turnMotor(turnMotor), 
    m_turnSensor(canCoder), 
    m_wheelDiameter(wheelDiameter),
    m_initialAngle(canCoder.get()->GetAbsolutePosition()),
    m_initialCounts(0),
    m_nt(),
    m_currentState()
{
    Rotation2d ang { units::angle::degree_t(0.0)};
    m_currentState.angle = ang;
    m_currentState.speed = 0_mps;
    
    // Set up the Drive Motor
    auto motor = m_driveMotor.get()->GetSpeedController();
    auto fx = dynamic_cast<WPI_TalonFX*>(motor.get());
    fx->ConfigSelectedFeedbackSensor( ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 10 );
    fx->ConfigIntegratedSensorInitializationStrategy(BootToZero);
    auto driveMotorSensors = fx->GetSensorCollection();
    driveMotorSensors.SetIntegratedSensorPosition(0, 0);

    // Set up the Absolute Turn Sensor
    m_turnSensor.get()->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180, 0);
    m_turnSensor.get()->GetAbsolutePosition();
    
    
    // Set up the Turn Motor
    motor = m_turnMotor.get()->GetSpeedController();
    fx = dynamic_cast<WPI_TalonFX*>(motor.get());
    auto error = fx->ConfigPeakOutputForward(0.2, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogError(string("SwerveModule"), string("ConfigPeakOutputForward error"));
		error = ErrorCode::OKAY;
	}
	error = fx->ConfigPeakOutputReverse(-0.2, 0);
	if ( error != ErrorCode::OKAY )
	{
		Logger::GetLogger()->LogError(string("SwerveModule"), string("ConfigPeakOutputReverse error"));
		error = ErrorCode::OKAY;
	}

    fx->ConfigSelectedFeedbackSensor( ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 10 );
    fx->ConfigIntegratedSensorInitializationStrategy(BootToZero);
    auto turnMotorSensors = fx->GetSensorCollection();
    turnMotorSensors.SetIntegratedSensorPosition(0, 0);
    auto turnCData = make_shared<ControlData>(  ControlModes::CONTROL_TYPE::POSITION_ABSOLUTE,
                                                ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
                                                string("Turn Angle"),
                                                turnP,
                                                turnI,
                                                turnD,
                                                turnF,
                                                0.0,
                                                turnMaxAcc,
                                                turnCruiseVel,
                                                turnPeakVal,
                                                turnNominalVal);
    m_turnMotor.get()->SetControlConstants( turnCData.get() );

    string ntName;
    switch ( GetType() )
    {
        case ModuleID::LEFT_FRONT:
            ntName = "LeftFrontSwerveModule";
            break;

        case ModuleID::LEFT_BACK:
            ntName = "LeftBackSwerveModule";
            break;

        case ModuleID::RIGHT_FRONT:
            ntName = "RightFrontSwerveModule";
            break;

        case ModuleID::RIGHT_BACK:
            ntName = "RightBackSwerveModule";
            break;

        default:
            Logger::GetLogger()->LogError( Logger::LOGGER_LEVEL::ERROR_ONCE, string("SwerveModuleDrive"), string("unknown module"));
            ntName = "UnknownSwerveModule";
            break;
    }
    m_nt = nt::NetworkTableInstance::GetDefault().GetTable(ntName);
}

void SwerveModule::Init
(
    units::velocity::meters_per_second_t                        maxVelocity,
    units::angular_velocity::radians_per_second_t               maxAngularVelocity,
    units::acceleration::meters_per_second_squared_t            maxAcceleration,
    units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration
)
{
    auto driveCData = make_shared<ControlData>( ControlModes::CONTROL_TYPE::VELOCITY_RPS,
                                                ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
                                                string("DriveSpeed"),
                                                0.01,
                                                0.0,
                                                0.0,
                                                0.5,
                                                0.0,
                                                maxAcceleration.to<double>(),
                                                maxVelocity.to<double>(),
                                                maxVelocity.to<double>(),
                                                0.0 );
    m_driveMotor.get()->SetControlConstants( driveCData.get() );
}

/// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
/// @returns void
void SwerveModule::ZeroAlignModule()
{
    return;
    // Desired State
    units::velocity::meters_per_second_t mps = 0_mps;
    Rotation2d angle {units::angle::degree_t(0.0)};
    SwerveModuleState desiredState{mps,angle};

    // Optimize based on current wheel positions
    Rotation2d currAngle = Rotation2d(units::angle::degree_t(m_turnSensor.get()->GetAbsolutePosition()));
    auto state = SwerveModuleState::Optimize(desiredState, currAngle);
    SetTurnAngle(state.angle.Degrees());
}


/// @brief Get the current state of the module (speed of the wheel and angle of the wheel)
/// @returns SwerveModuleState
SwerveModuleState SwerveModule::GetState() const 
{
    // Get the Current Chassis Velocity
    auto chassis = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis();
    units::velocity::meters_per_second_t mps = units::length::meter_t(GetWheelDiameter() * pi ) / units::angle::radian_t(1.0) * units::angular_velocity::radians_per_second_t(m_driveMotor->GetRPS()*2.0*pi);

    // Get the Current Chassis Rotation
    Rotation2d angle {units::angle::degree_t(m_turnSensor.get()->GetAbsolutePosition())};

    // Create the state and return it
    SwerveModuleState state{mps,angle};
    return state;
}


/// @brief Set the current state of the module (speed of the wheel and angle of the wheel)
/// @param [in] const SwerveModuleState& referenceState:   state to set the module to
/// @returns void
void SwerveModule::SetDesiredState
(
    const SwerveModuleState& referenceState
)
{
    // Update targets so the angle turned is less than 90 degrees
    // If the desired angle is less than 90 degrees from the target angle (e.g., -90 to 90 is the amount of turn), just use the angle and speed values
    // if it is more than 90 degrees (90 to 270), the can turn the opposite direction -- increase the angle by 180 degrees -- and negate the wheel speed
    // finally, get the value between -90 and 90
    Rotation2d currAngle = Rotation2d(units::angle::degree_t(m_turnSensor.get()->GetAbsolutePosition()));
    auto state = SwerveModuleState::Optimize(referenceState, currAngle);
    //auto state = referenceState;
    auto wheelSpeed = state.speed;
    auto delta = state.angle.Degrees() - currAngle.Degrees();
    if ( abs(delta.to<double>() > 90.0 ))
    {
        if ( delta.to<double>() > 0.0 ) // 90+ degree turn
        {
            delta = delta - units::angle::degree_t(180.0);
            wheelSpeed *= -1.0;
        }
        else // -90- degree turn
        {
            delta = delta + units::angle::degree_t(180.0);
            wheelSpeed *= -1.0;
        }
    }

    // Set Drive Target 
    SetDriveSpeed(wheelSpeed);

    // Set Turn Target 
    units::angle::degree_t targetAngle = currAngle.Degrees() + delta;
    SetTurnAngle(targetAngle);
}

void SwerveModule::RunCurrentState()
{
    SetDriveSpeed(m_currentState.speed);
    SetTurnAngle(m_currentState.angle.Degrees());
}
void SwerveModule::SetDriveSpeed( units::velocity::meters_per_second_t speed )
{
    m_currentState.speed = speed;
    // convert mps to unitless rps by taking the speed and dividing by the circumference of the wheel
    auto driveTarget = speed.to<double>() /  units::length::meter_t(m_wheelDiameter).to<double>() * wpi::math::pi;  
	m_nt.get()->PutString("drive motor id", to_string(m_driveMotor.get()->GetID()) );
    m_nt.get()->PutNumber("drive target", driveTarget );
    m_driveMotor.get()->Set(m_nt, driveTarget);
}

void SwerveModule::SetTurnAngle( units::angle::degree_t targetAngle )
{
    m_currentState.angle = targetAngle;
    Rotation2d currAngle = Rotation2d(units::angle::degree_t(m_turnSensor.get()->GetAbsolutePosition()));
    Rotation2d deltaAngle = targetAngle - currAngle.Degrees();

    /**
    if ( deltaAngle.Degrees().to<double>() > 180.0 )
    {
        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::ERROR, string("delta angle too large motor"), to_string(m_turnMotor.get()->GetID()));
        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::ERROR, string("delta angle too large"), to_string(deltaAngle.Degrees().to<double>()));
    }
    **/

    m_nt.get()->PutString("turn motor id", to_string(m_turnMotor.get()->GetID()) );
    m_nt.get()->PutNumber("current angle", currAngle.Degrees().to<double>() );
    m_nt.get()->PutNumber("target angle", targetAngle.to<double>() );
    m_nt.get()->PutNumber("delta angle", deltaAngle.Degrees().to<double>() );
     
    if ( abs(deltaAngle.Degrees().to<double>()) > 0.01 )
    {
        auto motor = m_turnMotor.get()->GetSpeedController();
        auto fx = dynamic_cast<WPI_TalonFX*>(motor.get());
        auto sensors = fx->GetSensorCollection();
        double currentTicks = sensors.GetIntegratedSensorPosition();
    //    double deltaTicks = deltaAngle.Degrees().to<double>() * 72.5; //72.4694;
        double deltaTicks = (deltaAngle.Degrees().to<double>() * 72.5) / 4.0; //72.4694;
        double desiredTicks = currentTicks + deltaTicks;

        m_nt.get()->PutNumber("currentTicks", currentTicks );
        m_nt.get()->PutNumber("deltaTicks", deltaTicks );
        m_nt.get()->PutNumber("desiredTicks", desiredTicks );

        m_turnMotor.get()->SetControlMode(ControlModes::CONTROL_TYPE::POSITION_ABSOLUTE);
        m_turnMotor.get()->Set(m_nt, desiredTicks);
    }
    else
    {
//        m_turnMotor.get()->SetControlMode(ControlModes::CONTROL_TYPE::PERCENT_OUTPUT);
//        m_turnMotor.get()->Set(m_nt, 0.0);
    }

}



