
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
    ModuleID                                                type, 
    shared_ptr<IDragonMotorController>                      driveMotor, 
    shared_ptr<IDragonMotorController>                      turnMotor, 
    std::shared_ptr<ctre::phoenix::sensors::CANCoder>		canCoder,
    units::length::inch_t                                   wheelDiameter
) : m_type(type), 
    m_driveMotor(driveMotor), 
    m_turnMotor(turnMotor), 
    m_turnSensor(canCoder), 
    m_wheelDiameter(wheelDiameter)
{
    auto motor = m_driveMotor.get()->GetSpeedController();
    auto fx = dynamic_cast<WPI_TalonFX*>(motor.get());
    fx->ConfigSelectedFeedbackSensor( ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 10 );

    auto chassis = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis();
    auto maxSp = chassis.get()->GetMaxSpeed();
    auto driveCTL = make_unique<ControlData>( ControlModes::CONTROL_TYPE::VELOCITY_RPS,
                                              ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
                                              string("DriveSpeed"),
                                              0.1,
                                              0.0,
                                              0.0,
                                              0.3,
                                              0.0,
                                              chassis.get()->GetMaxAcceleration(),
                                              maxSp.to<double>(),
                                              maxSp.to<double>(),
                                              0.0 );
    m_driveMotor.get()->SetControlConstants( driveCTL.get() );

//    m_turnSensor.get()->ConfigFactoryDefault();
    m_turnSensor.get()->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180, 0);
    motor = m_turnMotor.get()->GetSpeedController();
    fx = dynamic_cast<WPI_TalonFX*>(motor.get());
//    fx->ConfigSelectedFeedbackSensor( ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 10 );
//    fx->ConfigIntegratedSensorAbsoluteRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180 );
    
    fx->ConfigRemoteFeedbackFilter(m_turnSensor.get()->GetDeviceNumber(), ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 10 );
    fx->ConfigSelectedFeedbackSensor( ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor0, 0, 10 );

      auto maxAng = chassis->GetMaxAngularSpeed();
    auto turnCTL  = make_unique<ControlData>( ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE,
                                              ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
                                              string("TrunProfile"),
                                              0.1,
                                              0.0,
                                              0.0,
                                              0.3,
                                              0.0,
                                              chassis.get()->GetMaxAcceleration(),
                                              maxAng.to<double>(),
                                              maxAng.to<double>(),
                                              0.0 );
    m_turnMotor.get()->SetControlConstants( driveCTL.get() );

}

void SwerveModule::Init
(
    units::velocity::meters_per_second_t                maxVelocity,
    units::angular_velocity::radians_per_second_t       maxAngularVelocity,
    double                                              maxAccMperSecSq
)
{
    auto driveCTL = make_unique<ControlData>( ControlModes::CONTROL_TYPE::VELOCITY_RPS,
                                              ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
                                              string("DriveSpeed"),
                                              0.01,
                                              0.0,
                                              0.0,
                                              0.5,
                                              0.0,
                                              maxAccMperSecSq,
                                              maxVelocity.to<double>(),
                                              maxVelocity.to<double>(),
                                              0.0 );
    m_driveMotor.get()->SetControlConstants( driveCTL.get() );

    auto turnCTL  = make_unique<ControlData>( ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE,
                                              ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
                                              string("TurnProfile"),
                                              0.01,
                                              0.0,
                                              0.0,
                                              0.5,
                                              0.0,
                                              maxAccMperSecSq,
                                              maxAngularVelocity.to<double>(),
                                              maxAngularVelocity.to<double>(),
                                              0.0 );
    m_turnMotor.get()->SetControlConstants( turnCTL.get() );
}

/// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
/// @returns void
void SwerveModule::ZeroAlignModule()
{
     //SwerveModuleState state;
     auto state = GetState();
     state.angle = Rotation2d(units::angle::degree_t(0.0));
     state.speed = 0.0_mps; 

     SetDesiredState( state );
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
    const auto state = SwerveModuleState::Optimize(referenceState, units::angle::degree_t(m_turnSensor.get()->GetAbsolutePosition()));

    // Set Drive Target 
    // convert mps to unitless rps by taking the speed and dividing by the circumference of the wheel
    auto driveTarget = state.speed.to<double>() /  units::length::meter_t(m_wheelDiameter).to<double>() * wpi::math::pi;  

    // Set Turn Target
    auto turnTarget = state.angle.Degrees().to<double>();  

    // Run the motors
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
    auto nt = nt::NetworkTableInstance::GetDefault().GetTable(ntName);

    nt->PutNumber( "current angle", m_turnSensor.get()->GetAbsolutePosition() );
    nt->PutNumber( "target angle", turnTarget );
    nt->PutNumber( "drive target", driveTarget );
    m_driveMotor.get()->Set(nt, driveTarget);
    m_turnMotor.get()->Set(nt, turnTarget);
}