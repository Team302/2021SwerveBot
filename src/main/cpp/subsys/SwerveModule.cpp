
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
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <wpi/math>

// Team 302 includes
#include <controllers/ControlData.h>
#include <controllers/ControlModes.h>
#include <subsys/SwerveChassis.h>
#include <subsys/SwerveChassisFactory.h>
#include <subsys/SwerveModule.h>
#include <units/angle.h>


// Third Party Includes
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>


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
    m_wheelDiameter(wheelDiameter),
    m_driveFeedforward(1_V, 3_V / 1_mps),
    m_turnFeedforward(1_V, 0.5_V / 1_rad_per_s)
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
                                              chassis.get()->GetMaxAcceration(),
                                              maxSp.to<double>(),
                                              maxSp.to<double>(),
                                              0.0 );
    m_driveMotor.get()->SetControlConstants( driveCTL.get() );



    m_turnSensor.get()->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180, 0);
    motor = m_turnMotor.get()->GetSpeedController();
    fx = dynamic_cast<WPI_TalonFX*>(motor.get());
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
                                              chassis.get()->GetMaxAcceration(),
                                              maxAng.to<double>(),
                                              maxAng.to<double>(),
                                              0.0 );
    m_turnMotor.get()->SetControlConstants( driveCTL.get() );

    /**
    auto driveCountsPerRev = driveMotor.get()->GetCountsPerRev();
    m_driveEncoder.SetDistancePerPulse(wpi::math::pi * m_wheelDiameter.to<double>() / driveCountsPerRev);
    m_drivePIDController = make_shared<frc2::PIDController>(1.0, 0.0, 0.0);
    m_turnPIDController = make_shared<frc2::PIDController>(1.0, 0.0, 0.0);
    //TrapezoidProfile<units::meters>::Constraints constraints{wpi::math::pi * 1_rad_per_s, wpi::math::pi * 2_rad_per_s / 1_s};
    //m_turningPIDController = make_shared<ProfiledPIDController<units::radians>>(1.0, 0.0, 0.0, constraints);


    auto turnCountsPerRev = turnMotor.get()->GetCountsPerRev();  // todo need to make this utilize the cancoder
    m_turnEncoder.SetDistancePerPulse(2 * wpi::math::pi /turnCountsPerRev);

   // m_turnPIDController.get()->EnableContinuousInput(-1.0*units::angle::radian_t(wpi::math::pi), units::angle::radian_t(wpi::math::pi));
    m_turnPIDController.get()->EnableContinuousInput(-1.0*wpi::math::pi, wpi::math::pi);
    **/
}


/// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
/// @returns void
void SwerveModule::ZeroAlignModule()
{
     SwerveModuleState state;
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
    auto driveTarget = state.speed.to<double>();

    // Set Turn Target
    auto turnTarget = state.angle.Degrees().to<double>();  

    // Run the motors
    m_driveMotor.get()->Set(driveTarget);
    m_turnMotor.get()->Set(turnTarget);

    // set up the drive speed PID controller
    /**
    const auto driveOutput = m_drivePIDController.get()->Calculate(m_driveEncoder.GetRate(), referenceState.speed.to<double>());
    const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);
    **/
    // set up the turn angle position PID controller
    //const auto turnOutput = m_turnFeedforward.Calculate(m_turnPIDController.get()->GetSetpoint().velocity);
    //const auto turnFeedforward = m_turnFeedforward.Calculate(m_turnPIDController.get()->GetSetpoint().velocity);
    // TODO this is all wrong as it is velocity based instead of position based
    /**
    const auto turnOutput = m_drivePIDController.get()->Calculate(m_turnEncoder.GetRate(), referenceState.speed.to<double>());
    const auto turnFeedforward = m_driveFeedforward.Calculate(state.speed);

    auto drive = dynamic_cast<DragonFalcon*>(m_driveMotor.get());
    auto turn = dynamic_cast<DragonFalcon*>(m_turnMotor.get());

    drive -> SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
    turn -> SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
    **/
}