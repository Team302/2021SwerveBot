#include <subsys/DragonSwerveModule.h>
#include <units/angle.h>
#include <frc/geometry/Rotation2d.h>
#include <wpi/math>

DragonSwerveModule::DragonSwerveModule(DragonFalcon* driveMotor, DragonFalcon* turnMotor) : 
m_driveMotor(driveMotor), m_turnMotor(turnMotor)
{
    m_driveEncoder.SetDistancePerPulse(2 * wpi::math::pi * WheelRadius.to<double>() / EncoderResolution);

    m_turnEncoder.SetDistancePerPulse(2 * wpi::math::pi /EncoderResolution);

    m_turningPIDController.EnableContinuousInput(-units::angle::radian_t(wpi::math::pi), units::angle::radian_t(wpi::math::pi));
}

frc::SwerveModuleState DragonSwerveModule::GetState() const 
{
    return {units::meters_per_second_t{m_driveEncoder.GetRate()}, frc::Rotation2d(units::radian_t(m_turnEncoder.Get()))};
}



void DragonSwerveModule::SetDesiredState
(
    const frc::SwerveModuleState& referenceState
)
{
    //const auto state = frc::SwerveModuleState::Optimize(referenceState, units::angle::radian_t(m_turnEncoder.Get()));

    const auto driveOutput = m_drivePIDController.Calculate(m_driveEncoder.GetRate(), referenceState.speed.to<double>());

    const auto driveFeedforward = m_driveFeedforward.Calculate(referenceState.speed);

    const auto turnOutput = m_turnFeedforward.Calculate(m_turningPIDController.GetSetpoint().velocity);

    const auto turnFeedforward = m_turnFeedforward.Calculate(m_turningPIDController.GetSetpoint().velocity);

    m_driveMotor -> SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
    m_turnMotor -> SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}
