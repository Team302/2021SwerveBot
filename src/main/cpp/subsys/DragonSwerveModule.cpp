#include <memory>
#include <subsys/DragonSwerveModule.h>
#include <units/angle.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <wpi/math>

using namespace std;
using namespace frc;
using namespace ctre::phoenix::sensors;

DragonSwerveModule::DragonSwerveModule
(
    ModuleID                                                type, 
    shared_ptr<IDragonMotorController>                      driveMotor, 
    shared_ptr<IDragonMotorController>                      turnMotor, 
    std::shared_ptr<ctre::phoenix::sensors::CANCoder>		canCoder,
    units::degree_t                                         turnOffset,
    units::length::inch_t                                   wheelDiameter
) : m_type(type), 
    m_driveMotor(driveMotor), 
    m_turnMotor(turnMotor), 
    m_turnSensor(canCoder), 
    m_turnOffset(turnOffset),
    m_wheelDiameter(wheelDiameter),
    m_driveFeedforward(1_V, 3_V / 1_mps),
    m_turnFeedforward(1_V, 0.5_V / 1_rad_per_s)

{
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
}

void DragonSwerveModule::ZeroAlignModule()
{
     SwerveModuleState state;
     state.angle = m_turnOffset;
     state.speed = 0.0_mps; 

     SetDesiredState( state );
}


SwerveModuleState DragonSwerveModule::GetState() const 
{
    return {units::meters_per_second_t{m_driveEncoder.GetRate()}, Rotation2d(units::radian_t(m_turnEncoder.Get()))};
}



void DragonSwerveModule::SetDesiredState
(
    const SwerveModuleState& referenceState
)
{
    // Update targets so the angle turned is less than 90 degrees
    // If the desired angle is less than 90 degrees from the target angle (e.g., -90 to 90 is the amount of turn), just use the angle and speed values
    // if it is more than 90 degrees (90 to 270), the can turn the opposite direction -- increase the angle by 180 degrees -- and negate the wheel speed
    // finally, get the value between -90 and 90
    const auto state = SwerveModuleState::Optimize(referenceState, units::angle::radian_t(m_turnEncoder.Get()));

    // set up the drive speed PID controller
    const auto driveOutput = m_drivePIDController.get()->Calculate(m_driveEncoder.GetRate(), referenceState.speed.to<double>());
    const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

    // set up the turn angle position PID controller
    //const auto turnOutput = m_turnFeedforward.Calculate(m_turnPIDController.get()->GetSetpoint().velocity);
    //const auto turnFeedforward = m_turnFeedforward.Calculate(m_turnPIDController.get()->GetSetpoint().velocity);
    // TODO this is all wrong as it is velocity based instead of position based
    const auto turnOutput = m_drivePIDController.get()->Calculate(m_turnEncoder.GetRate(), referenceState.speed.to<double>());
    const auto turnFeedforward = m_driveFeedforward.Calculate(state.speed);

    auto drive = dynamic_cast<DragonFalcon*>(m_driveMotor.get());
    auto turn = dynamic_cast<DragonFalcon*>(m_turnMotor.get());

    drive -> SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
    turn -> SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}