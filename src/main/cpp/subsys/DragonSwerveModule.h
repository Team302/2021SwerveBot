#pragma once

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/math>
#include <hw/DragonFalcon.h>


class DragonSwerveModule {
    public:
        DragonSwerveModule(DragonFalcon* driveMotor, DragonFalcon* turningMotor);
        frc::SwerveModuleState GetState() const;
        void SetDesiredState(const frc::SwerveModuleState& state);

        
    private:
        static constexpr auto WheelRadius = 0.0508_m; //TODO #1 Put values in from XML
        static constexpr int EncoderResolution = 4096;

        static constexpr auto ModuleMaxAngularVelocity = wpi::math::pi * 1_rad_per_s; //Radians per second
        static constexpr auto ModuleMaxAngularAcceleration = wpi::math::pi * 2_rad_per_s / 1_s; //Radians per second ^2

        DragonFalcon* m_driveMotor;
        DragonFalcon* m_turnMotor;

        //TODO #2 Encoder from falcons
        frc::Encoder m_driveEncoder{0, 1};
        frc::Encoder m_turnEncoder{2, 3};

        frc2::PIDController m_drivePIDController{1.0, 0, 0};
        frc::ProfiledPIDController<units::radians> m_turningPIDController{
        1.0,
        0.0,
        0.0,
        {ModuleMaxAngularVelocity, ModuleMaxAngularAcceleration}};

        frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V, 3_V / 1_mps};
        frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
        1_V, 0.5_V / 1_rad_per_s};
};