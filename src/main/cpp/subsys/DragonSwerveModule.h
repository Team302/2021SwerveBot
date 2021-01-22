#pragma once

#include <memory>

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
#include <hw/interfaces/IDragonMotorController.h>
#include <ctre/phoenix/sensors/CANCoder.h>


class DragonSwerveModule 
{
    public:
        enum ModuleID
        {
            LEFT_FRONT,
            RIGHT_FRONT,
            LEFT_BACK,
            RIGHT_BACK
        };

        DragonSwerveModule( ModuleID type, 
                            std::shared_ptr<IDragonMotorController>                 driveMotor, 
                            std::shared_ptr<IDragonMotorController>                 turningMotor,
                            std::shared_ptr<ctre::phoenix::sensors::CANCoder>		canCoder, 
                            units::degree_t                                         turnOffset,
                            units::length::inch_t                                   wheelDiameter
                          );

        void ZeroAlignModule();
        frc::SwerveModuleState GetState() const;
        void SetDesiredState(const frc::SwerveModuleState& state);

        ModuleID GetType() {return m_type; }
        units::length::inch_t GetWheelDiameter() const {return m_wheelDiameter;}

        
    private:
       // static constexpr auto ModuleMaxAngularVelocity = wpi::math::pi * 1_rad_per_s; //Radians per second
       // static constexpr auto ModuleMaxAngularAcceleration = wpi::math::pi * 2_rad_per_s / 1_s; //Radians per second ^2

        ModuleID m_type;
        std::shared_ptr<IDragonMotorController>             m_driveMotor;
        std::shared_ptr<IDragonMotorController>             m_turnMotor;
        std::shared_ptr<ctre::phoenix::sensors::CANCoder>   m_turnSensor;
        units::degree_t                                     m_turnOffset;
        units::length::inch_t                               m_wheelDiameter;

        //TODO #2 Encoder from falcons
        frc::Encoder m_driveEncoder{0, 1};
        frc::Encoder m_turnEncoder{2, 3};

        //frc2::PIDController m_drivePIDController{1.0, 0, 0};
        std::shared_ptr<frc2::PIDController> m_drivePIDController;

        //frc::ProfiledPIDController<units::radians> m_turnPIDController{1.0, 0.0, 0.0, {ModuleMaxAngularVelocity, ModuleMaxAngularAcceleration}};
        std::shared_ptr<frc2::PIDController> m_turnPIDController;
        //std::shared_ptr<frc::ProfiledPIDController<units::radians>> m_turnPIDController;
        

        frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward;
        frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward;
};