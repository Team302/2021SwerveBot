
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

#include <subsys/PoseEstimatorEnum.h>
#include <subsys/SwerveChassis.h>
#include <subsys/SwerveChassisFactory.h>
#include <subsys/SwerveModule.h>
#include <utils/AngleUtils.h>
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
    m_nt(),
    m_activeState(),
    m_currentPose(),
    m_currentSpeed(0.0_rpm),
    m_currentRotations(0.0),
    //m_timer(),
    m_maxVelocity(1_mps),
    m_scale(1.0),
    m_boost(0.0),
    m_brake(0.0),
    m_runClosedLoopDrive(false)
{
    //m_timer.Reset();
    //m_timer.Start();
    
    Rotation2d ang { units::angle::degree_t(0.0)};
    m_activeState.angle = ang;
    m_activeState.speed = 0_mps;

    Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::PRINT, "CurrentPoseX", to_string(m_currentPose.X().to<double>()));
    Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::PRINT, "CurrentPoseY", to_string(m_currentPose.Y().to<double>()));
    
    // Set up the Drive Motor
    auto motor = m_driveMotor.get()->GetSpeedController();
    auto fx = dynamic_cast<WPI_TalonFX*>(motor.get());

    fx->ConfigOpenloopRamp(0.4, 0);
    fx->ConfigClosedloopRamp(0.4, 0);

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
    m_turnMotor.get()->SetControlConstants( 0, turnCData.get() );

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

/// @brief initialize the swerve module with information that the swerve chassis knows about
/// @param [in] units::velocity::meters_per_second_t - maxVelocity: maximum linear velocity of the chassis
/// @param [in] units::angular_velocity::radians_per_second_t - maxAngularVelocity: maximum angular velocity of the chassis
/// @param [in] units::acceleration::meters_per_second_squared_t - maxAcceleration: maximum linear acceleration of the chassis
/// @param [in] units::angular_acceleration::radians_per_second_squared_t - maxAngularAcceleration: maximum angular acceleration of the chassis

void SwerveModule::Init
(
    units::velocity::meters_per_second_t                        maxVelocity,
    units::angular_velocity::radians_per_second_t               maxAngularVelocity,
    units::acceleration::meters_per_second_squared_t            maxAcceleration,
    units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration,
    Translation2d                                               offsetFromCenterOfRobot
)
{
    m_maxVelocity = maxVelocity;
    auto driveCData = make_shared<ControlData>( ControlModes::CONTROL_TYPE::VELOCITY_RPS,
                                                ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
                                                string("DriveSpeed"),
                                                0.01,  // 0.01
                                                0.0,
                                                0.0,
                                                0.5,  // 0.5
                                                0.0,
                                                maxAcceleration.to<double>(),
                                                maxVelocity.to<double>(),
                                                maxVelocity.to<double>(),
                                                0.0 );
    m_driveMotor.get()->SetControlConstants( 0, driveCData.get() );

    //auto trans = Transform2d(offsetFromCenterOfRobot, Rotation2d() );
    //m_currentPose = m_currentPose + trans;
}

/// @brief Set all motor encoders to zero
/// @brief void
void SwerveModule::SetEncodersToZero()
{
    auto motor = m_driveMotor.get()->GetSpeedController();
    auto fx = dynamic_cast<WPI_TalonFX*>(motor.get());
    auto driveMotorSensors = fx->GetSensorCollection();
    driveMotorSensors.SetIntegratedSensorPosition(0, 0);
} 

/// @brief Get the encoder values
/// @returns double - the integrated sensor position
double SwerveModule::GetEncoderValues()
{
    auto motor = m_driveMotor.get()->GetSpeedController();
    auto fx = dynamic_cast<WPI_TalonFX*>(motor.get());
    auto driveMotorSensors = fx->GetSensorCollection();
    return driveMotorSensors.GetIntegratedSensorPosition();
}

/// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
/// @returns void
void SwerveModule::ZeroAlignModule()
{
    // Desired State
    SetTurnAngle(units::degree_t(0));
}


/// @brief Get the current state of the module (speed of the wheel and angle of the wheel)
/// @returns SwerveModuleState
SwerveModuleState SwerveModule::GetState() const 
{
    // Get the Module Drive Motor Speed
    auto mpr = units::length::meter_t(GetWheelDiameter() * pi );               
    auto mps = units::velocity::meters_per_second_t(mpr.to<double>() * m_driveMotor.get()->GetRPS());

    // Get the Module Current Rotation Angle
    Rotation2d angle {units::angle::degree_t(m_turnSensor.get()->GetAbsolutePosition())};

    // Create the state and return it
    SwerveModuleState state{mps,angle};
    return state;
}


/// @brief Set the current state of the module (speed of the wheel and angle of the wheel)
/// @param [in] const SwerveModuleState& targetState:   state to set the module to
/// @returns void
void SwerveModule::SetDesiredState
(
    const SwerveModuleState& targetState
)
{
    // Update targets so the angle turned is less than 90 degrees
    // If the desired angle is less than 90 degrees from the target angle (e.g., -90 to 90 is the amount of turn), just use the angle and speed values
    // if it is more than 90 degrees (90 to 270), the can turn the opposite direction -- increase the angle by 180 degrees -- and negate the wheel speed
    // finally, get the value between -90 and 90
    Rotation2d currAngle = Rotation2d(units::angle::degree_t(m_turnSensor.get()->GetAbsolutePosition()));
   auto optimizedState = Optimize(targetState, currAngle);
   // auto optimizedState = SwerveModuleState::Optimize(targetState, currAngle);
   // auto optimizedState = targetState;

    // Set Turn Target 
    SetTurnAngle(optimizedState.angle.Degrees());

    // Set Drive Target 
    SetDriveSpeed(optimizedState.speed);
}

/// @brief Given a desired swerve module state and the current angle of the swerve module, determine
///        if the changing the desired swerve module angle by 180 degrees is a smaller turn or not.
///        If it is, return a state that has that angle and the reversed speed.  Otherwise, return the 
///        original desired state.
/// Note:  the following was taken from the WPI code and tweaked because we were seeing some weird 
///        reversals that we believe was due to not using a tolerance
/// @param [in] const SwerveModuleState& desired state of the swerve module
/// @param [in] const Rotation2d& current angle of the swerve module
/// @returns SwerveModuleState optimized swerve module state
SwerveModuleState SwerveModule::Optimize
( 
    const SwerveModuleState& desiredState,
    const Rotation2d& currentAngle
) 
{
    SwerveModuleState optimizedState;
    optimizedState.angle = desiredState.angle;
    optimizedState.speed = desiredState.speed;

    auto delta = AngleUtils::GetDeltaAngle(currentAngle.Degrees(), optimizedState.angle.Degrees());

    string ntname = "Optimize";
    ntname += to_string(m_type);
    Logger::GetLogger()->ToNtTable(ntname, "current", currentAngle.Degrees().to<double>());
    Logger::GetLogger()->ToNtTable(ntname, "target", optimizedState.angle.Degrees().to<double>());
    Logger::GetLogger()->ToNtTable(ntname, "delta", delta.to<double>());
    
    // deal with roll over issues (e.g. want to go from -180 degrees to 180 degrees or vice versa)
    // keep the current angle
    if ((units::math::abs(delta) > 359_deg))
    {
        optimizedState.angle = currentAngle.Degrees();
    }
    // if delta is > 90 degrees or < -90 degrees, we can turn the wheel the otherway and
    // reverse the wheel direction (negate speed)
    // if the delta is > 90 degrees, rotate the module the opposite direction and negate the speed
    else if ((units::math::abs(delta)) > 90_deg) 
    {
        optimizedState.speed *= -1.0;
        optimizedState.angle += Rotation2d{180_deg};
        Logger::GetLogger()->ToNtTable(ntname, "reversing", delta.to<double>());
    }

    // if the delta is > 90 degrees, rotate the 
    //if ((units::math::abs(delta.Degrees()) - 160_deg) > 0.1_deg) 
    if ((units::math::abs(delta) - 90_deg) > 0.1_deg) 
    {
        Logger::GetLogger()->ToNtTable(ntname, "optimized", (desiredState.angle + Rotation2d{180_deg}).Degrees().to<double>());
        return {-desiredState.speed, desiredState.angle + Rotation2d{180_deg}};
    } 
    else 
    {
        Logger::GetLogger()->ToNtTable(ntname, "optimized", desiredState.angle.Degrees().to<double>());
        return {desiredState.speed, desiredState.angle};
    }
}

/// @brief Run the swerve module at the same speed and angle
/// @returns void
void SwerveModule::RunCurrentState()
{
    SetDriveSpeed(m_activeState.speed);

    auto motor = m_turnMotor.get()->GetSpeedController();
    auto fx = dynamic_cast<WPI_TalonFX*>(motor.get());
    fx->StopMotor();  
}

/// @brief run the drive motor at a specified speed
/// @param [in] speed to drive the drive wheel as
/// @returns void
void SwerveModule::SetDriveSpeed( units::velocity::meters_per_second_t speed )
{
    m_activeState.speed = ( abs(speed.to<double>()/m_maxVelocity.to<double>()) < 0.05 ) ? 0_mps : speed;

    Logger::GetLogger()->ToNtTable(m_nt, string("State Speed - mps"), m_activeState.speed.to<double>() );
    Logger::GetLogger()->ToNtTable(m_nt, string("Wheel Diameter - meters"), units::length::meter_t(m_wheelDiameter).to<double>() );
    Logger::GetLogger()->ToNtTable(m_nt, string("drive motor id"), m_driveMotor.get()->GetID() );
    Logger::GetLogger()->ToNtTable(m_nt, string("drive scale"), m_scale );

    if (m_runClosedLoopDrive)
    {
        // convert mps to unitless rps by taking the speed and dividing by the circumference of the wheel
        auto driveTarget = m_activeState.speed.to<double>() / (units::length::meter_t(m_wheelDiameter).to<double>() * wpi::math::pi);  
        driveTarget /= m_driveMotor.get()->GetGearRatio();
        driveTarget *= clamp((m_scale + m_boost - m_brake), 0.0, 1.0);
        
        Logger::GetLogger()->ToNtTable(m_nt, string("drive target - rps"), driveTarget );
        
        m_driveMotor.get()->SetControlMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        m_driveMotor.get()->Set(m_nt, driveTarget);
    }
    else
    {
        auto percent = m_activeState.speed / m_maxVelocity;
        percent *= clamp((m_scale + m_boost - m_brake), 0.0, 1.0);

        Logger::GetLogger()->ToNtTable(m_nt, string("drive target - percent"), percent );

        m_driveMotor.get()->SetControlMode(ControlModes::CONTROL_TYPE::PERCENT_OUTPUT);
        m_driveMotor.get()->Set(m_nt, percent);
    }
}

/// @brief Turn the swerve module to a specified angle
/// @param [in] units::angle::degree_t the target angle to turn the wheel to
/// @returns void
void SwerveModule::SetTurnAngle( units::angle::degree_t targetAngle )
{
    m_activeState.angle = targetAngle;

    Logger::GetLogger()->ToNtTable(m_nt, string("turn motor id"), m_turnMotor.get()->GetID() );
    Logger::GetLogger()->ToNtTable(m_nt, string("target angle"), targetAngle.to<double>() );

    auto currAngle  = units::angle::degree_t(m_turnSensor.get()->GetAbsolutePosition());
    auto deltaAngle = AngleUtils::GetDeltaAngle(currAngle, targetAngle);

    Logger::GetLogger()->ToNtTable(m_nt, string("current angle"), currAngle.to<double>() );
    Logger::GetLogger()->ToNtTable(m_nt, string("delta angle"), deltaAngle.to<double>() );

    if ( abs(deltaAngle.to<double>()) > 1.0 )
    {
        auto motor = m_turnMotor.get()->GetSpeedController();
        auto fx = dynamic_cast<WPI_TalonFX*>(motor.get());
        auto sensors = fx->GetSensorCollection();
        //=============================================================================
        // 5592 counts on the falcon for 76.729 degree change on the CANCoder (wheel)
        //=============================================================================
        //double deltaTicks = (deltaAngle.Degrees().to<double>() * 5592 / 76.729); 
        double deltaTicks = (deltaAngle.to<double>() * 5592 / 76.729); 
        double currentTicks = sensors.GetIntegratedSensorPosition();
        double desiredTicks = currentTicks + deltaTicks;

        Logger::GetLogger()->ToNtTable(m_nt, string("currentTicks"), currentTicks );
        Logger::GetLogger()->ToNtTable(m_nt, string("deltaTicks"), deltaTicks );
        Logger::GetLogger()->ToNtTable(m_nt, string("desiredTicks"), desiredTicks );

        m_turnMotor.get()->SetControlMode(ControlModes::CONTROL_TYPE::POSITION_ABSOLUTE);
        m_turnMotor.get()->Set(m_nt, desiredTicks);
    }
    else
    {
        m_turnMotor.get()->SetControlMode(ControlModes::CONTROL_TYPE::PERCENT_OUTPUT);
        m_turnMotor.get()->Set(m_nt, 0.0);
    }

}

/// @brief stop the drive and turn motors
/// @return void
void SwerveModule::StopMotors()
{
    m_turnMotor.get()->GetSpeedController()->StopMotor();
    m_driveMotor.get()->GetSpeedController()->StopMotor();
}

frc::Pose2d SwerveModule::GetCurrentPose(PoseEstimationMethod opt)
{
    // get change in time
    //auto deltaT = m_timer.Get();
    //m_timer.Reset();

    // get the information from the last pose
    auto startX         = m_currentPose.X();
    auto startY         = m_currentPose.Y();
    auto startAngle     = m_currentPose.Rotation().Radians();
    //auto startSpeed     = m_currentSpeed;
    auto startRotations = m_currentRotations;

    // read sensor info (cancoder, encoders) for current speed and angle of the module
    // calculate the average from the last 
    auto currentAngle   = units::angle::radian_t(units::angle::degree_t(m_turnSensor.get()->GetPosition()));
    //auto avgAngle       = (currentAngle - startAngle) / 2.0;
    auto currentRotations = m_driveMotor.get()->GetRotations();
    //auto currentSpeed   = units::angular_velocity::revolutions_per_minute_t(m_driveMotor.get()->GetRPS()*60.0);
    //auto avgSpeed       = (currentSpeed - startSpeed) / 2.0;

    units::length::meter_t currentX {units::length::meter_t(0)};
    units::length::meter_t currentY {units::length::meter_t(0)};

    if (opt == PoseEstimationMethod::EULER_USING_MODULES)
    {
        // Euler Method
        //
        // xk+1 = xk + vk cos θk T = xk + delta * cos θk
        // yk+1 = yk + vk sin θk T = yk + delta * sin θk
        // Thetak+1 = Thetagyro,k+1

        auto delta  = currentRotations - startRotations;
        auto circum = wpi::math::pi * m_wheelDiameter;

        currentX = startX + cos(startAngle.to<double>()) * circum;
        currentY = startY + sin(startAngle.to<double>()) * circum;
        //auto delta = units::length::meter_t((currentRotations - startRotations) * wpi::math::pi * m_wheelDiameter);
        //currentX = startX + cos(startAngle.to<double>())*delta;
        //currentY = startY + sin(startAngle.to<double>())*delta;

        Logger::GetLogger()->ToNtTable(m_nt, "start rotations",startRotations);
        Logger::GetLogger()->ToNtTable(m_nt, "current rotations",currentRotations);
        Logger::GetLogger()->ToNtTable(m_nt, "delta", delta);
        Logger::GetLogger()->ToNtTable(m_nt, "circumference", circum.to<double>());

        //
        // Would it be more accurate to use either the start or end angle instead of the average of 
        // the two?
        /**  
        currentX = startX + units::length::meter_t(units::length::inch_t(startSpeed.to<double>() * 60.0 *    // average speed (rps)
                                                  m_wheelDiameter * wpi::math::pi * // distance per revolution (inches)
                                                  cos(startAngle.to<double>()) *      // cosine of the average angle
                                                  deltaT.to<double>()));             // delta T (seconds)
        Logger::GetLogger()->ToNtTable(m_nt, "AvgSpeed", avgSpeed.to<double>());
        Logger::GetLogger()->ToNtTable(m_nt, "CosAvgAngle", cos(avgAngle.to<double>()));
        Logger::GetLogger()->ToNtTable(m_nt, "DeltaT", deltaT.to<double>());
        **/
        Logger::GetLogger()->ToNtTable(m_nt, "WheelDiameter", m_wheelDiameter.to<double>());
        Logger::GetLogger()->ToNtTable(m_nt, "CurrentX", currentX.to<double>());
        Logger::GetLogger()->ToNtTable(m_nt, "CurrentY", currentY.to<double>());
        Logger::GetLogger()->ToNtTable(m_nt, "startX", startX.to<double>());
        Logger::GetLogger()->ToNtTable(m_nt, "startY", startY.to<double>());


        //currentY = startY + units::length::meter_t(units::length::inch_t(startSpeed.to<double>() * 60.0 *    // average speed (rps)
        //                                          m_wheelDiameter * wpi::math::pi * // distance per revolution (inches)
        //                                          sin(startAngle.to<double>()) *      // sine of the average angle
        //                                          deltaT.to<double>()));             // delta T (seconds)
    }
    else if (opt == PoseEstimationMethod::POSE_EST_USING_MODULES)
    {
        // Pose estimation method from Controls Engineering in FIRST Robotics Competition by 
        // Tyler Veness
        //   
        // variables:
        //   dX - delta X - Change in pose's X (forward/backward directions)
        //   dY - delta Y - Change in pose's Y (strafe directions)
        //   dT - delta theta - Change in pose's theta (rotation about its center of rotation)
        //    T - theta - Starting Angle in global coordinate frame
        //    t - time since last pose update
        //    vx - velocity along X-axis
        //    vy - velocity along Y-axis
        //    w - omega - angular velocity
        //
        //   Matrices with a G are in global space and R are in Robot space
        //
        //  G -    -       -                     - R-                                  - R-    -
        //    | dX |       | cos(T)  -sin(T)   0 |  | sin(wt) / w     cos(wt)-1 / w  0 |  | vx |
        //    | dY |   =   | sin(T)   cos(T)   0 |  | 1-cos(wt) / w   sin(wt) / w    0 |  | vy |
        //    | dt |       |   0       0       1 |  |      0               0         t |  | w  |
        //    -    -       -                     -  -                                  -  -    -
        //
        // or
        //
        //  G -    -       -                     - R-                                    - R-    -
        //    | dX |       | cos(T)  -sin(T)   0 |  | sin(dT) / dT     cos(dT)-1 / dT  0 |  | dX |
        //    | dY |   =   | sin(T)   cos(T)   0 |  | 1-cos(dT) / dT   sin(dT) / dT    0 |  | dY |
        //    | dt |       |   0       0       1 |  |      0               0           1 |  | dY |
        //    -    -       -                     -  -                                    -  -    -
        // TODO:  if Euler isn't accurate enough implement this
        // If so, can we use the Euler matrix library to do this or do we need to manually do it ourselves?

    }

    // update the pose (do we need to look if our encoder distance matches and adjust here???)
    auto newpose = Pose2d(currentX, currentY, Rotation2d(currentAngle));
    auto trans   = newpose - m_currentPose;
    m_currentPose += trans;
    //m_currentSpeed = currentSpeed;

    Logger::GetLogger()->ToNtTable(m_nt, "NewPoseX", newpose.X().to<double>());
    Logger::GetLogger()->ToNtTable(m_nt, "NewPoseY", newpose.Y().to<double>());
    Logger::GetLogger()->ToNtTable(m_nt, "TransX", trans.X().to<double>());
    Logger::GetLogger()->ToNtTable(m_nt, "TransY", trans.Y().to<double>());

    m_currentRotations = currentRotations;
    // Do we need to do any alterations based on the actual distance driven since we're 
    // approximating the path with an average vector instead of an arc
    // auto deltaEncoder   = m_currentRotations - startRotations;
    // auto distTravelled  = deltaEncoder * m_wheelDiameter*wpi::math::pi;

    return m_currentPose;
}

void SwerveModule::UpdateCurrPose
(
    units::length::meter_t  x,
    units::length::meter_t  y
)
{
    m_currentPose += { Translation2d{x,y}, 
                        Rotation2d{units::angle::degree_t(m_turnSensor.get()->GetPosition())}};
}
