
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
#include <cmath>

// FRC includes
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

// Team 302 includes
#include <subsys/SwerveChassis.h>

// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>

using namespace std;
using namespace frc;

/// @brief Construct a swerve chassis
/// @param [in] std::shared_ptr<SwerveModule>           frontleft:          front left swerve module
/// @param [in] std::shared_ptr<SwerveModule>           frontright:         front right swerve module
/// @param [in] std::shared_ptr<SwerveModule>           backleft:           back left swerve module
/// @param [in] std::shared_ptr<SwerveModule>           backright:          back right swerve module
/// @param [in] units::length::inch_t                   wheelBase:          distance between the front and rear wheels
/// @param [in] units::length::inch_t                   track:              distance between the left and right wheels
/// @param [in] units::velocity::meters_per_second_t    maxSpeed:           maximum linear speed of the chassis 
/// @param [in] units::radians_per_second_t             maxAngularSpeed:    maximum rotation speed of the chassis 
/// @param [in] double                                  maxAcceleration:    maximum acceleration in meters_per_second_squared
SwerveChassis::SwerveChassis
(
    shared_ptr<SwerveModule>                                    frontLeft, 
    shared_ptr<SwerveModule>                                    frontRight,
    shared_ptr<SwerveModule>                                    backLeft, 
    shared_ptr<SwerveModule>                                    backRight, 
    units::length::inch_t                                       wheelBase,
    units::length::inch_t                                       track,
    units::velocity::meters_per_second_t                        maxSpeed,
    units::radians_per_second_t                                 maxAngularSpeed,
    units::acceleration::meters_per_second_squared_t            maxAcceleration,
    units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration
) : m_frontLeft(frontLeft), 
    m_frontRight(frontRight), 
    m_backLeft(backLeft), 
    m_backRight(backRight), 
    m_flState(),
    m_frState(),
    m_blState(),
    m_brState(),
    m_wheelBase(wheelBase),
    m_track(track),
    m_maxSpeed(maxSpeed),
    m_maxAngularSpeed(maxAngularSpeed),
    m_maxAcceleration(maxAcceleration),
    m_maxAngularAcceleration(maxAngularAcceleration),
    m_pigeon(PigeonFactory::GetFactory()->GetPigeon()),
    m_scale(1.0),
    m_runWPI(false),
    m_frontLeftLocation(wheelBase/2.0, track/2.0),
    m_frontRightLocation(wheelBase/2.0, -1.0*track/2.0),
    m_backLeftLocation(-1.0*wheelBase/2.0, track/2.0),
    m_backRightLocation(-1.0*wheelBase/2.0, -1.0*track/2.0)
{
    frontLeft.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration );
    frontRight.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration );
    backLeft.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration );
    backRight.get()->Init( maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration );

    ZeroAlignSwerveModules();
}
/// @brief Align all of the swerve modules to point forward
void SwerveChassis::ZeroAlignSwerveModules()
{
    m_frontLeft.get()->ZeroAlignModule();
    m_frontRight.get()->ZeroAlignModule();
    m_backLeft.get()->ZeroAlignModule();
    m_backRight.get()->ZeroAlignModule();
}

void SwerveChassis::SetDriveScaleFactor( double scale )
{
    m_scale = scale;
    m_frontLeft.get()->SetDriveScale(m_scale);
    m_frontRight.get()->SetDriveScale(m_scale);
    m_backLeft.get()->SetDriveScale(m_scale);
    m_backRight.get()->SetDriveScale(m_scale);
}



/// @brief Drive the chassis
/// @param [in] units::velocity::meters_per_second_t            xSpeed:         forward/reverse speed (positive is forward)
/// @param [in] units::velocity::meters_per_second_t            ySpeed:         left/right speed (positive is left)
/// @param [in] units::angular_velocity::radians_per_second_t   rot:            Rotation speed around the vertical (Z) axis; (positive is counter clockwise)
/// @param [in] bool                                            fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
///                                                                             false: direction is based on robot front/back
void SwerveChassis::Drive( units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, 
                           bool fieldRelative) 
{
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "XSpeed", xSpeed.to<double>() );
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "YSpeed", ySpeed.to<double>() );
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "ZSpeed", rot.to<double>() );
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "yaw", m_pigeon->GetYaw() );
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "scale", m_scale );
    
    if ( (abs(xSpeed.to<double>()) < m_deadband) && 
         (abs(ySpeed.to<double>()) < m_deadband) && 
         (abs(rot.to<double>())    < m_deadband) )
    {
        m_frontLeft.get()->StopMotors();
        m_frontRight.get()->StopMotors();
        m_backLeft.get()->StopMotors();
        m_backRight.get()->StopMotors();
    }
    else
    {   
        if ( m_runWPI )
        {
            units::degree_t yaw{m_pigeon->GetYaw()};
            Rotation2d currentOrientation {yaw};
            auto states = m_kinematics.ToSwerveModuleStates
                                    (fieldRelative ?  ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currentOrientation) : 
                                                    ChassisSpeeds{xSpeed, ySpeed, rot} );

            m_kinematics.NormalizeWheelSpeeds(&states, m_maxSpeed);

            auto [fl, fr, bl, br] = states;
        
            m_frontLeft.get()->SetDesiredState(fl);
            m_frontRight.get()->SetDesiredState(fr);
            m_backLeft.get()->SetDesiredState(bl);
            m_backRight.get()->SetDesiredState(br);        
        }
        else
        {
            ChassisSpeeds speeds = fieldRelative ? GetFieldRelativeSpeeds(xSpeed,ySpeed, rot) : 
                                                   ChassisSpeeds{xSpeed, ySpeed, rot};
            // TODO: need to spin our own kinematics module, but in the mean time do the following to get
            // approximate updates
            auto states = m_kinematics.ToSwerveModuleStates(speeds);
            m_kinematics.NormalizeWheelSpeeds(&states, m_maxSpeed);

            CalcSwerveModuleStates(speeds);

            m_frontLeft.get()->SetDesiredState(m_flState);
            m_frontRight.get()->SetDesiredState(m_frState);
            m_backLeft.get()->SetDesiredState(m_blState);
            m_backRight.get()->SetDesiredState(m_brState);
        }
    }
}

/// @brief Drive the chassis
/// @param [in] frc::ChassisSpeeds  speeds:         kinematics for how to move the chassis
/// @param [in] bool                fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
///                                                 false: direction is based on robot front/back
void SwerveChassis::Drive( ChassisSpeeds speeds, bool fieldRelative) 
{
    Drive( speeds.vx, speeds.vy, speeds.omega, fieldRelative); 
}

/// @brief Drive the chassis
/// @param [in] double  drivePercent:   forward/reverse percent output (positive is forward)
/// @param [in] double  steerPercent:   left/right percent output (positive is left)
/// @param [in] double  rotatePercent:  Rotation percent output around the vertical (Z) axis; (positive is counter clockwise)
/// @param [in] bool    fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
///                                     false: direction is based on robot front/back
void SwerveChassis::Drive( double drive, double steer, double rotate, bool fieldRelative )
{
    if ( abs(drive)  < m_deadband && 
         abs(steer)  < m_deadband && 
         abs(rotate) < m_deadband )
    {
        // feed the motors
        m_frontLeft.get()->StopMotors();
        m_frontRight.get()->StopMotors();
        m_backLeft.get()->StopMotors();
        m_backRight.get()->StopMotors();       
    }
    else
    {    
        // scale joystick values to velocities using max chassis values
        auto maxSpeed = GetMaxSpeed();
        auto maxRotation = GetMaxAngularSpeed();

        Logger::GetLogger()->ToNtTable("Swerve Chassis", "MaxSpeed", maxSpeed.to<double>() );
        Logger::GetLogger()->ToNtTable("Swerve Chassis", "maxRotation", maxRotation.to<double>() );

        units::velocity::meters_per_second_t            driveSpeed = drive * maxSpeed;
        units::velocity::meters_per_second_t            steerSpeed = steer * maxSpeed;
        units::angular_velocity::radians_per_second_t   rotateSpeed = rotate * maxRotation;

        Drive( driveSpeed, steerSpeed, rotateSpeed, fieldRelative );
    }
}

/// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
void SwerveChassis::UpdateOdometry() 
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d r2d {yaw};

    m_poseEstimator.Update(r2d, m_frontLeft.get()->GetState(),
                                m_frontRight.get()->GetState(), 
                                m_backLeft.get()->GetState(),
                                m_backRight.get()->GetState());
}

/// @brief set all of the encoders to zero
void SwerveChassis::SetEncodersToZero()
{
    m_frontLeft.get()->SetEncodersToZero();
    m_frontRight.get()->SetEncodersToZero();
    m_backLeft.get()->SetEncodersToZero();
    m_backRight.get()->SetEncodersToZero();
}

double SwerveChassis::GetEncoderValues(std::shared_ptr<SwerveModule> motor)
{
    return motor.get()->GetEncoderValues();
}


/// @brief Provide the current chassis speed information
ChassisSpeeds SwerveChassis::GetChassisSpeeds() const
{
    return m_kinematics.ToChassisSpeeds({ m_frontLeft.get()->GetState(), 
                                          m_frontRight.get()->GetState(),
                                          m_backLeft.get()->GetState(),
                                          m_backRight.get()->GetState() });
}

/// @brief Reset the current chassis pose based on the provided pose and rotation
/// @param [in] const Pose2d&       pose        Current XY position
/// @param [in] const Rotation2d&   angle       Current rotation angle
void SwerveChassis::ResetPosition
( 
    const Pose2d&       pose,
    const Rotation2d&   angle
)
{
    m_poseEstimator.ResetPosition(pose, angle);
}


void SwerveChassis::ResetPosition
( 
    const Pose2d&       pose
)
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d angle {yaw};
    ResetPosition(pose, angle);
}

ChassisSpeeds SwerveChassis::GetFieldRelativeSpeeds
(
    units::meters_per_second_t xSpeed,
    units::meters_per_second_t ySpeed,
    units::radians_per_second_t rot        
)
{
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "xSpeed (mps)", xSpeed.to<double>());
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "ySpeed (mps)", ySpeed.to<double>());
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "rot (radians per sec)", rot.to<double>());

    units::angle::radian_t yaw{m_pigeon->GetYaw()*wpi::math::pi/180.0};
    auto temp = xSpeed*cos(yaw.to<double>()) + ySpeed*sin(yaw.to<double>());
    auto strafe = -1.0*xSpeed*sin(yaw.to<double>()) + ySpeed*cos(yaw.to<double>());
    auto forward = temp;

    ChassisSpeeds output{forward, strafe, rot};

    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "yaw (radians)", yaw.to<double>());
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "forward (mps)", forward.to<double>());
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "stafe (mps)", strafe.to<double>());

    return output;
}

void SwerveChassis::CalcSwerveModuleStates
(
    frc::ChassisSpeeds speeds
)
{
    // These calculations are based on Ether's Chief Delphi derivation
    // The only changes are that that derivation is based on positive angles being clockwise
    // and our codes/sensors are based on positive angles being counter clockwise.

    // A = Vx - omega * L/2
    // B = Vx + omega * L/2
    // C = Vy - omega * W/2
    // D = Vy + omega * W/2
    //
    // Where:
    // Vx is the sideways (strafe) vector
    // Vy is the forward vector
    // omega is the rotation about Z vector
    // L is the wheelbase (front to back)
    // W is the wheeltrack (side to side)
    //
    // Since our Vx is forward and Vy is strafe we need to rotate the vectors
    // We will use these variable names in the code to help tie back to the document.
    // Variable names, though, will follow C++ standards and start with a lower case letter.

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Drive", speeds.vx.to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Strafe", speeds.vy.to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Rotate", speeds.omega.to<double>());

    auto l = GetWheelBase();
    auto w = GetTrack();

    auto vy = 1.0 * speeds.vx;
    auto vx = -1.0 * speeds.vy;
    auto omega = speeds.omega;

    units::velocity::meters_per_second_t omegaL = omega.to<double>() * l / 2.0 / 1_s ;
    units::velocity::meters_per_second_t omegaW = omega.to<double>() * w / 2.0 / 1_s;
    
    auto a = vx - omegaL;
    auto b = vx + omegaL;
    auto c = vy - omegaW;
    auto d = vy + omegaW;

    // here we'll negate the angle to conform to the positive CCW convention
    m_flState.angle = units::angle::radian_t(atan2(b.to<double>(), d.to<double>()));
    m_flState.angle = -1.0 * m_flState.angle.Degrees();
    m_flState.speed = units::velocity::meters_per_second_t(sqrt( pow(b.to<double>(),2) + pow(d.to<double>(),2) ));
    auto maxCalcSpeed = abs(m_flState.speed.to<double>());

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Left Angle", m_flState.angle.Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Left Speed", m_flState.speed.to<double>());

    m_frState.angle = units::angle::radian_t(atan2(b.to<double>(), c.to<double>()));
    m_frState.angle = -1.0 * m_frState.angle.Degrees();
    m_frState.speed = units::velocity::meters_per_second_t(sqrt( pow(b.to<double>(),2) + pow(c.to<double>(),2) ));
    if (abs(m_frState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_frState.speed.to<double>());
    }

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Right Angle", m_frState.angle.Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Right Speed - raw", m_frState.speed.to<double>());

    m_blState.angle = units::angle::radian_t(atan2(a.to<double>(), d.to<double>()));
    m_blState.angle = -1.0 * m_blState.angle.Degrees();
    m_blState.speed = units::velocity::meters_per_second_t(sqrt( pow(a.to<double>(),2) + pow(d.to<double>(),2) ));
    if (abs(m_blState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_blState.speed.to<double>());
    }

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Left Angle", m_blState.angle.Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Left Speed - raw", m_blState.speed.to<double>());

    m_brState.angle = units::angle::radian_t(atan2(a.to<double>(), c.to<double>()));
    m_brState.angle = -1.0 * m_brState.angle.Degrees();
    m_brState.speed = units::velocity::meters_per_second_t(sqrt( pow(a.to<double>(),2) + pow(c.to<double>(),2) ));
    if (abs(m_brState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_brState.speed.to<double>());
    }

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Right Angle", m_brState.angle.Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Right Speed - raw", m_brState.speed.to<double>());


    // normalize speeds if necessary (maxCalcSpeed > max attainable speed)
    if ( maxCalcSpeed > m_maxSpeed.to<double>() )
    {
        auto ratio = m_maxSpeed.to<double>() / maxCalcSpeed;
        m_flState.speed *= ratio;
        m_frState.speed *= ratio;
        m_blState.speed *= ratio;
        m_brState.speed *= ratio;
    }

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Left Speed - normalized", m_flState.speed.to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Right Speed - normalized", m_frState.speed.to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Left Speed - normalized", m_blState.speed.to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Right Speed - normalized", m_brState.speed.to<double>());
}


