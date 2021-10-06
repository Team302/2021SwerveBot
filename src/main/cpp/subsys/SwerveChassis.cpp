
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
#include <subsys/PoseEstimatorEnum.h>
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
    units::length::inch_t                                       wheelDiameter,
    units::length::inch_t                                       wheelBase,
    units::length::inch_t                                       track,
    double                                                      odometryComplianceCoefficient,
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
    m_wheelDiameter(wheelDiameter),
    m_wheelBase(wheelBase),
    m_track(track),
    m_odometryComplianceCoefficient(odometryComplianceCoefficient),
    m_maxSpeed(maxSpeed),
    m_maxAngularSpeed(maxAngularSpeed),
    m_maxAcceleration(maxAcceleration),
    m_maxAngularAcceleration(maxAngularAcceleration),
    m_pigeon(PigeonFactory::GetFactory()->GetPigeon()),
    m_accel(BuiltInAccelerometer()),
    m_isMoving(false),
    m_scale(1.0),
    m_boost(0.0),
    m_brake(0.0),
    m_runWPI(false),
    m_poseOpt(PoseEstimationMethod::EULER_AT_CHASSIS),
    m_pose(),
    m_offsetPoseAngle(0_deg),
    m_timer(),
    m_drive(units::velocity::meters_per_second_t(0.0)),
    m_steer(units::velocity::meters_per_second_t(0.0)),
    m_rotate(units::angular_velocity::radians_per_second_t(0.0)),
    m_rotateOffset(frc::Translation2d()),
    m_frontLeftLocation(wheelBase/2.0, track/2.0),
    m_frontRightLocation(wheelBase/2.0, -1.0*track/2.0),
    m_backLeftLocation(-1.0*wheelBase/2.0, track/2.0),
    m_backRightLocation(-1.0*wheelBase/2.0, -1.0*track/2.0)
{
    m_timer.Reset();
    m_timer.Start();

    frontLeft.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_frontLeftLocation );
    frontRight.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_frontRightLocation );
    backLeft.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_backLeftLocation );
    backRight.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_backRightLocation );

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

void SwerveChassis::SetBoost( double boost )
{
    m_boost = boost;
    m_frontLeft.get()->SetBoost(m_boost);
    m_frontRight.get()->SetBoost(m_boost);
    m_backLeft.get()->SetBoost(m_boost);
    m_backRight.get()->SetBoost(m_boost);
}

void SwerveChassis::SetBrake( double brake )
{
    m_brake = brake;
    m_frontLeft.get()->SetBrake(m_brake);
    m_frontRight.get()->SetBrake(m_brake);
    m_backLeft.get()->SetBrake(m_brake);
    m_backRight.get()->SetBrake(m_brake);
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
        m_drive = units::velocity::meters_per_second_t(0.0);
        m_steer = units::velocity::meters_per_second_t(0.0);
        m_rotate = units::angular_velocity::radians_per_second_t(0.0);
        m_isMoving = false;
    }
    else
    {   
        m_drive = units::velocity::meters_per_second_t(xSpeed*(m_scale+m_boost));
        m_steer = units::velocity::meters_per_second_t(ySpeed*(m_scale+m_boost));
        m_rotate = units::angular_velocity::radians_per_second_t(rot*(m_scale+m_boost));

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
            auto ax = m_accel.GetX();
            auto ay = m_accel.GetY();
            auto az = m_accel.GetZ();

            m_isMoving = (abs(ax) > 0.0 || abs(ay) > 0.0 || abs(az) > 0.0 );
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

            auto ax = m_accel.GetX();
            auto ay = m_accel.GetY();
            auto az = m_accel.GetZ();

            m_isMoving = (abs(ax) > 0.0 || abs(ay) > 0.0 || abs(az) > 0.0 );
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

void SwerveChassis::Drive(double drive, double steer, double rotate, bool fieldRelative, frc::Translation2d rotateOffset )
{
    m_rotateOffset = rotateOffset;

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

Pose2d SwerveChassis::GetPose() const
{
    if (m_poseOpt==PoseEstimationMethod::WPI)
    {
        return m_poseEstimator.GetEstimatedPosition();
    }
    return m_pose;
}

/// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
void SwerveChassis::UpdateOdometry() 
{
   // if ( !IsMoving() )  // not moving, so odometry isn't changing
   // {
   //    return;
   // }

    units::degree_t yaw{m_pigeon->GetYaw()};
    Rotation2d rot2d {yaw+m_offsetPoseAngle};
    Rotation2d realAngle {yaw};

    if (m_poseOpt == PoseEstimationMethod::WPI)
    {
        auto currentPose = m_poseEstimator.GetEstimatedPosition();
        Logger::GetLogger()->ToNtTable("Robot Odometry", "Current X", currentPose.X().to<double>());
        Logger::GetLogger()->ToNtTable("Robot Odometry", "Current Y", currentPose.Y().to<double>());

        m_poseEstimator.Update(rot2d, m_frontLeft.get()->GetState(),
                                      m_frontRight.get()->GetState(), 
                                      m_backLeft.get()->GetState(),
                                      m_backRight.get()->GetState());

        auto updatedPose = m_poseEstimator.GetEstimatedPosition();
        Logger::GetLogger()->ToNtTable("Robot Odometry", "Updated X", updatedPose.X().to<double>());
        Logger::GetLogger()->ToNtTable("Robot Odometry", "Updated Y", updatedPose.Y().to<double>());
    }
    else if (m_poseOpt==PoseEstimationMethod::EULER_AT_CHASSIS)
    {
        // get change in time
        auto deltaT = m_timer.Get();
        m_timer.Reset();

        // get the information from the last pose 
        auto startX = m_pose.X();
        auto startY = m_pose.Y();

        // xk+1 = xk + vk cos θk T
        // yk+1 = yk + vk sin θk T
        // Thetak+1 = Thetagyro,k+1
        units::angle::radian_t rads = yaw;          // convert angle to radians
        double cosAng = cos(rads.to<double>());
        double sinAng = sin(rads.to<double>());
        auto vx = m_drive * cosAng + m_steer * sinAng;
        auto vy = m_drive * sinAng + m_steer * cosAng;

        units::length::meter_t currentX = startX + m_odometryComplianceCoefficient*(vx * deltaT);
        units::length::meter_t currentY = startY + m_odometryComplianceCoefficient*(vy * deltaT);

        Pose2d currPose{currentX, currentY, rot2d};
        auto trans = currPose - m_pose;
        m_pose += trans;
    }
    else if (m_poseOpt==PoseEstimationMethod::EULER_USING_MODULES ||
             m_poseOpt==PoseEstimationMethod::POSE_EST_USING_MODULES)
    {
        auto flPose = m_frontLeft.get()->GetCurrentPose(m_poseOpt);
        auto frPose = m_frontRight.get()->GetCurrentPose(m_poseOpt);
        auto blPose = m_backLeft.get()->GetCurrentPose(m_poseOpt);
        auto brPose = m_backRight.get()->GetCurrentPose(m_poseOpt);

        auto chassisX = (flPose.X() + frPose.X() + blPose.X() + brPose.X()) / 4.0;
        auto chassisY = (flPose.Y() + frPose.Y() + blPose.Y() + brPose.Y()) / 4.0;
        Pose2d currPose{chassisX, chassisY, rot2d};
        auto trans = currPose - m_pose;
        m_pose += trans;


        // Get the swerve modules the correct position from the resolved pose
        /**
        Transform2d t_fl {m_frontLeftLocation,realAngle};
        flPose = m_pose + t_fl;
        m_frontLeft.get()->UpdateCurrPose(flPose.X(), flPose.Y());

        Transform2d t_fr {m_frontRightLocation,realAngle};
        frPose = m_pose + t_fr;
        m_frontRight.get()->UpdateCurrPose(frPose.X(), frPose.Y());

        Transform2d t_bl {m_backLeftLocation,realAngle};
        blPose = m_pose + t_bl;
        m_backLeft.get()->UpdateCurrPose(blPose.X(), blPose.Y());

        Transform2d t_br {m_backRightLocation,realAngle};
        brPose = m_pose + t_br;
        m_backRight.get()->UpdateCurrPose(brPose.X(), brPose.Y());
        **/
    }
    
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
    auto trans = pose - m_pose;
    m_pose += trans;

    m_offsetPoseAngle = units::angle::degree_t(m_pigeon->GetYaw()) - angle.Degrees();

    Transform2d t_fl {m_frontLeftLocation,angle};
    auto flPose = pose + t_fl;
    m_frontLeft.get()->UpdateCurrPose(flPose.X(), flPose.Y());

    Transform2d t_fr {m_frontRightLocation,angle};
    auto frPose = m_pose + t_fr;
    m_frontRight.get()->UpdateCurrPose(frPose.X(), frPose.Y());

    Transform2d t_bl {m_backLeftLocation,angle};
    auto blPose = m_pose + t_bl;
    m_backLeft.get()->UpdateCurrPose(blPose.X(), blPose.Y());

    Transform2d t_br {m_backRightLocation,angle};
    auto brPose = m_pose + t_br;
    m_backRight.get()->UpdateCurrPose(brPose.X(), brPose.Y());
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


    /*----Change center of rotation on robot 10/4/2021----

        (L/2, W/2) = center of robot = (0,0)
        Feeding in a different value for these does not change center of rotation, only scales the robot
                            Track(L)
        (B,D)Wheel2*(L, 0)--------(L,W)*Wheel1(B,C)
                              |
                              *Center (L/2,W/2)
                              |Wheelbase(W)
        (A,D)Wheel3*(0, 0)--------(0,W)*Wheel4(A,C)

        Give center of rotation as Point2d(x and y), then find where that is on the robot
        !!May not need the side measurements W, x, y, z = sides  W = left, X = right, Y = top, Z = bottom
        S, t, u, v = wheels S = Top Right wheel , T = Top left wheel , U = Bottom left wheel , V = bottom right wheel


        (0, 0) = 0 * L, 0 * W
        (100, 100) = 1 * L, 1 * W

        !!May not need the side measurements
        If COR(Center of Rotation) = (25, 25):  !!This is only a length, not a coordinate
        W length from COR = x. (. = as decimal) * L  -- For example: 0.25 * L(14)
        X length from COR = 1 - x. * L -- 0.25 * L(14) 
        Y length from COR = 1 - y. * W -- 0.25 * W(14)
        Z length from COR = y. * W -- 0.25 * L(14)

        !! X = Track, Y = Wheelbase
        S Equations -- S on coordinate grid is (100, 100), distance from COR is (S.x - COR.x) * L, (S.y - COR.y) * W
        V1x = Vx + omega(L(S.x. - COR.x.))
        V1y = Vy - omega(W(S.y. - COR.y.))

        V1x = Vx + omega(L(1.00 - 0.25))
        V1y = Vy - omega(W(1.00 - 0.25))

        T Equations -- T on grid is (0, 100)
        V2x = Vx + omega(L(T.x. - COR.x.))
        V2y = Vy + omega(W(T.y. - COR.y.))

        V2x = Vx + omega(L(0 - 0.25))
        V2y = Vy + omega(W(100 - 0.25))

        U Equations -- U on grid is (0, 0)
        V3x = Vx - omega(L(0 - COR.x.))
        V3y = Vy + omega(W(0 - COR.y.))

        V3x = Vx - omega(L(W.x. - 0.25))
        V3y = Vy + omega(W(W.y. - 0.25))

        V Equations -- V on grid is (100, 0)
        V4x = Vx + omega(L(V.x. - COR.x.))
        V4y = Vy + omega(W(V.y. - COR.y.))

        V4x = Vx + omega(L(100 - 0.25))
        V4y = Vy + omega(W(0 - 0.25))

        Code that needs to written:
        Done S, T, U, V need to be defined, Translation2d
        Done Map S, T, U, V to grid
        Done Define Center of Rotation, Translation2d
        Re-work code to use individual equations for each wheel, opposed to a, b, c, d
        Done Make sure STUV.x and .y can be converted to decimal, probably just use * 0.01

        NEW CODE:

        Translation2d SWheelGrid = new Translation2d(100, 100);
        Translation2d TWheelGrid = new Translation2d(0, 100);
        Translation2d UWheelGrid = new Translation2d(0, 0);
        Translation2d VWheelGrid = new Translation2d(100, 0);

        Translation2d m_centerOfRotationGrid; will be fed from Drive Function

        !!V#x = B, A
        !!V#y = C, D
        m_flState.speed = units::velocity::meters_per_second_t(sqrt( pow(b.to<double>(),2) + pow(d.to<double>(),2) ));
        ->
        (T)m_flState.speed = units::velocity::meters_per_second_t(sqrt( pow((Vx + omega(L(T.x. - COR.x.)).to<double>())),2) + pow((V2y = Vy + omega(W(T.y. - COR.y.)).to<double>())),2) )
        m_flState.angle = units::angle::radian_t(atan2((Vx + omega(L(T.x. - COR.x.)).to<double>()), (V2y = Vy + omega(W(T.y. - COR.y.)).to<double>())));
        ->
        m_flState.angle = units::angle::radian_t((Vx + omega(L(T.x. - COR.x.)).to<double>()), (V2y = Vy + omega(W(T.y. - COR.y.)).to<double>())));
    */


    

    //original code from main
    units::meter_t omegaL = omega.to<double>() * l / 2.0 / 1_s; 
    units::meter_t omegaW = omega.to<double>() * w / 2.0 / 1_s;   

    //Debugging for turn around point
    /*
    Logger::GetLogger()->ToNtTable("ATurnAbout", "omegaL MPS", omegaL.to<double>());
    Logger::GetLogger()->ToNtTable("ATurnAbout", "omegaW MPS", omegaW.to<double>());
    Logger::GetLogger()->ToNtTable("ATurnAbout", "rotateOffset.x", m_rotateOffset.X().to<double>());
    Logger::GetLogger()->ToNtTable("ATurnAbout", "rotateOffset.y", m_rotateOffset.Y().to<double>());
    Logger::GetLogger()->ToNtTable("ATurnAbout", "l / WheelBase", l.to<double>());
    Logger::GetLogger()->ToNtTable("ATurnAbout", "w / Track", w.to<double>());  */

    //rotateOffset.x and y should be equal to l and w

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

