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
#pragma once

//C++ Includes
#include <memory>

//Team302 Includes
#include <units/angular_velocity.h>
#include <auton/PrimitiveParams.h>
#include <auton/primitives/IPrimitive.h>

//FRC,WPI Includes

#include <wpi/SmallString.h>
#include <wpi/Path.h>
#include <wpi/math>

#include <frc/geometry/Pose2d.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryConfig.h>

#include <frc/controller/RamseteController.h>
#include <frc/Timer.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <subsys/SwerveModule.h>

#include <subsys/SwerveChassisFactory.h>
#include <subsys/SwerveModule.h>

class SwerveChassis;

namespace frc
{
    class Timer;
}

class DrivePath : public IPrimitive
{
public:
    DrivePath();

    virtual ~DrivePath() = default;

    void Init(PrimitiveParams *params) override;
    void Run() override;
    bool IsDone() override;

private:
    std::shared_ptr<SwerveChassis> m_chassis;
    std::unique_ptr<frc::Timer> m_timer;
 

    double m_maxTime;

    frc::Pose2d m_currentChassisPosition;
    frc::Trajectory m_trajectory;
    frc::RamseteController m_ramseteController;
    frc::Pose2d m_CurPos;                                          // Current position used for motion detect.
    frc::Pose2d m_PrevPos;                                         // previous position of robot for compare to current position.
    frc::Timer m_PosChgTimer;                                      // scan time for position change
    bool m_bRobotStopped = false;                                  // check to see if robot is stopped
    bool lRobotStopped(frc::Pose2d, frc::Pose2d); // routine to check for motion

    bool m_reverse = false;
    bool m_isDone = false;

    std::string sPath2Load = ""; // used to read file name of path.
    std::string lGetGSPathFromVisionTbl();

    bool CheckTarget(double*, double, double, double,double);


    // network table reading
    // Using Default table no referances for Swerve Drive Module m_nt.
    // May need to create a link to m_nt if more than one network table is used.
    nt::NetworkTableInstance inst = nt::NetworkTableInstance().GetDefault();
     
    wpi::Twine sTableName = "visionTable";
    wpi::StringRef sRef_TblCVAngle = "CellVisionAngle";
    wpi::StringRef sRef_TblCVDistance = "CellVisionDistance";
 
    PRIMITIVE_IDENTIFIER m_path;  //m_mode Not sure if this should be unique.
};