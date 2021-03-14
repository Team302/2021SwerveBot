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
#include <frc/controller/HolonomicDriveController.h>
#include <frc/Timer.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

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
    bool IsSamePose(frc::Pose2d, frc::Pose2d, double tolerance); // routine to check for motion
    void GetTrajectory(std::string  path);
    void CalcCurrentAndDesiredStates();



    std::shared_ptr<SwerveChassis>          m_chassis;
    std::unique_ptr<frc::Timer>             m_timer;

    frc::Pose2d                             m_currentChassisPosition;
    frc::Trajectory                         m_trajectory;
    bool                                    m_runHoloController;
    bool                                    m_wasMoving;
    frc::RamseteController                  m_ramseteController;
    frc::HolonomicDriveController           m_holoController;
    frc::Pose2d                             m_PrevPos;          // previous position of robot for compare to current position.
    std::unique_ptr<frc::Timer>             m_PosChgTimer;      // scan time for position change
    int                                     m_timesRun;
    frc::Pose2d                             m_targetPose;
    std::string                             m_pathname;
    double                                  m_deltaX;
    double                                  m_deltaY;
    std::vector<frc::Trajectory::State>     m_trajectoryStates;
    frc::Trajectory::State                  m_desiredState;
 
};