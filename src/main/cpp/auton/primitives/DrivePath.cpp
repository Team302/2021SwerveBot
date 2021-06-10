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

//C++
#include <string>

//FRC Includes
#include <frc/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <units/angular_velocity.h>

// 302 Includes
#include <auton/primitives/DrivePath.h>

using namespace std;
using namespace frc;

using namespace wpi::math;

DrivePath::DrivePath() : m_chassis(SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
                         m_timer(make_unique<Timer>()),
                         m_currentChassisPosition(m_chassis.get()->GetPose()),
                         m_trajectory(),
                         m_runHoloController(false),
                         m_ramseteController(),
                         m_holoController(frc2::PIDController{1, 0, 0},
                                          frc2::PIDController{1, 0, 0},
                                          frc::ProfiledPIDController<units::radian>{1, 0, 0,
                                                                                    frc::TrapezoidProfile<units::radian>::Constraints{6.28_rad_per_s, 3.14_rad_per_s / 1_s}}),
                         //max velocity of 1 rotation per second and a max acceleration of 180 degrees per second squared.
                         m_PrevPos(m_chassis.get()->GetPose()),
                         m_PosChgTimer(make_unique<Timer>()),
                         m_timesRun(0),
                         m_targetPose(),
                         m_deltaX(0.0),
                         m_deltaY(0.0),
                         m_trajectoryStates(),
                         m_desiredState()

{
    m_trajectoryStates.clear();
}
void DrivePath::Init(PrimitiveParams *params)
{

    //Drive mode switcher
    if (params->GetDriveMode() == "RAMSETE")
    {
        m_runHoloController = false;
    }
    else if (params->GetDriveMode() == "HOLONOMIC")
    {
        m_runHoloController = true;
    }
    else
    {
        Logger::GetLogger()->LogError(Logger::GetLogger()->ERROR_ONCE, "DrivePath Drive Mode Switcher", "Invalid drive mode");
    }
    


    auto m_pathname = params->GetPathName();

    m_heading = params->GetHeading();

    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Initialized", "False");
    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Running", "False");
    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Done", "False");
    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "WhyDone", "Not done");
    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Times Ran", 0);

    m_trajectoryStates.clear();

    m_wasMoving = false;

    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Initialized", "True");

    GetTrajectory(params->GetPathName());
    Logger::GetLogger()->ToNtTable(m_pathname + "Trajectory", "Time", m_trajectory.TotalTime().to<double>());
    if (!m_trajectoryStates.empty()) // only go if path name found
    {
        m_desiredState = m_trajectoryStates.front();

        m_timer.get()->Reset();
        m_timer.get()->Start();

        Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosX", m_currentChassisPosition.X().to<double>());
        Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosY", m_currentChassisPosition.Y().to<double>());

        Logger::GetLogger()->ToNtTable("Deltas", "iDeltaX", "0");
        Logger::GetLogger()->ToNtTable("Deltas", "iDeltaX", "0");

        m_PosChgTimer.get()->Reset();
        m_PosChgTimer.get()->Start(); // start scan timer to detect motion

        if (m_runHoloController)
        {
            m_holoController.SetEnabled(true);
            m_ramseteController.SetEnabled(false);
        }
        else
        {
            m_ramseteController.SetEnabled(true);
            m_holoController.SetEnabled(false);
        }
        auto targetState = m_trajectory.Sample(m_trajectory.TotalTime());
        m_targetPose = targetState.pose;
        auto currPose = m_chassis.get()->GetPose();
        auto trans = m_targetPose - currPose;
        m_deltaX = trans.X().to<double>();
        m_deltaY = trans.Y().to<double>();
        m_chassis.get()->RunWPIAlgorithm(true);
    }
    m_timesRun = 0;
}
void DrivePath::Run()
{

    Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Running", "True");

    if (!m_trajectoryStates.empty()) 
    {
        // debugging
        m_timesRun++;
        Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Times Ran", m_timesRun);

        // calculate where we are and where we want to be
        CalcCurrentAndDesiredStates();

        // Use the controller to calculate the chassis speeds for getting there
        auto refChassisSpeeds = m_runHoloController ? m_holoController.Calculate(m_currentChassisPosition, m_desiredState, m_desiredState.pose.Rotation()) :
                                                      m_ramseteController.Calculate(m_currentChassisPosition, m_desiredState);

        // debugging
        Logger::GetLogger()->ToNtTable("DrivePathValues", "ChassisSpeedsX", refChassisSpeeds.vx());
        Logger::GetLogger()->ToNtTable("DrivePathValues", "ChassisSpeedsY", refChassisSpeeds.vy());
        Logger::GetLogger()->ToNtTable("DrivePathValues", "ChassisSpeedsZ", units::degrees_per_second_t(refChassisSpeeds.omega()).to<double>());

        //stop z rotation
        refChassisSpeeds.omega = units::angular_velocity::radians_per_second_t(0);

        //correct z rotation by comparing primitive param heading to pigeon yaw
        //print out error using frc2::PIDController().GetPositionError
        /*
        DragonPigeon* pigeon = PigeonFactory::GetFactory()->GetPigeon();
        bool turn = m_heading == pigeon->GetYaw() ? true : false;
        double delta = pigeon->GetYaw() - m_heading;
        units::angular_velocity::radians_per_second_t speedRads;
         if (turn)
         {
             if (delta < 0)
             {
                 speedRads = units::radians_per_second_t(0.1745);
             }
             else if (delta > 0)
             {
                speedRads = units::radians_per_second_t(-0.1745);
             }
         }
         else
         {
             speedRads = units::radians_per_second_t(0);
         }
         */
         

        // Run the chassis
        m_chassis->Drive(refChassisSpeeds, false);
    }
    else
    {
        m_chassis->Drive(0, 0, 0, false);
    }

}

bool DrivePath::IsDone()
{

    bool isDone = false;
    string whyDone = "";
    
    if (!m_trajectoryStates.empty()) 
    {
        // Check if the current pose and the trajectory's final pose are the same
        auto curPos = m_chassis.get()->GetPose();
        //isDone = IsSamePose(curPos, m_targetPose, 100.0);
        if (IsSamePose(curPos, m_targetPose, 10.0))
        {
            isDone = true;
            whyDone = "Current Pose = Trajectory final pose";
        }
        

        
        if ( !isDone )
        {
            // Now check if the current pose is getting closer or farther from the target pose 
            auto trans = m_targetPose - curPos;
            auto thisDeltaX = trans.X().to<double>();
            auto thisDeltaY = trans.Y().to<double>();
            if (abs(thisDeltaX) < m_deltaX && abs(thisDeltaY) < m_deltaY)
            {   // Getting closer so just update the deltas
                m_deltaX = thisDeltaX;
                m_deltaY = thisDeltaY;
            }
            else
            {   // Getting farther away:  determine if it is because of the path curvature (not straight line between start and the end)
                // or because we went past the target (in this case, we are done)
                // Assume that once we get within a tenth of a meter (just under 4 inches), if we get
                // farther away we are passing the target, so we should stop.  Otherwise, keep trying.
                isDone = ((abs(m_deltaX) < 0.1 && abs(m_deltaY) < 0.1));
                if ((abs(m_deltaX) < 0.1 && abs(m_deltaY) < 0.1))
                {
                    whyDone = "Within 4 inches of target or getting farther away from target";  
                }
            }
        }       
        

        


        if (m_PosChgTimer.get()->Get() > 1.0)
        {
           //auto moving = !IsSamePose(curPos, m_PrevPos, 7.5);
            auto moving = m_chassis.get()->IsMoving();
            if (!moving && m_wasMoving)
            {
                    isDone = true;
                    whyDone = "Stopped moving";                    
            }
            m_PrevPos = curPos;
            m_wasMoving = moving;
        }

        // finally, do it based on time (we have no more states);  if we want to keep 
        // going, we need to understand where in the trajectory we are, so we can generate
        // a new state.
        if (!isDone)
        {
            //return (units::second_t(m_timer.get()->Get()) >= m_trajectory.TotalTime()); 
        }
    }
    else
    {
        Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Done", "True");
        return true;
    }
    if (isDone)
    {
        Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "Done", "True");
        Logger::GetLogger()->ToNtTable("DrivePath" + m_pathname, "WhyDone", whyDone);
        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::PRINT, "DrivePath" + m_pathname, "Is done because: " + whyDone);
    }
    return isDone;
    
}

bool DrivePath::IsSamePose(frc::Pose2d lCurPos, frc::Pose2d lPrevPos, double tolerance)
{
    // Detect if the two poses are the same within a tolerance
    double dCurPosX = lCurPos.X().to<double>() * 100; //cm
    double dCurPosY = lCurPos.Y().to<double>() * 100;
    double dPrevPosX = lPrevPos.X().to<double>() * 100;
    double dPrevPosY = lPrevPos.Y().to<double>() * 100;

    double dDeltaX = abs(dPrevPosX - dCurPosX);
    double dDeltaY = abs(dPrevPosY - dCurPosY);

    Logger::GetLogger()->ToNtTable("Deltas", "iDeltaX", to_string(dDeltaX));
    Logger::GetLogger()->ToNtTable("Deltas", "iDeltaY", to_string(dDeltaY));
    Logger::GetLogger()->ToNtTable("Deltas", "curPosX", to_string(dCurPosX));
    Logger::GetLogger()->ToNtTable("Deltas", "curPosY", to_string(dCurPosY));
    Logger::GetLogger()->ToNtTable("Deltas", "prevPosX", to_string(dPrevPosX));
    Logger::GetLogger()->ToNtTable("Deltas", "prevPosY", to_string(dPrevPosY));

    //  If Position of X or Y has moved since last scan..  Using Delta X/Y
    return (dDeltaX <= tolerance && dDeltaY <= tolerance);
}

void DrivePath::GetTrajectory
(
    string  path
)
{
    if (!path.empty()) // only go if path name found
    {
        Logger::GetLogger()->LogError(string("DrivePath" + m_pathname), string("Finding Deploy Directory"));

        // Read path into trajectory for deploy directory.  JSON File ex. Bounce1.wpilid.json
        wpi::SmallString<64> deployDir;
        frc::filesystem::GetDeployDirectory(deployDir);
        wpi::sys::path::append(deployDir, "paths");
        wpi::sys::path::append(deployDir, path); // load path from deploy directory

        Logger::GetLogger()->LogError(string("Deploy path is "), deployDir.str());
        m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDir);
        m_trajectoryStates = m_trajectory.States();
        Logger::GetLogger()->LogError(string("DrivePath - Loaded = "), path);
        Logger::GetLogger()->ToNtTable("DrivePathValues", "TrajectoryTotalTime", m_trajectory.TotalTime().to<double>());
    }
}

void DrivePath::CalcCurrentAndDesiredStates()
{
    m_currentChassisPosition = m_chassis.get()->GetPose();
    auto sampleTime = units::time::second_t(m_timer.get()->Get()); //+ 0.02

    m_desiredState = m_trajectory.Sample(sampleTime);

    // May need to do our own sampling based on position and time     

    Logger::GetLogger()->ToNtTable("DrivePathValues", "DesiredPoseX", m_desiredState.pose.X().to<double>());
    Logger::GetLogger()->ToNtTable("DrivePathValues", "DesiredPoseY", m_desiredState.pose.Y().to<double>());
    Logger::GetLogger()->ToNtTable("DrivePathValues", "DesiredPoseOmega", m_desiredState.pose.Rotation().Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosX", m_currentChassisPosition.X().to<double>());
    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosY", m_currentChassisPosition.Y().to<double>());
    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentPosOmega", m_currentChassisPosition.Rotation().Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("DeltaValues", "DeltaX", m_desiredState.pose.X().to<double>() - m_currentChassisPosition.X().to<double>());
    Logger::GetLogger()->ToNtTable("DeltaValues", "DeltaY", m_desiredState.pose.Y().to<double>() - m_currentChassisPosition.Y().to<double>());

    Logger::GetLogger()->ToNtTable("DrivePathValues", "CurrentTime", m_timer.get()->Get());
}
