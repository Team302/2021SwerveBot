//====================================================================================================================================================
// Copyright 2019 Lake Orion Robotics FIRST Team 302
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

//C++ Includes
#include <memory>
#include <string>

//Team 302 includes
#include <auton/primitives/ResetPosition.h>
#include <auton/PrimitiveParams.h>
#include <auton/primitives/IPrimitive.h>
#include <subsys/SwerveChassisFactory.h>
#include <hw/factories/PigeonFactory.h>

using namespace std;
using namespace frc;

ResetPosition::ResetPosition() : m_chassis(SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis())
{
}

void ResetPosition::Init(PrimitiveParams* params)
{
    string pathToLoad = params->GetPathName();

    if (pathToLoad != "")
    {
        wpi::SmallString<64> deployDir;
        frc::filesystem::GetDeployDirectory(deployDir);
        wpi::sys::path::append(deployDir, "paths");
        wpi::sys::path::append(deployDir, pathToLoad);

        m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDir);

        frc::Rotation2d StartAngle;
        StartAngle.Degrees() = m_trajectory.InitialPose().Rotation().Degrees();

        m_chassis->SetEncodersToZero();

        m_chassis->ResetPosition(m_trajectory.InitialPose(), StartAngle);

        PigeonFactory::GetFactory()->GetPigeon()->ReZeroPigeon(0, 0);

        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::PRINT, "ResetPosX", to_string(m_chassis.get()->GetPose().X().to<double>()));
        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::PRINT, "ResetPosY", to_string(m_chassis.get()->GetPose().Y().to<double>()));
        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::PRINT, "InitialPoseX", to_string(m_trajectory.InitialPose().X().to<double>()));
        Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::PRINT, "InitialPoseY", to_string(m_trajectory.InitialPose().Y().to<double>()));
        
    }
}

void ResetPosition::Run()
{

}

bool ResetPosition::IsDone()
{
    return true;
}