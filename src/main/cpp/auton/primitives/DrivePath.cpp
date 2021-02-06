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
#include <algorithm>
#include <memory>
#include <string>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/acceleration.h>

//FRC Includes
#include <frc/Timer.h>
#include <wpi/math>
#include <frc/geometry/Translation2d.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>



//Team 302 Includes
#include <auton/PrimitiveParams.h>
#include <auton/primitives/IPrimitive.h>
#include <auton/primitives/DrivePath.h> //Geo3
#include <controllers/ControlData.h>
#include <controllers/ControlModes.h>
#include <hw/DragonPigeon.h>
#include <hw/factories/PigeonFactory.h>
#include <subsys/SwerveChassisFactory.h>
#include <subsys/SwerveChassis.h>
#include <utils/Logger.h>



using namespace std;
using namespace frc;

using namespace wpi::math;



DrivePath::DrivePath() : m_chassis( SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()),
                         m_timer( make_unique<Timer>() ),
                         m_maxTime(0.0),
                         m_currentChassisPosition(units::meter_t(0), units::meter_t(0), units::radian_t(0))
{
}


void DrivePath::Init(  PrimitiveParams* params)
{

  wpi::SmallString<64> deployDir;
  frc::filesystem::GetDeployDirectory(deployDir);
  wpi::sys::path::append(deployDir, "paths");
  wpi::sys::path::append(deployDir, params->GetPathName() );
  frc::Trajectory Mytraj_1 = frc::TrajectoryUtil::FromPathweaverJson(deployDir);


    m_timer->Reset();
    // Reset the drivetrain's odometry to the starting pose of the trajectory.
   //  m_chassis->ResetOdometry(m_trajectory.InitialPose());

   
//  if( params->GetID() == DRIVE_PATH){}

}

void DrivePath::Run()
{
    // Update odometry.
    //m_chassis->UpdateOdometry();  done in robot.cpp 
    auto desiredPose = m_trajectory.Sample(units::second_t(m_timer.get()->Get());

      // Get the reference chassis speeds from the Ramsete Controller.
    
    auto refChassisSpeeds = 
          m_ramseteController.Calculate(m_currentChassisPosition, desiredPose);
                                        
    units::angular_velocity::radians_per_second_t radianTrapezoidSpeed(m_setpoint.velocity.to<double>() * m_distanceToAngleConversion);
                            
    m_chassis->Drive(refChassisSpeeds,false);
 
    }
     
}

bool DrivePath::IsDone()
{

    return units::second_t(m_timer.get()->Get()) >= m_trajectory.TotalTime();

}



