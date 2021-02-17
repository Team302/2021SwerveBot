
//====================================================================================================================================================
// Copyright 2021 Lake Orion Robotics FIRST Team 302
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

//========================================================================================================
/// Robot.h
//========================================================================================================
///
/// File Description:
///     Top-level robot code that controls the various states of the robot.  It is our specific 
///     implementation of the frc::TimedRobot.
///
//========================================================================================================

#pragma once

// c++ includes
#include <memory>

// wpilib includes
#include <frc/TimedRobot.h>

// team 302 includes
#include <states/chassis/SwerveDrive.h>
#include <hw/DragonLimelight.h>
#include <vision/DriverMode.h>


// third party includes

/// @class Robot
/// @brief Top-level robot code that controls the various states of the robot.  It is our specific 
///        implementation of the frc::TimedRobot.
class Robot : public frc::TimedRobot 
{
  public:
      Robot() = default;
      ~Robot() = default;

      void RobotInit() override;
      void RobotPeriodic() override;
      void AutonomousInit() override;
      void AutonomousPeriodic() override;
      void TeleopInit() override;
      void TeleopPeriodic() override;
      void TestInit() override;
      void TestPeriodic() override;

  private:
 
    std::shared_ptr<SwerveDrive> m_drive;
    DragonLimelight*             m_limelight;
    DriverMode*                  m_driverMode;
};
