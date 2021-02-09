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
#include <auton/primitives/IPrimitive.h>
#include <units/angular_velocity.h>

//FRC Includes
#include <frc/geometry/Pose2d.h>

#include <wpi/SmallString.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <frc/controller/RamseteController.h> //geo3


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

        void Init(PrimitiveParams* params) override;
        void Run() override;
        bool IsDone() override;
      
        private:
            std::shared_ptr<SwerveChassis> m_chassis;
            std::unique_ptr<frc::Timer> m_timer;

            double                   m_maxTime;

            frc::Pose2d              m_currentChassisPosition;
            frc::Trajectory          m_trajectory;  
            frc::RamseteController   m_ramseteController;

            bool                     m_reverse = false;
            bool                     m_isDone = false;

            PRIMITIVE_IDENTIFIER     m_mode;

};