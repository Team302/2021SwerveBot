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
#include <units/velocity.h>

//FRC Includes
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

//Team 302 Includes
#include <auton/primitives/IPrimitive.h>
#include <subsys/SwerveModule.h>

class SwerveChassis;
class PrimitiveParams;

class StayInPlace : public IPrimitive
{
    public: 
            void Init(PrimitiveParams*      params) override;
            void Run() override;
            bool IsDone() override;
            StayInPlace();
            virtual ~StayInPlace() = default;

    private:
            std::shared_ptr<SwerveChassis> m_chassis;

            frc::SwerveModuleState m_frontLeftState;
            frc::SwerveModuleState m_frontRightState;
            frc::SwerveModuleState m_backLeftState;
            frc::SwerveModuleState m_backRightState;

            std::shared_ptr<SwerveModule> m_frontLeft;
            std::shared_ptr<SwerveModule> m_frontRight;
            std::shared_ptr<SwerveModule> m_backLeft;
            std::shared_ptr<SwerveModule> m_backRight;

            units::radian_t m_frontLeftRadian;
            units::radian_t m_frontRightRadian;
            units::radian_t m_backLeftRadian;
            units::radian_t m_backRightRadian;

            frc::Rotation2d m_frontLeftRotation;
            frc::Rotation2d m_frontRightRotation;
            frc::Rotation2d m_backLeftRotation;
            frc::Rotation2d m_backRightRotation;

            double m_timeRemaining; //In seconds
};