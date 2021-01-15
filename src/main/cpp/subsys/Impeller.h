
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

// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <subsys/Mech1IndMotor.h>

// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>

class IDragonMotorController;

class Impeller : public Mech1IndMotor
{
	public:
        Impeller() = delete;
        Impeller
        (
            std::shared_ptr<IDragonMotorController> motor,
            std::shared_ptr<ctre::phoenix::sensors::CANCoder> encoder
        );

        ///@brief Return the current position of the impeller in degrees (positive is cw, negative is ccw)
        ///@return  double position in degrees
        double GetPosition() const override;

        ///@brief Return the current position of the impeller in degrees per second (positive is cw, negative is ccw)
        ///@return double speed in degrees per second
        double GetSpeed() const override;

        private:
        std::shared_ptr<ctre::phoenix::sensors::CANCoder> m_encoder;
};
