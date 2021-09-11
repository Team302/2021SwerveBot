

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

// FRC includes

// Team 302 includes

// Third Party Includes
#include <team302/gamepad/IDragonGamePad.h>
#include <team302/gamepad/TeleopControl.h>

class FunctionMap
{
    public:
        FunctionMap();
        FunctionMap
        ( 
            team302::gamepad::TeleopControl::FUNCTION_IDENTIFIER  function,
            int                                 controllerIndex,
            team302::gamepad::IDragonGamePad::AXIS_IDENTIFIER     axisID,
            team302::gamepad::IDragonGamePad::AXIS_DEADBAND       deadBand,
            team302::gamepad::IDragonGamePad::AXIS_PROFILE        profile,
            team302::gamepad::IDragonGamePad::BUTTON_IDENTIFIER   buttonID,
            team302::gamepad::IDragonGamePad::BUTTON_MODE         buttonType
        );
        ~FunctionMap() = default;

        team302::gamepad::TeleopControl::FUNCTION_IDENTIFIER GetFunction() const;
        int GetControllerIndex() const;
        team302::gamepad::IDragonGamePad::AXIS_IDENTIFIER GetAxisID() const;
        team302::gamepad::IDragonGamePad::AXIS_DEADBAND GetDeadband() const;
        team302::gamepad::IDragonGamePad::AXIS_PROFILE GetProfile() const;
        team302::gamepad::IDragonGamePad::BUTTON_IDENTIFIER GetButtonID() const;
        team302::gamepad::IDragonGamePad::BUTTON_MODE GetMode() const;


    private:
        team302::gamepad::TeleopControl::FUNCTION_IDENTIFIER      m_function;
        int                                     m_controllerIndex;
        team302::gamepad::IDragonGamePad::AXIS_IDENTIFIER         m_axisID;
        team302::gamepad::IDragonGamePad::AXIS_DEADBAND           m_axisDeadband;
        team302::gamepad::IDragonGamePad::AXIS_PROFILE            m_axisProfile;
        team302::gamepad::IDragonGamePad::BUTTON_IDENTIFIER       m_buttonID;
        team302::gamepad::IDragonGamePad::BUTTON_MODE             m_buttonMode;
};