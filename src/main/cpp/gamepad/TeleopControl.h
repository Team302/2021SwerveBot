
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
#include <map>


// FRC includes
#include <frc/Driverstation.h>

// Team 302 includes
#include <gamepad/IDragonGamePad.h>
#include <gamepad/DragonXbox.h>
#include <gamepad/DragonGamePad.h>

// Third Party Includes

class TeleopControl
{
    public:

        enum FUNCTION_IDENTIFIER
        {
            UNKNOWN_FUNCTION,
            SWERVE_DRIVE_DRIVE,
            SWERVE_DRIVE_ROTATE,
            SWERVE_DRIVE_STEER,
            DRIVE_FULL,
            DRIVE_75PERCENT,
            DRIVE_50PERCENT,
            DRIVE_25PERCENT,
            DRIVE_SHIFT_UP,
            DRIVE_SHIFT_DOWN,
            DRIVE_TURBO,
            DRIVE_NITRO,
            INTAKE_ON,
            INTAKE_OFF,
            BALL_TRANSFER_OFF,
            BALL_TRANSFER_TO_SHOOTER,
            SHOOTER_PREPARE_TO_SHOOT,
            SHOOTER_AUTO_SHOOT,
            SHOOTER_MANUAL_AIM,
            SHOOTER_MANUAL_ADJUST_DISTANCE,
            SHOOTER_MANUAL_SHOOT_GREEN,
            SHOOTER_MANUAL_SHOOT_YELLOW,
            SHOOTER_MANUAL_SHOOT_BLUE,
            SHOOTER_MANUAL_SHOOT_RED,
            SHOOTER_OFF,
            TURRET_MANUAL_AXIS,
            TURRET_MANUAL_BUTTON,
            TURRET_LIMELIGHT_AIM,
            REZERO_PIGEON,
            OFF,
            INTAKE,
            INTAKE_HUMAN_PLAYER,
            HOLD,
            GET_READY_TO_SHOOT,
            SHOOT,
            TRANSFER_UP,
            TRANSFER_DOWN,
            MAX_FUNCTIONS
        };


        //----------------------------------------------------------------------------------
        // Method:      GetInstance
        // Description: If there isn't an instance of this class, it will create one.  The
        //              single class instance will be returned.
        // Returns:     OperatorInterface*  instance of this class
        //----------------------------------------------------------------------------------
        static TeleopControl* GetInstance();


        //------------------------------------------------------------------
        // Method:      SetScaleFactor
        // Description: Allow the range of values to be set smaller than
        //              -1.0 to 1.0.  By providing a scale factor between 0.0
        //              and 1.0, the range can be made smaller.  If a value
        //              outside the range is provided, then the value will
        //              be set to the closest bounding value (e.g. 1.5 will
        //              become 1.0)
        // Returns:     void
        //------------------------------------------------------------------
        void SetAxisScaleFactor
        (
            TeleopControl::FUNCTION_IDENTIFIER  axis,          // <I> - axis number to update
            double                              scaleFactor    // <I> - scale factor used to limit the range
        );

        void SetDeadBand
        (
            TeleopControl::FUNCTION_IDENTIFIER  axis,
            IDragonGamePad::AXIS_DEADBAND       deadband
        );

        void SetSlewRateLimiter
        (
            TeleopControl::FUNCTION_IDENTIFIER  axis,
            double                               slewRateLimiter
        );

        //------------------------------------------------------------------
        // Method:      SetAxisProfile
        // Description: Sets the axis profile for the specifed axis
        // Returns:     void
        //------------------------------------------------------------------
        void SetAxisProfile
        (
            TeleopControl::FUNCTION_IDENTIFIER      axis,       // <I> - axis number to update
			IDragonGamePad::AXIS_PROFILE			profile     // <I> - profile to use
        );

        //------------------------------------------------------------------
        // Method:      GetAxisValue
        // Description: Reads the joystick axis, removes any deadband (small
        //              value) and then scales as requested.
        // Returns:     double   -  scaled axis value
        //------------------------------------------------------------------
        double GetAxisValue
        (
            TeleopControl::FUNCTION_IDENTIFIER     axis // <I> - axis number to update
        ) const;

        //------------------------------------------------------------------
        // Method:      GetRawButton
        // Description: Reads the button value.  Also allows POV, bumpers,
        //              and triggers to be treated as buttons.
        // Returns:     bool   -  scaled axis value
        //------------------------------------------------------------------
        bool IsButtonPressed
        (
            TeleopControl::FUNCTION_IDENTIFIER button   // <I> - button number to query
        ) const;


    private:
        //----------------------------------------------------------------------------------
        // Method:      OperatorInterface <<constructor>>
        // Description: This will construct and initialize the object
        //----------------------------------------------------------------------------------
        TeleopControl();

        //----------------------------------------------------------------------------------
        // Method:      ~OperatorInterface <<destructor>>
        // Description: This will clean up the object
        //----------------------------------------------------------------------------------
        virtual ~TeleopControl() = default;

        //----------------------------------------------------------------------------------
        // Attributes
        //----------------------------------------------------------------------------------
        static TeleopControl*               m_instance; // Singleton instance of this class

        std::map<FUNCTION_IDENTIFIER, IDragonGamePad::AXIS_IDENTIFIER> m_axisMap;
        std::map<FUNCTION_IDENTIFIER, IDragonGamePad::BUTTON_IDENTIFIER> m_buttonMap;
        std::map<FUNCTION_IDENTIFIER, int> m_controllerMap;

        std::vector<IDragonGamePad::AXIS_IDENTIFIER>     m_axisIDs;
        std::vector<IDragonGamePad::BUTTON_IDENTIFIER>   m_buttonIDs;
        std::vector<int>							     m_controllerIndex;

        IDragonGamePad*			            m_controllers[frc::DriverStation::kJoystickPorts];

        mutable int                         m_count;
};

