
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
#include <string>
#include <vector>

// FRC includes
#include <networktables/NetworkTable.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

// Team 302 includes
#include <hw/interfaces/IDragonSensor.h>
#include <hw/interfaces/IDragonDistanceSensor.h>

// Third Party Includes


class DragonLimelight //: public IDragonSensor, public IDragonDistanceSensor
{
    public:
        //Enums
        enum LED_MODE 
        {
            LED_DEFAULT,
            LED_OFF,
            LED_BLINK,
            LED_ON
        };

        enum CAM_MODE
        {
            CAM_VISION,
            CAM_DRIVER
        };

        enum STREAM_MODE
        {
            STREAM_DEFAULT,  // side by side if two cams
            STREAM_MAIN_AND_SECOND, // Second Cam bottom right of Main Cam
            STREAM_SECOND_AND_MAIN // Main Cam bottom right of Second Cam
        };

        enum SNAPSHOT_MODE
        {
            SNAP_OFF,
            SNAP_ON
        };

        ///-----------------------------------------------------------------------------------
        /// Method:         DragonLimelight (constructor)
        /// Description:    Create the object
        ///-----------------------------------------------------------------------------------
        DragonLimelight() = delete;
        DragonLimelight
        (
            std::string                 tableName,                  /// <I> - network table name
            units::length::inch_t       mountingHeight,             /// <I> - mounting height of the limelight
            units::length::inch_t       mountingHorizontalOffset,   /// <I> - mounting horizontal offset from the middle of the robot
            units::angle::degree_t      rotation,                   /// <I> - clockwise rotation of limelight
            units::angle::degree_t      mountingAngle,              /// <I> - mounting angle of the camera
            units::length::inch_t       targetHeight,               /// <I> - height the target
            units::length::inch_t       targetHeight2               /// <I> - height of second target
        );

        ///-----------------------------------------------------------------------------------
        /// Method:         ~DragonLimelight (destructor)
        /// Description:    Delete the object
        ///-----------------------------------------------------------------------------------
        ~DragonLimelight() = default;


        // Getters
        bool HasTarget() const;
        units::angle::degree_t GetTargetHorizontalOffset() const;
        units::angle::degree_t GetTargetVerticalOffset() const;
        double GetTargetArea() const;
        units::angle::degree_t GetTargetSkew() const;
        units::time::microsecond_t GetPipelineLatency() const;
        units::length::inch_t EstimateTargetDistance() const;
        std::vector<double> Get3DSolve() const;

        // Setters
        void SetTargetHeight
        (
            units::length::inch_t targetHeight
        );

        void SetLEDMode
        (
            DragonLimelight::LED_MODE mode // 0-Default, 1-Off, 2-Blink, 3-On 
        );

        void SetCamMode
        (
            DragonLimelight::CAM_MODE mode // 0-Vision, 1-Driver
        );

        void SetPipeline
        (
            int pipeline // 0-9 
        );

        void SetStreamMode
        (
            DragonLimelight::STREAM_MODE mode // 0-Side By Side, 1-Second cam bottom right of main, 2-Main bottom right second
        );

        void ToggleSnapshot
        (
            DragonLimelight::SNAPSHOT_MODE toggle // 0-No snapshots 1- two snapshots/second: Max of 32 saved
        );

        void SetCrosshairPos
        (
            double crosshairPosX,
            double crosshairPosY
        );

        void SetSecondaryCrosshairPos
        (
            double crosshairPosX,
            double crosshairPosY
        );

        void PrintValues(); // Prints out all values to ensure everything is working and connected

        units::angle::degree_t GetMountingAngle() const {return m_mountingAngle;}
        units::length::inch_t  GetMountingHeight() const {return m_mountHeight;}
        units::length::inch_t  GetTargetHeight() const {return m_targetHeight;}

    private:
        std::shared_ptr<nt::NetworkTable> m_networktable;
        units::length::inch_t m_mountHeight;
        units::length::inch_t m_mountingHorizontalOffset;
        units::angle::degree_t m_rotation;
        units::angle::degree_t m_mountingAngle;
        units::length::inch_t m_targetHeight;
        units::length::inch_t m_targetHeight2;

        double PI = 3.14159265;


};
