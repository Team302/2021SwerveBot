
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
#include <string>
#include <vector>
#include <cmath>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

// Team 302 includes
#include <hw/DragonLimelight.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace nt;
using namespace std;

///-----------------------------------------------------------------------------------
/// Method:         DragonLimelight (constructor)
/// Description:    Create the object
///-----------------------------------------------------------------------------------
DragonLimelight::DragonLimelight
(
    string                      tableName,                  /// <I> - network table name
    units::length::inch_t       mountingHeight,             /// <I> - mounting height of the limelight
    units::length::inch_t       mountingHorizontalOffset,   /// <I> - mounting horizontal offset from the middle of the robot,
    units::angle::degree_t      rotation,                   /// <I> - clockwise rotation of limelight
    units::angle::degree_t      mountingAngle,              /// <I> - mounting angle of the camera
    units::length::inch_t       targetHeight,               /// <I> - height the target
    units::length::inch_t       targetHeight2               /// <I> - height of second target
) : //IDragonSensor(),
    //IDragonDistanceSensor(),
    m_networktable( NetworkTableInstance::GetDefault().GetTable( tableName.c_str()) ),
    m_mountHeight( mountingHeight ),
    m_mountingHorizontalOffset( mountingHorizontalOffset ),
    m_rotation(rotation),
    m_mountingAngle( mountingAngle ),
    m_targetHeight( targetHeight ),
    m_targetHeight2( targetHeight2 )
{
    SetLEDMode( DragonLimelight::LED_MODE::LED_OFF);
}

std::vector<double> DragonLimelight::Get3DSolve() const
{
    return m_networktable.get()->GetNumberArray("camtran", 0);
}

bool DragonLimelight::HasTarget() const
{
    return ( m_networktable.get()->GetNumber("tv", 0.0) > 0.1 );
}

units::angle::degree_t DragonLimelight::GetTargetHorizontalOffset() const
{
    units::angle::degree_t tx = units::angle::degree_t(m_networktable.get()->GetNumber("tx", 0.0));
    units::angle::degree_t ty = units::angle::degree_t(m_networktable.get()->GetNumber("ty", 0.0));
    if(m_rotation == units::angle::degree_t(0.0))
    {
        return tx;
    }
    else if(m_rotation == units::angle::degree_t(90.0))
    {
        return -ty;
    }
    else if(m_rotation == units::angle::degree_t(180.0))
    {
        return -tx;
    }
    else if(m_rotation == units::angle::degree_t(270.0))
    {
        return ty;
    }
    else
    {
        Logger::GetLogger()->LogError("DragonLimelight::GetTargetVerticalOffset", "Invalid limelight rotation");
        return units::angle::degree_t(-180.0);
    }
    
   
}

units::angle::degree_t DragonLimelight::GetTargetVerticalOffset() const
{
    units::angle::degree_t tx = units::angle::degree_t(m_networktable.get()->GetNumber("tx", 0.0));
    units::angle::degree_t ty = units::angle::degree_t(m_networktable.get()->GetNumber("ty", 0.0));
    if(m_rotation == units::angle::degree_t(0.0))
    {
        return ty;
    }
    else if(m_rotation == units::angle::degree_t(90.0))
    {
        return tx;
    }
    else if(m_rotation == units::angle::degree_t(180.0))
    {
        return -ty;
    }
    else if(m_rotation == units::angle::degree_t(270.0))
    {
        return -tx;
    }
    else
    {
        Logger::GetLogger()->LogError("DragonLimelight::GetTargetVerticalOffset", "Invalid limelight rotation");
        return units::angle::degree_t(-180.0);
    }
    
}

double DragonLimelight::GetTargetArea() const
{
    return m_networktable.get()->GetNumber("ta", 0.0);
}

units::angle::degree_t DragonLimelight::GetTargetSkew() const
{
    return units::angle::degree_t(m_networktable.get()->GetNumber("ts", 0.0));
}

units::time::microsecond_t DragonLimelight::GetPipelineLatency() const
{
    return units::time::second_t(m_networktable.get()->GetNumber("tl", 0.0));
}


void DragonLimelight::SetTargetHeight
(
    units::length::inch_t targetHeight
)
{
    m_targetHeight = targetHeight;
}

void DragonLimelight::SetLEDMode(DragonLimelight::LED_MODE mode)
{
    m_networktable.get()->PutNumber("ledMode", mode);
}

void DragonLimelight::SetCamMode(DragonLimelight::CAM_MODE mode)
{
    m_networktable.get()->PutNumber("camMode", mode);
}

void DragonLimelight::SetPipeline(int pipeline)
{
    m_networktable.get()->PutNumber("pipeline", pipeline);
}

void DragonLimelight::SetStreamMode(DragonLimelight::STREAM_MODE mode)
{
    m_networktable.get()->PutNumber("stream", mode);
}

void DragonLimelight::SetCrosshairPos( double crosshairPosX, double crosshairPosY)
{
    m_networktable.get()->PutNumber("cx0", crosshairPosX);
    m_networktable.get()->PutNumber("cy0", crosshairPosY);
}

void DragonLimelight::SetSecondaryCrosshairPos( double crosshairPosX, double crosshairPosY)
{
    m_networktable.get()->PutNumber("cx1", crosshairPosX);
    m_networktable.get()->PutNumber("cy1", crosshairPosY);
}

// MAX of 32 snapshots can be saved
void DragonLimelight::ToggleSnapshot(DragonLimelight::SNAPSHOT_MODE toggle)
{
    m_networktable.get()->PutNumber("snapshot", toggle);
}

void DragonLimelight::PrintValues()
{
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues HasTarget", to_string( HasTarget() ) );    
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues XOffset", to_string( GetTargetHorizontalOffset().to<double>() ) ); 
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues YOffset", to_string( GetTargetVerticalOffset().to<double>() ) ); 
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues Area", to_string( GetTargetArea() ) ); 
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues Skew", to_string( GetTargetSkew().to<double>() ) ); 
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues Latency", to_string( GetPipelineLatency().to<double>() ) ); 
}

units::length::inch_t DragonLimelight::EstimateTargetDistance() const
{
    units::angle::degree_t angleDifference = (GetMountingAngle() + GetTargetVerticalOffset());
    units::angle::radian_t angleRad = angleDifference;
    return (GetTargetHeight()-GetMountingHeight()) / tan(angleRad.to<double>());
}