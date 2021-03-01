
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

#include <string>
#include <xmlhw/LimelightDefn.h>
#include <hw/DragonLimelight.h>
#include <utils/HardwareIDValidation.h>
#include <utils/UsageValidation.h>
#include <utils/Logger.h>
#include <hw/factories/LimelightFactory.h>

#include <pugixml/pugixml.hpp>

using namespace std;

DragonLimelight* LimelightDefn::ParseXML(pugi::xml_node    limelightNode)
{
    // initialize the output
    DragonLimelight* limelight = nullptr;

    bool hasError = false;

    // initialize attributes to default values
    std::string tableName = "";
    units::length::inch_t mountingHeight = units::length::inch_t(0.0);
    units::length::inch_t horizontalOffset = units::length::inch_t(0.0);
    units::angle::degree_t mountingAngle = units::angle::degree_t(0.0);
    units::angle::degree_t rotation = units::angle::degree_t(0.0);
    units::length::inch_t targetHeight = units::length::inch_t(0.0);
    units::length::inch_t targetHeight2 = units::length::inch_t(0.0);

    DragonLimelight::LED_MODE ledMode = DragonLimelight::LED_MODE::LED_DEFAULT;
    DragonLimelight::CAM_MODE camMode = DragonLimelight::CAM_MODE::CAM_VISION;
    DragonLimelight::STREAM_MODE streamMode = DragonLimelight::STREAM_MODE::STREAM_DEFAULT;
    DragonLimelight::SNAPSHOT_MODE snapMode = DragonLimelight::SNAPSHOT_MODE::SNAP_OFF;
    double defaultXHairX = -2.0;
    double defaultXHairY = -2.0;
    double secXHairX = -2.0;
    double secXHairY = -2.0;

    for (pugi::xml_attribute attr = limelightNode.first_attribute(); attr && !hasError; attr = attr.next_attribute())
    {
        // validate/set the usage
        if ( strcmp( attr.name(), "tablename" ) == 0 )
        {
            tableName = attr.value();
        }
        else if ( strcmp( attr.name(), "mountingheight" ) == 0 )
        {
            mountingHeight = units::length::inch_t(attr.as_double());
        }
        else if ( strcmp( attr.name(), "horizontaloffset" ) == 0 )
        {
            horizontalOffset = units::length::inch_t(attr.as_double());
        }
        else if ( strcmp( attr.name(), "mountingangle" ) == 0 )
        {
            mountingAngle = units::angle::degree_t(attr.as_double());
        }
        else if ( strcmp( attr.name(), "rotation" ) == 0 )
        {
            rotation = units::angle::degree_t(attr.as_double());
        }
		//todo: should this come from state XMLs
        else if ( strcmp( attr.name(), "targetheight" ) == 0 )
        {
            targetHeight = units::length::inch_t(attr.as_double());
        }
        else if ( strcmp( attr.name(), "targetheight2" ) == 0 )
        {
            targetHeight2 = units::length::inch_t(attr.as_double());
        }
        else if ( strcmp( attr.name(), "defaultledmode" ) == 0 )
        {
            if ( strcmp( attr.value(), "currentpipeline") == 0 )
            {
                ledMode = DragonLimelight::LED_MODE::LED_DEFAULT;
            }
            else if ( strcmp( attr.value(), "off") == 0 )
            {
                ledMode = DragonLimelight::LED_MODE::LED_OFF;
            }
            else if ( strcmp( attr.value(), "blink") == 0 )
            {
                ledMode = DragonLimelight::LED_MODE::LED_BLINK;
            }
            else if ( strcmp( attr.value(), "on") == 0 )
            {
                ledMode = DragonLimelight::LED_MODE::LED_ON;
            }
        }        
        else if ( strcmp( attr.name(), "defaultcammode" ) == 0 )
        {
            if ( strcmp( attr.value(), "drivercamera")==0)
            {
                camMode = DragonLimelight::CAM_MODE::CAM_DRIVER;
            }
        }        
        else if ( strcmp( attr.name(), "streammode" ) == 0 )
        {
            if ( strcmp( attr.value(), "pipmain") == 0 )
            {
                streamMode = DragonLimelight::STREAM_MODE::STREAM_MAIN_AND_SECOND;
            }
            else if ( strcmp( attr.value(), "pipsecondary" ) == 0 )
            {
                streamMode = DragonLimelight::STREAM_MODE::STREAM_SECOND_AND_MAIN;
            }
        }        
        else if ( strcmp( attr.name(), "snapshots" ) == 0 )
        {
            if ( strcmp( attr.value(), "twopersec") == 0 )
            {
                snapMode = DragonLimelight::SNAPSHOT_MODE::SNAP_ON;
            }
        }
        else if ( strcmp( attr.name(), "crosshairx" ) == 0 )
        {
            defaultXHairX = attr.as_double();
        }        
        else if ( strcmp( attr.name(), "crosshairy" ) == 0 )
        {
            defaultXHairY = attr.as_double();
        }        
        else if ( strcmp( attr.name(), "secondcrosshairx" ) == 0 )
        {
            secXHairX = attr.as_double();
        }        
        else if ( strcmp( attr.name(), "secondcrosshairy" ) == 0 )
        {
            secXHairY = attr.as_double();
        }


		//todo:  add cross hair stuff/streaming options -- everything after target heights
        else
        {
            string msg = "unknown attribute ";
            msg += attr.name();
            Logger::GetLogger()->LogError( "LimelightDefn::ParseXML", msg );
            hasError = true;
        }

        if(!hasError)
        {
            limelight = LimelightFactory::GetLimelightFactory()->CreateLimelight
            (
                tableName,
                mountingHeight,
                horizontalOffset,
                rotation,
                mountingAngle,
                targetHeight,
                targetHeight2,
                ledMode,
                camMode,
                streamMode,
                snapMode,
                defaultXHairX,
                defaultXHairY,
                secXHairX,
                secXHairY
            );
        }
    }
    return limelight;
}