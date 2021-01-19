
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

/// @class SwerveChassisDefn. 
/// @brief Create a chassis from an XML definition.   CHASSIS_TYPE (ChassisFactory.h) determines the type of 
///        chassis to create.   WheelBase is the front to back distance between the wheel centers.   Track 
///        is the left to right distance between the wheels.

// C++ includes
#include <map>
#include <memory>
#include <string>
#include <utility>


// FRC includes

// Team302 includes
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/IDragonMotorControllerMap.h>
#include <subsys/interfaces/IChassis.h>

#include <xmlhw/SwerveChassisDefn.h>
#include <xmlhw/MotorDefn.h>
#include <utils/Logger.h>

// Third Party includes
#include <pugixml/pugixml.hpp>


using namespace frc;
using namespace pugi;
using namespace std;



/// @brief  Parse the chassie element (and it children).  When this is done a IChassis object exists.
///		   It can be retrieved from the factory.
/// @param [in]  pugi::xml_node the chassis element in the XML document
/// @return void

shared_ptr<IChassis> SwerveChassisDefn::ParseXML
(
	xml_node      chassisNode
)
{
    shared_ptr<IChassis> chassis;
    // initialize the attributes to the default values
    //ChassisFactory::CHASSIS_TYPE type = ChassisFactory::CHASSIS_TYPE::SWERVE
    double wheelDiameter	= 0.0;
    double wheelBase 	    = 0.0;
    double track 		    = 0.0;
    bool hasError 		    = false;

    // process attributes
    for (xml_attribute attr = chassisNode.first_attribute(); attr && !hasError; attr = attr.next_attribute())
    {
        string attrName (attr.name());
        if ( attrName.compare("type") == 0 )
        {
            auto val = string( attr.value() );
            if ( !val.compare( "SWERVE" ) == 0 )
            {
                string msg = "Unknown Chassis Type";
                msg += attr.value();
                Logger::GetLogger()->LogError( string( "SwerveChassisDefn::ParseXML" ), msg );
            }
        }
        else if (  attrName.compare("wheelDiameter") == 0 )
        {
        	wheelDiameter = attr.as_double();
        }
        else if (  attrName.compare("wheelBase") == 0 )
        {
        	wheelBase = attr.as_double();
        }
        else if (  attrName.compare("track") == 0 )
        {
        	track = attr.as_double();
        }
        else   // log errors
        {
            string msg = "unknown attribute ";
            msg += attr.name();
            Logger::GetLogger()->LogError( string("SwerveChassisDefn::ParseXML"), msg );
            hasError = true;
        }
    }


    // Process child element nodes
    IDragonMotorControllerMap motors;
    unique_ptr<MotorDefn> motorXML = make_unique<MotorDefn>();

    for (xml_node child = chassisNode.first_child(); child; child = child.next_sibling())
    {
        string childName (child.name());
    	if ( childName.compare("swervemodule") == 0 )
    	{
            auto motor = motorXML.get()->ParseXML(child);
            if ( motor.get() != nullptr )
            {
                motors[ motor.get()->GetType() ] =  motor ;
            }
    	}
    	else  // log errors
    	{
            string msg = "unknown child ";
            msg += child.name();
            Logger::GetLogger()->LogError( string("SwerveChassisDefn::ParseXML"), msg );
    	}
    }


    // create chassis instance
    if ( !hasError )
    {
        // TODO:   create swerve chassis
    }
    return chassis;
}

