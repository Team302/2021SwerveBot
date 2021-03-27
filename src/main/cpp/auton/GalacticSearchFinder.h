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

//Team302 Includes
#include <auton/PrimitiveParams.h>
#include <auton/primitives/IPrimitive.h>
#include <utils/Logger.h>

//FRC,WPI Includes

#include <frc/geometry/Pose2d.h>

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class GalacticSearchFinder// : public IPrimitive

{
public:
<<<<<<< Updated upstream
    GalacticSearchFinder() = default;
=======
     GalacticSearchFinder() = default;
>>>>>>> Stashed changes

    virtual ~GalacticSearchFinder() = default;

    std::string GetGalacticSearchPath() ;  // return path name for galatic search
    std::string GetGSPathFromVisionTbl(); 

private:

    

    bool CheckTarget(double*, double, double, double,double);

    // network table reading
    // Using Default table no referances for Swerve Drive Module m_nt.
    // May need to create a link to m_nt if more than one network table is used.
    nt::NetworkTableInstance inst = nt::NetworkTableInstance().GetDefault();
     
    wpi::Twine sTableName = "visionTable";
    wpi::StringRef sRef_TblCVAngle = "horAngle";
    wpi::StringRef sRef_TblCVDistance = "CellDistance";
 };