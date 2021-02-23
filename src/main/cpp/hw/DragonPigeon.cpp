/*
 * DragonPigeon.cpp
 *
 *  Created on: Feb 3, 2018
 *      Author: team302
 */

#include <ctre/phoenix/Sensors/PigeonIMU.h>
#include <hw/DragonPigeon.h>
#include <memory>

using namespace std;

using namespace ctre::phoenix::sensors;

DragonPigeon::DragonPigeon
(
    int    canID,
    double rotation
) : m_pigeon(make_unique<PigeonIMU>(canID)),
    m_initialYaw(rotation),
    m_initialPitch(0.0),
    m_initialRoll(0.0)
{
    m_pigeon = make_unique<PigeonIMU>( canID );
    m_pigeon.get()->ConfigFactoryDefault();
    m_pigeon.get()->SetYaw(0.0, 0);
    m_pigeon.get()->SetFusedHeading( rotation, 0);
    m_pigeon.get()->SetStatusFramePeriod( PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_4_Mag, 120, 0);
    m_pigeon.get()->SetStatusFramePeriod( PigeonIMU_StatusFrame::PigeonIMU_CondStatus_11_GyroAccum, 120, 0);
    m_pigeon.get()->SetStatusFramePeriod( PigeonIMU_StatusFrame::PigeonIMU_CondStatus_9_SixDeg_YPR, 120, 0); // using fused heading not yaw
    
    /**
    double ypr[3];
    m_pigeon.get()->GetYawPitchRoll(ypr);

    m_initialYaw   = ypr[0];
    m_initialPitch = ypr[1];
    m_initialRoll  = ypr[2];
    **/
}


double DragonPigeon::GetPitch()
{
    return -(GetRawPitch() - m_initialPitch); //TODO: add inversions into code
}

double DragonPigeon::GetRoll()
{
    return GetRawRoll() - m_initialRoll;
}

double DragonPigeon::GetYaw()
{
    return GetRawYaw() - m_initialYaw;
}

void DragonPigeon::ReZeroPigeon( double angleDeg, int timeoutMs)
{
    m_pigeon.get()->SetFusedHeading( angleDeg, timeoutMs);
}

double DragonPigeon::GetRawPitch()
{
    return 0.0;
    /**
    double ypr[3];
    m_pigeon.get()->GetYawPitchRoll(ypr);

    // return ypr[1]; // yaw = 0 pitch = 1 roll = 2 
    return ypr[2];
    **/
}

double DragonPigeon::GetRawRoll()
{
    return 0.0;
    /**
    double ypr[3];
    m_pigeon.get()->GetYawPitchRoll(ypr);

    // return ypr[2]; // yaw = 0 pitch = 1 roll = 2 
    return ypr[1];
    **/
}

double DragonPigeon::GetRawYaw()
{
    double yaw = m_pigeon.get()->GetFusedHeading();
    //double ypr[3]; // yaw = 0 pitch = 1 roll = 2
    //m_pigeon.get()->GetYawPitchRoll(ypr);
    //double yaw = ypr[0];
    // normalize it to be between -180 and + 180
    if ( yaw > 180 )
    {
        yaw -= 360.0;
    }
    else if ( yaw < -180 )
    {
        yaw += 360.0;
    }
    return yaw;  
}