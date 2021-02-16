/*
 * DragonPigeon.h
 *
 *  Created on: Feb 3, 2018
 *      Author: team302
 */

#pragma once

#include <memory>
#include <ctre/phoenix/Sensors/PigeonIMU.h>


class DragonPigeon
{
    public:
        DragonPigeon
        (
            int  canID,
            double rotation
        );
        DragonPigeon() = delete;
        virtual ~DragonPigeon() = default;

        double GetPitch();
        double GetRoll();
        double GetYaw();
        void ReZeroPigeon( double angleDeg, int timeoutMs = 0);

    private:

        std::unique_ptr<ctre::phoenix::sensors::PigeonIMU> m_pigeon;

        double m_initialYaw;
        double m_initialPitch;
        double m_initialRoll;

        // these methods correct orientation, but do not apply the initial offsets
        double GetRawYaw();
        double GetRawRoll();
        double GetRawPitch();
};



