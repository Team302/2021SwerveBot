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

//C++ includes
#include <memory>

//FRC Includes
#include <frc2/Timer.h>

//Team 302 Includes
#include <subsys/BallHopper.h>
#include <states/Mech1MotorState.h>


class ControlData;

class BallHopperSlowRelease : public Mech1MotorState
{
    public:

        BallHopperSlowRelease() = delete;
        BallHopperSlowRelease
        (
            ControlData*        control,
            double              target
        );
        ~BallHopperSlowRelease() = default;

        void Init();
        void Run();
        bool AtTarget();

    private:

        //Hold timer
        frc2::Timer      m_timer;
        //BallHopper object to access sensor to detect balls
        std::shared_ptr<BallHopper> m_ballHopper;
        //Time to wait until we release another ball
        const double m_waitTime;
        //These bools make sure we aren't detecting the same ball multiple times and we are running the right state
        bool m_isHolding;
        bool m_canDetect;
        //used to know when we have hit our target of 3 balls
        int m_timesSeen;
        //The pointers to run the holdState and releaseState without switching off of the slowReleaseState in the stateMgr
        IState*     m_holdState;
        IState*     m_releaseState;
};