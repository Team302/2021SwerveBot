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

#include <map>
#include <vector>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

#include <states/IState.h>

class TurretStateMgr {
  public:
      enum TURRET_STATE
      {
        HOLD,
        LIMELIGHT_AIM,
        MAX_TURRET_STATES
      };


        /// @brief  Find or create the state manmanager
        /// @return TurretStateMgr* pointer to the state manager
        static TurretStateMgr* GetInstance();

        /// @brief  run the current state
        /// @return void
        void RunCurrentState();

        /// @brief  set the current state, initialize it and run it
        /// @param [in]      - state to set
        /// @param [in]     run - true means run, false just initialize it
        /// @return void
        void SetCurrentState
        (
            TURRET_STATE  state,
            bool            run
        );

        /// @brief  return the current state
        
        inline TURRET_STATE GetCurrentState() const { return m_currentStateEnum; };
        
        inline void SetApproxTargetAngle( double angle ) { m_approxTargetAngle = angle; }

    private:

        std::array<IState*, MAX_TURRET_STATES> m_states;
        IState* m_currentState;
        TURRET_STATE m_currentStateEnum;
        double m_approxTargetAngle;
        std::shared_ptr<nt::NetworkTable> m_nt;


        TurretStateMgr();
        ~TurretStateMgr() = default;

        static TurretStateMgr*	m_instance;

};
