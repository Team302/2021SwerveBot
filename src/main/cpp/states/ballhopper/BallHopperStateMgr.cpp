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

//C++ Includes
#include <map>
#include <memory>
#include <vector>

//Team 302 Includes
#include <states/IState.h>
#include <states/ballhopper/BallHopperStateMgr.h>
#include <xmlmechdata/StateDataDefn.h>
#include <controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <gamepad/TeleopControl.h>
#include <states/ballhopper/BallHopperState.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>


using namespace std;

BallHopperStateMgr* BallHopperStateMgr::m_instance = nullptr;
BallHopperStateMgr* BallHopperStateMgr::GetInstance()
{
    if ( BallHopperStateMgr::m_instance == nullptr)
    {
        BallHopperStateMgr::m_instance = new BallHopperStateMgr();
    }
    return BallHopperStateMgr::m_instance;
}

/// @brief initialize the state manager, parse the configuration file and create the states
BallHopperStateMgr::BallHopperStateMgr() : m_currentState(),
                                           m_stateVector(),
                                           m_currentStateEnum(BALL_HOPPER_STATE::OFF)
{
    //Parse the configuration file
    auto stateXML = make_unique<StateDataDefn>();
    vector<MechanismTargetData*> targetData = stateXML.get()->ParseXML( MechanismTypes::MECHANISM_TYPE::BALL_HOPPER );

    // initialize the xml string to state map
    map<string, BALL_HOPPER_STATE> stateMap;
    stateMap["BALLHOPPEROFF"] = BALL_HOPPER_STATE::OFF;
    stateMap["BALLHOPPERHOLD"] = BALL_HOPPER_STATE::HOLD;
    stateMap["BALLHOPPERRAPIDRELEASE"] = BALL_HOPPER_STATE::RAPID_RELEASE;
    stateMap["BALLHOPPERSLOWRELEASE"] = BALL_HOPPER_STATE::SLOW_RELEASE;
    m_stateVector.resize(5);
    //create the states passign the configuration data
    for ( auto td: targetData )
    {
        auto stateString = td->GetStateString();
        auto stateStringToEnumItr = stateMap.find( stateString );
        if ( stateStringToEnumItr != stateMap.end() )
        {
            auto stateEnum = stateStringToEnumItr->second;
            if ( m_stateVector[stateEnum]  == nullptr)
            {
                auto controlData = td->GetController();
                auto target = td->GetTarget();
                switch ( stateEnum)
                {
                    case BALL_HOPPER_STATE::OFF:
                    {
                        auto thisState = new BallHopperState( controlData, target );
                        m_stateVector[stateEnum] = thisState;
                        m_currentState = thisState;
                        m_currentStateEnum = stateEnum;
                        m_currentState->Init();
                    }
                    break;

                    case BALL_HOPPER_STATE::HOLD:
                    {
                        auto thisState = new BallHopperState( controlData, target );
                        m_stateVector[stateEnum] = thisState;
                    }
                    break;

                    case BALL_HOPPER_STATE::SLOW_RELEASE:
                    {
                        auto thisState = new BallHopperState( controlData, target );
                        m_stateVector[stateEnum] = thisState;
                    }
                    break;

                    case BALL_HOPPER_STATE::RAPID_RELEASE:
                    {
                        auto thisState = new BallHopperState( controlData, target );
                        m_stateVector[stateEnum] = thisState;
                    }
                    break;

                    default:
                    {
                        Logger::GetLogger()->LogError( Logger::LOGGER_LEVEL::ERROR_ONCE, string("BallHopperStateMgr::BallHopperStateMgr"), string("unknown state"));

                    }
                    break;
                }
            }
            else
            {
                Logger::GetLogger()->LogError( Logger::LOGGER_LEVEL::ERROR_ONCE, string("BallHopperStateMgr::BallHopperStateMgr"), string("multiple mechanism state info for state"));
            }
            
        }
        else
        {
            Logger::GetLogger()->LogError( Logger::LOGGER_LEVEL::ERROR_ONCE, string("BallHopperStateMgr::BallHopperStateMgr"), string("state not found"));
        }
    }
}

/// @brief run the current state 
/// @return void
void BallHopperStateMgr::RunCurrentState( BALL_HOPPER_STATE hopperState)
{
    if( MechanismFactory::GetMechanismFactory()->GetBallHopper().get() != nullptr)
    {
        if ( hopperState == BALL_HOPPER_STATE::OFF && m_currentStateEnum != BALL_HOPPER_STATE::OFF)
        {
            SetCurrentState( BALL_HOPPER_STATE::OFF, false);
        }
        else if ( hopperState == BALL_HOPPER_STATE::HOLD && m_currentStateEnum != BALL_HOPPER_STATE::HOLD)
        {
            SetCurrentState( BALL_HOPPER_STATE::HOLD, false);
        }
        else if ( hopperState == BALL_HOPPER_STATE::SLOW_RELEASE && m_currentStateEnum != BALL_HOPPER_STATE::SLOW_RELEASE)
        {

            
            //SetCurrentState( BALL_HOPPER_STATE::SLOW_RELEASE, false);
        }
        else if ( hopperState == BALL_HOPPER_STATE::RAPID_RELEASE && m_currentStateEnum != BALL_HOPPER_STATE::RAPID_RELEASE)
        {
            SetCurrentState( BALL_HOPPER_STATE::RAPID_RELEASE, false);
        }
        else 
        {
            Logger::GetLogger()->LogError( Logger::LOGGER_LEVEL::ERROR_ONCE, string("BallHopperStateMgr::RunCurrentState"), string("state not found"));
        }
    }

    //run the current state
    if ( m_currentState != nullptr)
    {
        m_currentState->Run();
    }
}

/// @brief set the current state, initialize it and run it
/// @return void
void BallHopperStateMgr::SetCurrentState
(
    BALL_HOPPER_STATE       stateEnum,
    bool                    run
)
{

    Logger::GetLogger()->LogError( Logger::LOGGER_LEVEL::PRINT_ONCE, string("BallHopperStateMgr::SetCurrentState"), string("about to set current state: " + to_string(stateEnum)));
    auto state = m_stateVector[stateEnum];
    if ( state != nullptr && state != m_currentState)
    {
        m_currentState = state;
        m_currentStateEnum = stateEnum;
        m_currentState->Init();
        if( run )
        {
            if ( MechanismFactory::GetMechanismFactory()->GetBallHopper().get() != nullptr )
            {
                m_currentState->Run();
            }
        }
    }
}