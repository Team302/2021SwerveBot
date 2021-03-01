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

#include <vector>
#include <string>

#include <controllers/MechanismTargetData.h>
#include <controllers/ControlModes.h>

using namespace std;

MechanismTargetData::MechanismTargetData
(
    string                                      state,
    string                                      controller,
    string                                      controller2,
    double                                      target,
    SOLENOID                                    solenoid,
    double                                      secondTarget
) : m_state( state ),
    m_controller( controller ),
    m_controller2( controller2 ),
    m_target( target ),
    m_controlData(),
    m_controlData2(),
    m_solenoid( solenoid ),
    m_secondTarget( secondTarget )
{

}


/// @brief update to include ControlData
/// @param [in] std::vector<ControlData*> - vector of ControlData Objects
/// @return void
void MechanismTargetData::Update( std::vector<ControlData*> data )
{
    for ( auto cd : data )
    {
        if ( m_controller.compare( string(cd->GetIdentifier()))  == 0 )
        {
            m_controlData = cd;
        }
        if ( m_controller2.compare( string(cd->GetIdentifier())) == 0)
        {
            m_controlData2 = cd;
        }
        if ( m_controlData != nullptr && m_controlData2 != nullptr )
        {
            break;
        }
    }
    if ( m_controlData2 == nullptr && m_controlData != nullptr)
    {
        m_controlData2 = m_controlData;
    }
}
