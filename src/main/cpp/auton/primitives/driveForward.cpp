/*#include <memory>
#include <string>
#include <auton/PrimitiveParams.h>
#include <auton/primitives/IPrimitive.h>
#include <subsys/SwerveChassisFactory.h>
#include <utils/Logger.h>

#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include <auton/primitives/driveForward.h>
#include <hw/factories/PigeonFactory.h>
#include <hw/DragonPigeon.h>


using namespace std;
using namespace frc;

driveForward::driveForward() :
m_params(nullptr),
m_targetDistance(0),
m_initialDistance(0),
m_timeRemaining(0),
m_minSpeedCountTime(0),
m_underSpeedCounts(0),
m_startHeading(0),
m_endHeading(0),
m_minSpeed(0),
m_arcing(false)
{
}

void driveForward::Init(PrimitiveParams* params)
{
    
    //m_arcing = abs(params->GetHeading()) > 0.1;
    m_endHeading = m_startHeading + params->GetHeading();

    m_minSpeedCountTime = MIN_SPEED_COUNT_TIME;
    m_underSpeedCounts = 0;
    m_params = params;

    m_targetDistance = params->GetDistance();
    //m_initialDistance = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()->UpdateOdometry();
    frc::SmartDashboard::PutNumber("Initial Distance", m_initialDistance);
}

void driveForward::Run()
{
    Logger::GetLogger() -> LogError(string("DriveDistance::Run()"), string("Arrived!"));

    CalculateSlowDownDistance();
}*/