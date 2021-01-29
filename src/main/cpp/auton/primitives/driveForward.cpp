#include <memory>
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
m_initialDistance(),
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
    //Fix this
    m_arcing = abs(params->GetHeading()) > 0.1;
    m_endHeading = m_startHeading + params->GetHeading();

    m_minSpeedCountTime = MIN_SPEED_COUNT_TIME;
    m_underSpeedCounts = 0;
    m_params = params;

    m_targetDistance = params->GetDistance();
    //Fix this
    m_initialDistance = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()->GetPose().GetEstimatedPosition();
    //frc::SmartDashboard::PutNumber("Initial Distance", m_initialDistance);
}

void driveForward::Run()
{
    Logger::GetLogger() -> LogError(string("DriveDistance::Run()"), string("Arrived!"));

    CalculateSlowDownDistance();

    //Fix this
    if (m_minSpeedCountTime <= 0)
    {
        //if (abs(SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()->Get))
    }

    //Fix this error
    m_minSpeedCountTime -= IPrimitive::LOOP_LENGTH;
}

bool driveForward::IsDone()
{
    //Fix getCurrentPosition
    Pose2d progress = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()->GetPose().GetEstimatedPosition();
    //bool reachedTarget = (progress) > (m_targetDistance);
    //frc::SmartDashboard::PutNumber("Current Chassis Distance", progress);
    frc::SmartDashboard::PutNumber("Target Chassis Distance", m_targetDistance);
    //Fix IPrimitive::Loop_LENGTH
    m_timeRemaining -= IPrimitive::LOOP_LENGTH;

    auto initialTrans = m_initialDistance.Translation();
    auto currentTrans = progress.Translation();

    
    units::length::inch_t dist = currentTrans.Distance(initialTrans);

    bool reachedTarget = std::abs (targetDistance - distance.length.to<double>()) < 0.2;

    bool done = reachedTarget;
    if (done)
    {
        SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()->SetTargetHeading(m_endHeading);
        return done;
    }
}

void driveForward::CalculateSlowDownDistance()
{
    float currentVel = SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis()->GetCurrentSpeed();
    float decelTime = currentVel / SuperDrive::INCHES_PER_SECOND_SECOND;
    float decelDist = abs(((currentVel - m_minSpeed)) * decelTime * DECEL_TIME_MULTIPLIER);
    float currentDistance = abs(SwerveChassisFactory::GetSwerveChassisFactory() -> GetSwerveChassis()->GetCurrentPosition() - m_initialDistance);
    float distanceRemaining = abs(m_targetDistance =- currentDistance);

    if (distanceRemaining <= decelDist)
    {
        SuperDrive::SlowDown();
    }
}

void driveForward::SetDistance(double distance)
{
    m_targetDistance = distance;
}