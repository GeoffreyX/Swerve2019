/**
 *  SwerveModule.cpp
 *  Date: 5/21/19
 *  Last Edited By: Geoffrey Xue
 *  Huge credit to team 2910 for the entirety of this code system
 */


#include "SwerveModule.h"
#include "Const.h"
#include <cmath>


using namespace std;


SwerveModule::SwerveModule(int modulenumber, WPI_TalonSRX *talondrive, WPI_TalonSRX *talonangle, double offset)
{
    m_talondrive = talondrive;
    m_talonangle = talonangle;

    m_modulenum = modulenumber;
    m_offset = offset;
    m_lasttargetangle = 0;
    m_lasterror = 0;
    m_inverted = false;

    /// defaults for angle motor
    m_talonangle->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
    m_talonangle->SetSensorPhase(true);     // ?

    m_talonangle->Config_kP(0, 30, 0);
    m_talonangle->Config_kI(0, 0.001, 0);
    m_talonangle->Config_kD(0, 200, 0);

    m_talonangle->SetNeutralMode(NeutralMode::Brake);
    m_talonangle->Set(ControlMode::Position, 0);

    /// defaults for drive motor
    m_talondrive->ConfigMotionCruiseVelocity(640, 0);
    m_talondrive->ConfigMotionAcceleration(200, 0);

    m_talondrive->SetNeutralMode(NeutralMode::Brake);

    /* These are things that I'm not gonna use for this code cause I don't think we're doing autonomous w/ drive motors
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

    driveMotor.config_kP(0, 15, 0);
    driveMotor.config_kI(0, 0.01, 0);
    driveMotor.config_kD(0, 0.1, 0);
    driveMotor.config_kF(0, 0.2, 0);
    */

    /// Set amperage limits
    m_talonangle->ConfigContinuousCurrentLimit(MOTOR_ANGLE_CURRENT_LIMIT, 0);
    m_talonangle->ConfigPeakCurrentLimit(MOTOR_ANGLE_CURRENT_LIMIT, 0);
    m_talonangle->ConfigPeakCurrentDuration(MOTOR_CURRENT_DURATION, 0);
    m_talonangle->EnableCurrentLimit(true);

    m_talondrive->ConfigContinuousCurrentLimit(MOTOR_DRIVE_CURRENT_LIMIT, 0);
    m_talondrive->ConfigPeakCurrentLimit(MOTOR_DRIVE_CURRENT_LIMIT, 0);
    m_talondrive->ConfigPeakCurrentDuration(MOTOR_CURRENT_DURATION, 0);
    m_talondrive->EnableCurrentLimit(true);
}


SwerveModule::~SwerveModule()
{
    if (m_talondrive != nullptr)
    {   delete m_talondrive; }
    if (m_talonangle != nullptr)
    {   delete m_talonangle;    }
}


double SwerveModule::GetCurrentAngle()
{
    double angle = m_talonangle->GetSelectedSensorPosition(0) * TICKS_PER_REV;

    angle -= m_offset;
    fmod(angle, 360.0);

    if (angle < 0)
    {
        angle += 360.0;
    }
    return angle;
}


void SwerveModule::SetTargetAngle(double targetangle)
{
//  if(angleMotorJam) 
//  {
//      m_talonangle->set(ControlMode.Disabled, 0);
//      return;
//	}
    
    m_lasttargetangle = targetangle;

    /// targetangle is reduced to a value inside +- 360
    fmod(targetangle, 360.0);

//  SmartDashboard->PutNumber("Module Target Angle " + moduleNumber, targetangle % 360);

    targetangle += m_offset;

    double delta = GetCurrentAngle() - targetangle;

    /// Constraints delta and angle to -180 < delta/angle < 180 
    if (delta > 180) 
    {   targetangle += 360; }
    else if (delta < -180)
    {   targetangle -= 360; }

    delta = GetCurrentAngle() - targetangle;

    /// If the delta is more than 90 from 0, don't invert drive motor
    if (delta > 90 || delta < -90)
    {
        if (delta > 90)
        {   targetangle += 180; }
        else if (delta < -90)
        {   targetangle -= 180; }
        m_talondrive->SetInverted(false);
    }
    else
    {
        m_talondrive->SetInverted(true);
    }

    /// targetangle is increased to adjust to encoders
    targetangle += m_talonangle->GetSelectedSensorPosition(0) * TICKS_PER_REV - GetCurrentAngle();

    double currenterror = m_talonangle->GetClosedLoopError(0);

//        if (Math.abs(currentError - mLastError) < 7.5 &&
//                Math.abs(currentAngle - targetAngle) > 5) {
//            if (mStallTimeBegin == Long.MAX_VALUE) {
//            	mStallTimeBegin = System.currentTimeMillis();
//            }
//            if (System.currentTimeMillis() - mStallTimeBegin > STALL_TIMEOUT) {
//            	angleMotorJam = true;
//            	m_talonangle->set(ControlMode.Disabled, 0);
//            	m_talondrive->set(ControlMode.Disabled, 0);
//            	SmartDashboard.putBoolean("Motor Jammed" + moduleNumber, angleMotorJam);
//            	return;
//            }
//        } else {
//            mStallTimeBegin = Long.MAX_VALUE;
//        }

    m_lasterror = currenterror;
    targetangle /= TICKS_PER_REV;
    m_talonangle->Set(ControlMode::Position, targetangle);
}


void SwerveModule::SetDriveSpeed(double drivespeed)
{
    if (m_inverted) drivespeed = -drivespeed;

    m_talondrive->Set(ControlMode::PercentOutput, drivespeed);
}