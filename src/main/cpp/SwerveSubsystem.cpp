/**
 *  SwerveSubsystem.cpp
 *  Date: 5/21/19
 *  Last Edited By: Geoffrey Xue
 *  Huge credit to team 2910 for the entirety of this code system
 */


#include "SwerveSubsystem.h"
#include "Const.h"
#include <cmath>


using namespace std;



SwerveSubsystem::SwerveSubsystem(double speedmultiplier)
{
	m_frontleftmov = nullptr;
	m_frontleftang = nullptr;
	m_frontrightmov = nullptr;
	m_frontrightang = nullptr;
	m_backleftmov = nullptr;
	m_backleftang = nullptr;
	m_backrightmov = nullptr;
	m_backrightang = nullptr;

	for (int i = 0; i < 4; i++)
	{	m_swervemodules[i] = nullptr;	}

	m_gyro = nullptr;

	m_speedmultiplier = speedmultiplier;
}


SwerveSubsystem::~SwerveSubsystem()
{
	if (m_frontleftmov != nullptr)
	{	delete m_frontleftmov;	}
	if (m_frontleftang != nullptr)
	{	delete m_frontleftang;	}
	if (m_frontrightmov != nullptr)
	{	delete m_frontrightmov;	}
	if (m_frontrightang != nullptr)
	{	delete m_frontrightang;	}
	if (m_backleftmov != nullptr)
	{	delete m_backleftmov;	}
	if (m_backleftang != nullptr)
	{	delete m_backleftang;	}
	if (m_backrightmov != nullptr)
	{	delete m_backrightmov;	}
	if (m_backrightang != nullptr)
	{	delete m_backrightang;	}
	for (int i = 0; i < 4; i++)
	{	delete m_swervemodules[i];	}
	if (m_gyro != nullptr)
	{	delete m_gyro;	}
}


void SwerveSubsystem::Init()
{
	if ((m_frontleftmov == nullptr) && (CAN_FRONT_LEFT_MOV != -1))
	{	m_frontleftmov = new WPI_TalonSRX(CAN_FRONT_LEFT_MOV);	}
	if ((m_frontleftang == nullptr) && (CAN_FRONT_LEFT_ANG != -1))
	{	m_frontleftang = new WPI_TalonSRX(CAN_FRONT_LEFT_ANG);	}

	if ((m_frontrightmov == nullptr) && (CAN_FRONT_RIGHT_MOV != -1))
	{	m_frontrightmov = new WPI_TalonSRX(CAN_FRONT_RIGHT_MOV);	}
	if ((m_frontrightang == nullptr) && (CAN_FRONT_RIGHT_ANG != -1))
	{	m_frontrightang = new WPI_TalonSRX(CAN_FRONT_RIGHT_ANG);	}

	if ((m_backleftmov == nullptr) && (CAN_BACK_LEFT_MOV != -1))
	{	m_backleftmov = new WPI_TalonSRX(CAN_BACK_LEFT_MOV);	}
	if ((m_backleftang == nullptr) && (CAN_BACK_LEFT_ANG != -1))
	{	m_backleftang = new WPI_TalonSRX(CAN_BACK_LEFT_ANG);	}

	if ((m_backrightmov == nullptr) && (CAN_BACK_RIGHT_MOV != -1))
	{	m_backrightmov = new WPI_TalonSRX(CAN_BACK_RIGHT_MOV);	}
	if ((m_backrightang == nullptr) && (CAN_BACK_RIGHT_ANG != -1))
	{	m_backrightang = new WPI_TalonSRX(CAN_BACK_RIGHT_ANG);	}

	m_swervemodules[0] = new SwerveModule(0,
							m_frontleftmov,
							m_frontleftang,
							87.890);
	m_swervemodules[1] = new SwerveModule(1,
							m_frontrightmov,
							m_frontrightang,
							235.195);
	m_swervemodules[2] = new SwerveModule(2,
							m_backleftmov,
							m_backleftang,
							320.976);
	m_swervemodules[3] = new SwerveModule(3,
							m_backrightmov,
							m_backrightang,
							245.742);

	m_swervemodules[0]->SetDriveInverted(true);
	m_swervemodules[3]->SetDriveInverted(true);

	for (SwerveModule *module : m_swervemodules) 
	{
		module->SetTargetAngle(0);
	}

	if (m_gyro == nullptr)
	{	m_gyro = new DualGyro(CAN_GYRO1, CAN_GYRO2);	}
}


void SwerveSubsystem::GetSwerveModuleValues(double forward, double strafe, double rotation, double *p_angles, double *p_speeds)
{
	double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
	double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
	double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
	double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

	double angles[] =
	{
		atan2(b, c) * 180 / PI,
		atan2(b, d) * 180 / PI,
		atan2(a, d) * 180 / PI,
		atan2(a, c) * 180 / PI
	};

	double speeds[] =
	{
		sqrt(b * b + c * c),
		sqrt(b * b + d * d),
		sqrt(a * a + d * d),
		sqrt(a * a + c * c)
	};

	for (int i = 0; i < 4; i++)
	{
		*(p_angles + i) = angles[i];
		*(p_speeds + i) = speeds[i];
	}
}

void SwerveSubsystem::SwerveDrive(double forward, double strafe, double rotation)
{
	forward *= m_speedmultiplier;
	strafe *= m_speedmultiplier;

	double angles[4];
	double speeds[4];
	double *p_angles = angles;
	double *p_speeds = speeds;

	GetSwerveModuleValues(forward, strafe, rotation, p_angles, p_speeds);

    double max = speeds[0];

	for (double speed : speeds)
	{
		if (speed > max)
		{	max = speed;}
	}

	for (int i = 0; i < 4; i++)
	{
		if (fabs(forward) > 0.05 ||
			fabs(strafe) > 0.05 ||
			fabs(rotation) > 0.05) 
		{
			m_swervemodules[i]->SetTargetAngle(angles[i] + 180);
		}
		else
		{
			m_swervemodules[i]->SetTargetAngle(m_swervemodules[i]->GetTargetAngle());
		}
		m_swervemodules[i]->SetDriveSpeed(speeds[i]);
	}
}


void SwerveSubsystem::StopSwerveModules()
{
	for (SwerveModule *module : m_swervemodules) {
		module->SetDriveSpeed(0);
	}
}


void SwerveSubsystem::Disabled()
{
	m_gyro->Stop();
}