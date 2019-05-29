/**
 *  SwerveSubsystem.h
 *  Date: 5/21/19
 *  Last Edited By: Geoffrey Xue
 *  Huge credit to team 2910 for the entirety of this code system
 */


#ifndef SRC_SwerveSubsystem_H_
#define SRC_SwerveSubsystem_H_


#include <frc\WPILib.h>
#include <ctre\Phoenix.h>
#include "OperatorInputs.h"
#include "SwerveModule.h"
#include "Gyro.h"


using namespace frc;


class SwerveSubsystem
{
public:
	SwerveSubsystem(double speedmultiplier);
	~SwerveSubsystem();
	void Init();
	void GetSwerveModuleValues(double forward, double strafe, double rotation, double *p_angles, double *p_speeds);
	void SwerveDrive(double forward, double strafe, double rotation);
	void StopSwerveModules();
	void Disabled();

protected:
	WPI_TalonSRX *m_frontleftmov;
	WPI_TalonSRX *m_frontleftang;
	WPI_TalonSRX *m_frontrightmov;
	WPI_TalonSRX *m_frontrightang;
	WPI_TalonSRX *m_backleftmov;
	WPI_TalonSRX *m_backleftang;
	WPI_TalonSRX *m_backrightmov;
	WPI_TalonSRX *m_backrightang;

	SwerveModule *m_swervemodules[4];

	DualGyro *m_gyro;

	double m_speedmultiplier;
};


#endif /* SRC_DriveTrain_H_ */
