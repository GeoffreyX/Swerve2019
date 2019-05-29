/**
 *  SwerveModule.h
 *  Date: 5/21/19
 *  Last Edited By: Geoffrey Xue
 *  Huge credit to team 2910 for the entirety of this code system
 */


#ifndef SRC_SwerveModule_H_
#define SRC_SwerveModule_H_


#include <frc\WPILib.h>
#include <ctre\Phoenix.h>



using namespace frc;


class SwerveModule
{
public:
	SwerveModule(int modulenumber, WPI_TalonSRX *talondrive, WPI_TalonSRX *talonangle, double offset);
	~SwerveModule();

	// angle 
	double GetCurrentAngle();
    void SetTargetAngle(double targetangle);
	double GetTargetAngle(){ return m_lasttargetangle;	};

	// drive 
	void SetDriveInverted(bool inverted){	m_inverted = inverted;	};
	void SetDriveSpeed(double drivespeed);
	/*
	double GetDriveDistance();
	void ZeroDistance();
	*/

	int GetModuleNumber(){	return m_modulenum;	};

protected:
	WPI_TalonSRX *m_talondrive;
    WPI_TalonSRX *m_talonangle;

	int m_modulenum;
    double m_offset;
	double m_lasttargetangle;
	double m_lasterror;
	bool m_inverted;
};


#endif /* SRC_SwerveModule_H_ */
