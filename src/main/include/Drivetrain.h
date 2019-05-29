/**
 *  Drivetrain.h
 *  Date: 5/21/19
 *  Last Edited By: Geoffrey Xue
 *  Huge credit to team 2910 for the entirety of this code system
 */


#ifndef SRC_Drivetrain_H_
#define SRC_Drivetrain_H_


#include <frc\WPILib.h>
#include <ctre\Phoenix.h>
#include "OperatorInputs.h"
#include "SwerveSubsystem.h"


using namespace frc;


class Drivetrain
{
public:
	Drivetrain(OperatorInputs *inputs);
	~Drivetrain();
	void Init();
	void Loop();
	void Stop();
	void Disabled();

protected:
	OperatorInputs *m_inputs;
	SwerveSubsystem *m_swerve;
};


#endif /* SRC_Drivetrain_H_ */
