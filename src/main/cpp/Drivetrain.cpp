/**
 *  Drivetrain.cpp
 *  Date: 5/21/19
 *  Last Edited By: Geoffrey Xue
 *  Huge credit to team 2910 for the entirety of this code system
 */


#include <Drivetrain.h>
#include "Const.h"


using namespace std;


Drivetrain::Drivetrain(OperatorInputs *inputs)
{
	m_inputs = inputs;
	m_swerve = new SwerveSubsystem(1.0);
}


Drivetrain::~Drivetrain()
{
	if (m_swerve != nullptr)
	{	delete m_swerve;	}
}


void Drivetrain::Init()
{
	m_swerve->Init();
}


void Drivetrain::Loop()
{
	double forward = m_inputs->xBoxLeftY(0);
	double strafe = m_inputs->xBoxLeftX(0);
	double rotation = m_inputs->xBoxRightTrigger(OperatorInputs::kHold, 0) - m_inputs->xBoxLeftTrigger(OperatorInputs::kHold, 0);
	m_swerve->SwerveDrive(forward, strafe, rotation);
}


void Drivetrain::Stop()
{
	m_swerve->StopSwerveModules();
}


void Drivetrain::Disabled()
{
	m_swerve->Disabled();
}