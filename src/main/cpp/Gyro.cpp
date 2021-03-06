/**
 *  Gryo.cpp
 *  Date: 5/21/19
 *  Last Edited By: Geoffrey Xue
 *  Huge credit to team 2910 for the entirety of this code system
 */


#include "Gyro.h"
#include "Const.h"


using namespace std;


DualGyro::DualGyro(int gyro1, int gyro2)
{
    m_pigeon1 = nullptr;
    m_pigeon2 = nullptr;
   	m_gyroval1[0] = 0.0;
	m_gyroval1[1] = 0.0;
	m_gyroval1[2] = 0.0;
	
    m_gyroval2[0] = 0.0;
	m_gyroval2[1] = 0.0;
	m_gyroval2[2] = 0.0;

    m_heading1 = 0.0;
    m_heading2 = 0.0;

    m_gyrovalid1 = false;
    m_gyrovalid2 = false;

    if (gyro1 != -1)
        m_pigeon1 = new PigeonIMU(gyro1);

    if (gyro2 != -1)
        m_pigeon2 = new PigeonIMU(gyro2);
}


DualGyro::~DualGyro()
{
    if (m_pigeon1 != nullptr)
        delete m_pigeon1;
    
    if (m_pigeon2 != nullptr)
        delete m_pigeon2;
}


void DualGyro::Init()
{
   	m_gyroval1[0] = 0.0;
	m_gyroval1[1] = 0.0;
	m_gyroval1[2] = 0.0;
	
    m_gyroval2[0] = 0.0;
	m_gyroval2[1] = 0.0;
	m_gyroval2[2] = 0.0;

    m_heading1 = 0.0;
    m_heading2 = 0.0;

    m_gyrovalid1 = false;
    m_gyrovalid2 = false;
}


void DualGyro::Loop()
{
    if (m_pigeon1 != nullptr)
    {
        m_pigeon1->GetAccumGyro(m_gyroval1);
        m_heading1 = m_pigeon1->GetFusedHeading();
        m_gyrovalid1 = true;
    }

    if (m_pigeon2 != nullptr)
    {
        m_pigeon2->GetAccumGyro(m_gyroval2);    
        m_heading2 = m_pigeon1->GetFusedHeading();
        m_gyrovalid2 = true;
    }
    Dashboard();
}


void DualGyro::Stop()
{
}


bool DualGyro::GetHeading(double &heading)
{
    if (m_gyrovalid1)
    {
        heading = m_heading1;
        return true;
    }
    else
    if (m_gyrovalid2)
    {
        heading = m_heading2;
        return true;
    }
    return false;
}


void DualGyro::Dashboard()
{
	double heading;

	if (GetHeading(heading))
		SmartDashboard::PutNumber("GyroFused", heading);
}


void DualGyro::ZeroHeading()
{
    if (m_pigeon1 != nullptr)
    {
		m_pigeon1->SetFusedHeading(0, 0);
    }

    if (m_pigeon2 != nullptr)
    {
		m_pigeon2->SetFusedHeading(0, 0);
    }
}
