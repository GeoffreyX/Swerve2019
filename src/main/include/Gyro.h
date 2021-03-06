/**
 *  Gyro.h
 *  Date: 5/21/19
 *  Last Edited By: Geoffrey Xue
 *  Huge credit to team 2910 for the entirety of this code system
 */


#ifndef SRC_Gyro_H_
#define SRC_Gyro_H_


#include <frc\WPILib.h>
#include <ctre\Phoenix.h>


using namespace frc;


class DualGyro
{
public:
	DualGyro(int gyro1 = -1, int gryo2 = -1);
	~DualGyro();
	void Init();
	void Loop();
	void Stop();
    bool GetHeading(double &heading);
    void Dashboard();
    void ZeroHeading();

protected:
    PigeonIMU *m_pigeon1;
    PigeonIMU *m_pigeon2;
	double m_gyroval1[3];
    double m_gyroval2[3];
    double m_heading1;
    double m_heading2;
    bool m_gyrovalid1;
    bool m_gyrovalid2;
};


#endif /* SRC_Gyro_H_ */
