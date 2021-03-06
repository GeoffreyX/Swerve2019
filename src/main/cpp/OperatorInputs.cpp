/**
 *  OperatorInputs.cpp
 *  Date: 5/21/19
 *  Last Edited By: Geoffrey Xue
 *  Huge credit to team 2910 for the entirety of this code system
 */


#include "OperatorInputs.h"
#include "Const.h"
#include <cmath>


OperatorInputs::OperatorInputs()
{
	m_joystick = nullptr;
	if (INP_JOYSTICK != -1)
		m_joystick = new Joystick(INP_JOYSTICK);
	if (INP_XBOX_1 != -1)
		m_xbox.push_back(new XboxController(INP_XBOX_1));
	if (INP_XBOX_2 != -1)
		m_xbox.push_back(new XboxController(INP_XBOX_2));
}


OperatorInputs::~OperatorInputs()
{
	if (m_joystick != nullptr)
		delete m_joystick;
	for (std::vector<XboxController * >::iterator it = m_xbox.begin() ; it != m_xbox.end(); it++)
	     delete (*it);
	m_xbox.clear();
}


double OperatorInputs::xBoxLeftX(unsigned int i)
{
	if (i < m_xbox.size())
		return deadzoneFilterX(INVERT_X_AXIS * m_xbox[i]->GetX(GenericHID::JoystickHand::kLeftHand));
	return false;
}


double OperatorInputs::xBoxRightX(unsigned int i)
{
	if (i < m_xbox.size())
		return deadzoneFilterX(m_xbox[i]->GetX(GenericHID::JoystickHand::kRightHand));
	return false;
}


double OperatorInputs::xBoxLeftY(unsigned int i)
{
	if (i < m_xbox.size())
		return deadzoneFilterY(INVERT_Y_AXIS * m_xbox[i]->GetY(GenericHID::JoystickHand::kLeftHand));
	return false;
}


double OperatorInputs::xBoxRightY(unsigned int i)
{
	if (i < m_xbox.size())
		return deadzoneFilterY(m_xbox[i]->GetY(GenericHID::JoystickHand::kRightHand));
	return false;
}


bool OperatorInputs::xBoxAButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(A_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxAButton_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxBButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(B_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxBButton_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxXButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(X_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxXButton_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxYButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(Y_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxYButton_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxLeftBumper(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(LEFT_BUMPER);

		if (choice == kToggle)
			return toggle("xBoxLeftBumper_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxRightBumper(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(RIGHT_BUMPER);

		if (choice == kToggle)
			return toggle("xBoxRightBumper_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxLeftTrigger(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		double axis = m_xbox[i]->GetRawAxis(XBOX_LEFT_TRIGGER_AXIS - 10);

		if (choice == kToggle)
			return toggle("xBoxLeftTrigger_" + i, (LEFT_TRIGGER_MIN <= axis) && (axis <= LEFT_TRIGGER_MAX));
		if (choice == kHold)
			return ((LEFT_TRIGGER_MIN <= axis) && (axis <= LEFT_TRIGGER_MAX));
	}
	return false;
}


bool OperatorInputs::xBoxRightTrigger(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		double axis = m_xbox[i]->GetRawAxis(XBOX_RIGHT_TRIGGER_AXIS - 10);

		if (choice == kToggle)
			return toggle("xBoxRightTrigger_" + i, (RIGHT_TRIGGER_MIN <= axis && axis <= RIGHT_TRIGGER_MAX));
		if (choice == kHold)
			return (RIGHT_TRIGGER_MIN <= axis && axis <= RIGHT_TRIGGER_MAX);
	}
	return false;
}


bool OperatorInputs::xBoxStartButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(START_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxStartButton_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxBackButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(BACK_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxBackButton_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadUp(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = (m_xbox[i]->GetPOV() == 0);

		if (choice == kToggle)
			return toggle("xBoxDPadUp_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadUpRight(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = (m_xbox[i]->GetPOV() == 45);

		if (choice == kToggle)
			return toggle("xBoxDPadUpRight_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadRight(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = (m_xbox[i]->GetPOV() == 90);

		if (choice == kToggle)
			return toggle("xBoxDPadRight_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadDownRight(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
	bool button = (m_xbox[i]->GetPOV() == 135);

	if (choice == kToggle)
		return toggle("xBoxDPadDownRight_" + i, button);
	if (choice == kHold)
		return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadDown(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
	bool button = (m_xbox[i]->GetPOV() == 180);

	if (choice == kToggle)
		return toggle("xBoxDPadDown_" + i, button);
	if (choice == kHold)
		return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadDownLeft(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = (m_xbox[i]->GetPOV() == 225);

		if (choice == kToggle)
			return toggle("xBoxDPadDownLeft_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadLeft(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = (m_xbox[i]->GetPOV() == 270);

		if (choice == kToggle)
			return toggle("xBoxDPadLeft_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadUpLeft(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = (m_xbox[i]->GetPOV() == 315);

		if (choice == kToggle)
			return toggle("xBoxDPadUpLeft_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxL3(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(L3_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxL3_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxR3(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(R3_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxR3_" + i, button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBox(int Button, ToggleChoice choice, unsigned int i)
{
	switch (Button)
	{
	case A_BUTTON:
		return xBoxAButton(choice, i);
	case B_BUTTON:
		return xBoxBButton(choice, i);
	case X_BUTTON:
		return xBoxXButton(choice, i);
	case Y_BUTTON:
		return xBoxYButton(choice, i);
	case LEFT_BUMPER:
		return xBoxLeftBumper(choice, i);
	case RIGHT_BUMPER:
		return xBoxRightBumper(choice, i);
	case XBOX_LEFT_TRIGGER_AXIS:
		return xBoxLeftTrigger(choice, i);
	case XBOX_RIGHT_TRIGGER_AXIS:
		return xBoxRightTrigger(choice, i);
	case START_BUTTON:
		return xBoxStartButton(choice, i);
	case BACK_BUTTON:
		return xBoxBackButton(choice, i);
	case L3_BUTTON:
		return xBoxL3(choice, i);
	case R3_BUTTON:
		return xBoxR3(choice, i);
	}
	return false;	
}


double OperatorInputs::joystickX()
{
	if (m_joystick != nullptr)
		return deadzoneFilterX(INVERT_X_AXIS * m_joystick->GetX());
	return false;
}


double OperatorInputs::joystickY()
{
	if (m_joystick != nullptr)
		return deadzoneFilterY(INVERT_Y_AXIS * m_joystick->GetY());
	return false;
}


double OperatorInputs::joystickZ()
{
	if (m_joystick != nullptr)
		return deadzoneFilterZ(m_joystick->GetZ());
	return false;
}


bool OperatorInputs::joystickAxis0Left(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		double axis = m_joystick->GetRawAxis(JOYSTICK_X_AXIS);

		if (choice == kToggle)
			return toggle("joystickAxis0Left", (AXIS0_LEFT_MIN <= axis && axis <= AXIS0_LEFT_MAX));
		if (choice == kHold)
			return (AXIS0_LEFT_MIN <= axis && axis <= AXIS0_LEFT_MAX);
		return false;
	}
	return false;
}


bool OperatorInputs::joystickAxis0Right(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		double axis = m_joystick->GetRawAxis(JOYSTICK_X_AXIS);

		if (choice == kToggle)
			return toggle("joystickAxis0Right", (AXIS0_RIGHT_MIN <= axis && axis <= AXIS0_RIGHT_MAX));
		if (choice == kHold)
			return (AXIS0_RIGHT_MIN <= axis && axis <= AXIS0_RIGHT_MAX);
		return false;
	}
	return false;
}


bool OperatorInputs::joystickAxis1Back(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		double axis = m_joystick->GetRawAxis(JOYSTICK_Y_AXIS);

		if (choice == kToggle)
			return toggle("joystickAxis1Back", (AXIS1_BACK_MIN <= axis && axis <= AXIS1_BACK_MAX));
		if (choice == kHold)
			return (AXIS1_BACK_MIN <= axis && axis <= AXIS1_BACK_MAX);
		return false;
	}
	return false;
}


bool OperatorInputs::joystickAxis1Forward(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		double axis = m_joystick->GetRawAxis(JOYSTICK_Y_AXIS);

		if (choice == kToggle)
			return toggle("joystickAxis1Forward", (AXIS1_FORWARD_MIN <= axis && axis <= AXIS1_FORWARD_MAX));
		if (choice == kHold)
			return (AXIS1_FORWARD_MIN <= axis && axis <= AXIS1_FORWARD_MAX);
		return false;
	}
	return false;
}


bool OperatorInputs::joystickTrigger(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		bool button = m_joystick->GetTrigger();

		if (choice == kToggle)
			return toggle("joystickTrigger", button);
		if (choice == kHold)
			return button;
		return false;
	}
	return false;
}


bool OperatorInputs::joystickButton2(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		bool button = m_joystick->GetRawButton(2);

		if (choice == kToggle)
			return toggle("joystickbutton2", button);
		if (choice == kHold)
			return button;
		return false;
	}
	return false;
}


bool OperatorInputs::joystickButton3(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		bool button = m_joystick->GetRawButton(3);

		if (choice == kToggle)
			return toggle("joystickbutton3", button);
		if (choice == kHold)
			return button;
		return false;
	}
	return false;
}


bool OperatorInputs::joystickButton5(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		bool button = m_joystick->GetRawButton(5);

		if (choice == kToggle)
			return toggle("joystickbutton5", button);
		if (choice == kHold)
			return button;
		return false;
	}
	return false;
}


bool OperatorInputs::joystickButton6(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		bool button = m_joystick->GetRawButton(6);

		if (choice == kToggle)
			return toggle("joystickbutton6", button);
		if (choice == kHold)
			return button;
		return false;
	}
	return false;
}


bool OperatorInputs::joystickButton7(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		bool button = m_joystick->GetRawButton(7);

		if (choice == kToggle)
			return toggle("joystickbutton7", button);
		if (choice == kHold)
			return button;
		return false;
	}
	return false;
}


bool OperatorInputs::joystickButton8(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		bool button = m_joystick->GetRawButton(8);

		if (choice == kToggle)
			return toggle("joystickbutton8", button);
		if (choice == kHold)
			return button;
		return false;
	}
	return false;
}


bool OperatorInputs::joystickButton9(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		bool button = m_joystick->GetRawButton(9);

		if (choice == kToggle)
			return toggle("joystickbutton9", button);
		if (choice == kHold)
			return button;
		return false;
	}
	return false;
}


bool OperatorInputs::joystickButton10(ToggleChoice choice)
{
	if (m_joystick != nullptr)
	{
		bool button = m_joystick->GetRawButton(10);

		if (choice == kToggle)
			return toggle("joystickbutton10", button);
		if (choice == kHold)
			return button;
		return false;
	}
	return false;
}


bool OperatorInputs::toggle(string buttonname, bool buttonval)
{
	bool toggleval = !m_togglebuttons[buttonname] && buttonval;
	m_togglebuttons[buttonname] = buttonval;
	return toggleval;
}


double OperatorInputs::deadzoneFilterX(double joyStickValue)
{
	if (abs(joyStickValue) <= DEADZONE_X)
	{
		return 0;
	}
	double sub = joyStickValue / abs(joyStickValue);
	return (joyStickValue - (sub * DEADZONE_X)) / (1.0 - DEADZONE_X);
}


double OperatorInputs::deadzoneFilterY(double joyStickValue)
{
	if (abs(joyStickValue) <= DEADZONE_Y)
	{
		return 0;
	}
	double sub = joyStickValue / abs(joyStickValue);
	return (joyStickValue - (sub * DEADZONE_Y)) / (1.0 - DEADZONE_Y);
}


double OperatorInputs::deadzoneFilterZ(double joyStickValue)
{
	if (abs(joyStickValue) <= DEADZONE_Z)
	{
		return 0;
	}
	double sub = joyStickValue / abs(joyStickValue);
	return (joyStickValue - (sub * DEADZONE_Z)) / (1.0-DEADZONE_Z);
}
