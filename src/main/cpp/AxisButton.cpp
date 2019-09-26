#include "AxisButton.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::wpilibj::Joystick;

		AxisButton::AxisButton(Joystick *j, int num, const std::wstring &n)
		{
			numb = num;
			stick = j;
			name = n;
		}

		std::wstring AxisButton::getName()
		{
			return name;
		}

		double AxisButton::getState()
		{
			return state;
		}

		bool AxisButton::held()
		{
			return held_Conflict;
		}

		bool AxisButton::changed()
		{
			return changed_Conflict;
		}

		void AxisButton::update()
		{
			abutton = stick->getRawAxis(numb);
			//Make abutton less bad
			if (abutton >= -0.09 || abutton <= 0.09)
			{
				fudgeAxis = 0.0;
			}
			if (abutton > 0.5)
			{
				fudgeAxis = 1.0;
			}
			if (abutton < -0.5)
			{
				fudgeAxis = -1.0;
			}

			//System.out.println("RA: " + abutton + " FA: " + fudgeAxis);

			if (fudgeAxis != 0 && (fudgeAxis != lastState) && fudgeAxis != lastAxisTriggered)
			{
				state = fudgeAxis;
				changed_Conflict = true;
				lastAxisTriggered = fudgeAxis;
				//System.out.println("State switched to " + state);

			}
			else if (fudgeAxis != 0 && (fudgeAxis == lastState && fudgeAxis != lastAxisTriggered))
			{
				state = 0.0;
				changed_Conflict = true;
				lastAxisTriggered = fudgeAxis;
				//System.out.println("State off: " + state);
			}
			else if (fudgeAxis == 0)
			{
				lastAxisTriggered = 0;
			}
			else
			{
				state = lastState;
				changed_Conflict = false;
				//System.out.println("State not switched." + lastState + "    " + lastAxisState);
			}

			held_Conflict = fudgeAxis != 0.0;
			lastState = state;

		}

		void AxisButton::reset()
		{
			state = 0.0;
			lastState = 0.0;
			changed_Conflict = false;
		}

		void AxisButton::toggleOff()
		{
			reset();
			update();
		}
	}
}
