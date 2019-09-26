#include "Button.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::wpilibj::Joystick;

		Button::Button(Joystick *j, int num, const std::wstring &n)
		{
			numb = num;
			stick = j;
			name = n;
		}

		std::wstring Button::getName()
		{
			return name;
		}

		bool Button::on()
		{
			return state;
		}

		bool Button::held()
		{
			return held_Conflict;
		}

		bool Button::changed()
		{
			return changed_Conflict;
		}

		void Button::update()
		{
			cbutton = stick->getRawButton(numb);
			if (cbutton && (cbutton != lastState))
			{
				state = !state;
				changed_Conflict = true;

			}
			else
			{
				changed_Conflict = false;
			}

			held_Conflict = cbutton;
			lastState = cbutton;
		}

		void Button::reset()
		{
			state = false;
			lastState = false;
			changed_Conflict = false;
		}

		void Button::toggleOff()
		{
			reset();
			update();
		}

		void Button::toggleOn()
		{
			state = true;
		}
	}
}
