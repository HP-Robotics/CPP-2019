#pragma once

#include <string>

namespace frc
{
	namespace robot
	{

		using edu::wpi::first::wpilibj::Joystick;

		class Button
		{
		private:
//JAVA TO C++ CONVERTER NOTE: Fields cannot have the same name as methods:
			bool held_Conflict = false; //stands for 'held', true if the Button is being actively held down
			bool state = false; //stands for 'state', true if the Button is pressed
			bool lastState = false; //stands for 'last state', stores the previous state of the Button
//JAVA TO C++ CONVERTER NOTE: Fields cannot have the same name as methods:
			bool changed_Conflict = false; //stands for 'changed', true if the Button's previous state does not match its current state
			bool cbutton = false;
			std::wstring name;
			Joystick *stick;
			int numb = 0;

		public:
			virtual ~Button()
			{
				delete stick;
			}

			Button(Joystick *j, int num, const std::wstring &n);

			virtual std::wstring getName();

			//check if the Button is pressed
			virtual bool on();

			//check if the Button is held down
			virtual bool held();

			//check if the Button has changed
			virtual bool changed();

			//update the Button, should be called periodically
			virtual void update();

			//reset all values
			virtual void reset();

			virtual void toggleOff();

			virtual void toggleOn();

		};

	}
}
