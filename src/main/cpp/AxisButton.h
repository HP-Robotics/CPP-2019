#pragma once

#include <string>

namespace frc
{
	namespace robot
	{

		using edu::wpi::first::wpilibj::Joystick;

		class AxisButton
		{
		private:
//JAVA TO C++ CONVERTER NOTE: Fields cannot have the same name as methods:
			bool held_Conflict = false; //stands for 'held', true if the Button is being actively held down
			double state = 0.0; //stands for 'state', true if the Button is pressed
			double lastState = 0.0; //stands for 'last state', stores the previous state of the Button
//JAVA TO C++ CONVERTER NOTE: Fields cannot have the same name as methods:
			bool changed_Conflict = false; //stands for 'changed', true if the Button's previous state does not match its current state
			double lastAxisTriggered = 0.0;
			double fudgeAxis = 0.0;
			double abutton = 0;
			std::wstring name;
			Joystick *stick;
			int numb = 0;

		public:
			virtual ~AxisButton()
			{
				delete stick;
			}

			AxisButton(Joystick *j, int num, const std::wstring &n);

			virtual std::wstring getName();

			//check if the Button is pressed
			virtual double getState();

			//check if the Button is held down
			virtual bool held();

			//check if the Button has changed
			virtual bool changed();

			//update the Button, should be called periodically
			virtual void update();

			//reset all values
			virtual void reset();

			virtual void toggleOff();

		};

	}
}
