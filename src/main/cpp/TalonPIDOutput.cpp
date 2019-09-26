#include "TalonPIDOutput.h"

namespace frc
{
	namespace robot
	{
		using com::ctre::phoenix::motorcontrol::ControlMode;
		using com::ctre::phoenix::motorcontrol::can::TalonSRX;
		using edu::wpi::first::wpilibj::PIDOutput;

		TalonPIDOutput::TalonPIDOutput(TalonSRX *motor, double multiplier)
		{
			m = motor;
			mult = multiplier;
		}

		void TalonPIDOutput::pidWrite(double output)
		{

				m->set(ControlMode::PercentOutput, mult * output);
		}
	}
}
