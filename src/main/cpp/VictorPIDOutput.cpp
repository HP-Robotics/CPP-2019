#include "VictorPIDOutput.h"

namespace frc
{
	namespace robot
	{
		using com::ctre::phoenix::motorcontrol::ControlMode;
		using com::ctre::phoenix::motorcontrol::can::VictorSPX;
		using edu::wpi::first::wpilibj::PIDOutput;

		VictorPIDOutput::VictorPIDOutput(VictorSPX *motor, double multiplier)
		{
			m = motor;
			mult = multiplier;
		}

		void VictorPIDOutput::pidWrite(double output)
		{

				m->set(ControlMode::PercentOutput, mult * output);
		}
	}
}
