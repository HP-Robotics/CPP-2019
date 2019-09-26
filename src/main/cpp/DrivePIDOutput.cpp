#include "DrivePIDOutput.h"

namespace frc
{
	namespace robot
	{
		using com::ctre::phoenix::motorcontrol::ControlMode;
		using com::ctre::phoenix::motorcontrol::can::TalonSRX;
		using com::ctre::phoenix::motorcontrol::can::VictorSPX;
		using edu::wpi::first::wpilibj::PIDOutput;

		DrivePIDOutput::DrivePIDOutput(TalonSRX *motor, TalonSRX *motor2, double multiplier)
		{
			m = motor;
			m2 = motor2;
			mult = multiplier;
		}

		void DrivePIDOutput::pidWrite(double output)
		{
			m->set(ControlMode::PercentOutput, mult * output);
			m2->set(ControlMode::PercentOutput, mult * output);
		}
	}
}
