#include "SpinnyPIDOutput.h"

namespace frc
{
	namespace robot
	{
		using com::ctre::phoenix::motorcontrol::ControlMode;
		using com::ctre::phoenix::motorcontrol::can::TalonSRX;
		using com::ctre::phoenix::motorcontrol::can::VictorSPX;
		using edu::wpi::first::wpilibj::PIDOutput;

		SpinnyPIDOutput::SpinnyPIDOutput(TalonSRX *motor, TalonSRX *motor2, TalonSRX *motor3, TalonSRX *motor4, double multiplier)
		{
			m = motor;
			m2 = motor2;
			m3 = motor3;
			m4 = motor4;
			mult = multiplier;
		}

		void SpinnyPIDOutput::pidWrite(double output)
		{
			m->set(ControlMode::PercentOutput, mult * output);
			m2->set(ControlMode::PercentOutput, mult * output);
			m3->set(ControlMode::PercentOutput, mult * output);
			m4->set(ControlMode::PercentOutput, mult * output);
		}
	}
}
