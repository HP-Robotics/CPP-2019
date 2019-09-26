#pragma once

namespace frc
{
	namespace robot
	{

		using com::ctre::phoenix::motorcontrol::can::TalonSRX;

		using edu::wpi::first::wpilibj::PIDOutput;

		class TalonPIDOutput : public PIDOutput
		{

		public:
			TalonSRX *m;
			double mult = 0;
			virtual ~TalonPIDOutput()
			{
				delete m;
			}

			TalonPIDOutput(TalonSRX *motor, double multiplier);

			void pidWrite(double output) override;

		};

	}
}
