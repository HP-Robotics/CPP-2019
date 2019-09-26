#pragma once

namespace frc
{
	namespace robot
	{

		using com::ctre::phoenix::motorcontrol::can::TalonSRX;

		using edu::wpi::first::wpilibj::PIDOutput;

		class SpinnyPIDOutput : public PIDOutput
		{


			/*CALYPSO IS VICTOR ATLAS IS TALON*/
		public:
			TalonSRX *m;
			TalonSRX *m2;
			TalonSRX *m3;
			TalonSRX *m4;
			double mult = 0;
			virtual ~SpinnyPIDOutput()
			{
				delete m;
				delete m2;
				delete m3;
				delete m4;
			}

			SpinnyPIDOutput(TalonSRX *motor, TalonSRX *motor2, TalonSRX *motor3, TalonSRX *motor4, double multiplier);

			void pidWrite(double output) override;

		};
	}
}
