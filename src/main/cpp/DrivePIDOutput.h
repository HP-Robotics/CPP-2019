#pragma once

namespace frc
{
	namespace robot
	{

		using com::ctre::phoenix::motorcontrol::can::TalonSRX;

		using edu::wpi::first::wpilibj::PIDOutput;

		class DrivePIDOutput : public PIDOutput
		{


			/*CALYPSO IS VICTOR ATLAS IS TALON*/
		public:
			TalonSRX *m;
			TalonSRX *m2;
			double mult = 0;
			virtual ~DrivePIDOutput()
			{
				delete m;
				delete m2;
			}

			DrivePIDOutput(TalonSRX *motor, TalonSRX *motor2, double multiplier);

			void pidWrite(double output) override;

		};

	}
}
