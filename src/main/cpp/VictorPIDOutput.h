#pragma once

namespace frc
{
	namespace robot
	{

		using com::ctre::phoenix::motorcontrol::can::VictorSPX;

		using edu::wpi::first::wpilibj::PIDOutput;

		class VictorPIDOutput : public PIDOutput
		{

		public:
			VictorSPX *m;
			double mult = 0;
			virtual ~VictorPIDOutput()
			{
				delete m;
			}

			VictorPIDOutput(VictorSPX *motor, double multiplier);

			void pidWrite(double output) override;

		};

	}
}
