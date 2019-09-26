#pragma once

namespace frc
{
	namespace robot
	{

		using edu::wpi::first::wpilibj::PIDOutput;

		class TurnPIDOutput : public PIDOutput
		{

		public:
			double m_value = 0.0;

			void pidWrite(double output) override;

		};

	}
}
