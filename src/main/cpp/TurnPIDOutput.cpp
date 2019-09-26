#include "TurnPIDOutput.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::wpilibj::PIDOutput;

		void TurnPIDOutput::pidWrite(double output)
		{
			m_value = output;
		}
	}
}
