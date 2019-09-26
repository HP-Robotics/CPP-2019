#include "LiteButton.h"
#include "Button.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::wpilibj::smartdashboard::SmartDashboard;

		void LiteButton::light(Button *b)
		{
			SmartDashboard::putBoolean(b->getName(), true);
		}

		void LiteButton::unlight(Button *b)
		{
			SmartDashboard::putBoolean(b->getName(), false);

		}
	}
}
