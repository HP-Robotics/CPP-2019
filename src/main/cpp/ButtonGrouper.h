#pragma once

#include <vector>

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
namespace frc { namespace robot { class Button; } }
namespace frc { namespace robot { class LiteButton; } }

namespace frc
{
	namespace robot
	{

		class ButtonGrouper
		{
		public:
			std::vector<Button*> group;
			LiteButton *lights;
			Button *lastButton;

			virtual ~ButtonGrouper()
			{
				delete lights;
				delete lastButton;
			}

			ButtonGrouper(std::vector<Button*> &g, LiteButton *lb);

			virtual void update();
		};
	}
}
