#include "ButtonGrouper.h"
#include "Button.h"
#include "LiteButton.h"

namespace frc
{
	namespace robot
	{

		ButtonGrouper::ButtonGrouper(std::vector<Button*> &g, LiteButton *lb)
		{
			group = g;
			lights = lb;
		}

		void ButtonGrouper::update()
		{
			for (auto b : group)
			{
				b->update();
				if (b->changed())
				{
					lastButton = b;
				}
			}

			for (auto b : group)
			{
				if (b == lastButton)
				{
					lights->light(b);
				}
				else
				{
					b->toggleOff();
					lights->unlight(b);
					//System.out.println(b + " unlight");
				}
			}

		}
	}
}
