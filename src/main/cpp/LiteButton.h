#pragma once

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
namespace frc { namespace robot { class Button; } }

namespace frc
{
	namespace robot
	{

		class LiteButton
		{

		public:
			virtual void light(Button *b);

			virtual void unlight(Button *b);

		};
	}
}
