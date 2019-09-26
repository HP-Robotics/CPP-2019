#pragma once

#include "Autonomous.h"
#include <vector>

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
namespace frc { namespace robot { class Robot; } }

namespace frc
{
	namespace robot
	{



		class LeftRocketToFeederAuto : public Autonomous
		{


		public:
			LeftRocketToFeederAuto(Robot *robot);

			void init() override;


			virtual int goStart();



			virtual int goPeriodic();


			virtual int secondMoveStart();



		virtual int secondMovePeriodic();
		};

	}
}
