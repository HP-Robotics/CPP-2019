#pragma once

#include "Autonomous.h"
#include <vector>

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
namespace frc { namespace robot { class Robot; } }

namespace frc
{
	namespace robot
	{



		class RightRocketFarAuto : public Autonomous
		{


		public:
			RightRocketFarAuto(Robot *robot);

			void init() override;

			virtual int goStart();

			virtual int turnStart();



			virtual int goPeriodic();

			virtual int turnPeriodic();



		};


	}
}
