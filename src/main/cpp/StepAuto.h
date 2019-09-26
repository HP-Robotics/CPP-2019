#pragma once

#include "Autonomous.h"
#include <vector>

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
namespace frc { namespace robot { class Robot; } }

namespace frc
{
	namespace robot
	{



		class StepAuto : public Autonomous
		{


		public:
			StepAuto(Robot *robot);

			void init() override;

			virtual int holdStart();

			virtual int holdPeriodic();

			virtual int goStart();

			virtual int backStart();



			virtual int goPeriodic();

			virtual int backPeriodic();



		};


	}
}
