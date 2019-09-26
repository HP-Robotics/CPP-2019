#pragma once

#include "Autonomous.h"
#include <vector>

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
namespace frc { namespace robot { class Robot; } }

namespace frc
{
	namespace robot
	{



		class RightRocketCloseAuto : public Autonomous
		{


		public:
			RightRocketCloseAuto(Robot *robot);

			void init() override;

			virtual int winchStart();

			virtual int winchPeriodic();

			virtual int goStart();





			virtual int goPeriodic();

			virtual int winchWaitStart();

			virtual int winchWaitPeriodic();

			virtual int elevatorStart();

			virtual int elevatorPeriodic();

			virtual int secondMoveStart();



		virtual int secondMovePeriodic();

		virtual int hatchDownStart();

		virtual int hatchDownPeriodic();

		virtual int backStart();

		virtual int backPeriodic();

		virtual int lowerStart();

		virtual int lowerPeriodic();


		};


	}
}
