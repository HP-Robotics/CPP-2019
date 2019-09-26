#pragma once

#include <vector>
#include <iostream>

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
namespace frc { namespace robot { class Robot; } }
namespace frc { namespace robot { class StageDataElement; } }
namespace frc { namespace robot { class Blueprint; } }

namespace frc
{
	namespace robot
	{

		using edu::wpi::first::wpilibj::Timer;

		class Autonomous
		{
		public:
			Robot *robot;
			Timer *timer;
			double initTime = 0;
			int stage = 0;
			std::vector<StageDataElement*> stageData;

		public:
			class StageDataElement
			{
				private:
					Autonomous *outerInstance;

				public:
					virtual ~StageDataElement()
					{
						delete blueprint;
						delete outerInstance;
					}

					StageDataElement(Autonomous *outerInstance);

				Blueprint *blueprint;
				bool entered = false;
			};

		public:
			virtual ~Autonomous()
			{
				delete robot;
				delete timer;
			}

			Autonomous(Robot *robot);

			virtual void setBlueprints(std::vector<Blueprint*> &b);

			virtual void start();

			virtual bool checkStageTimeout();

			virtual void nextStage();

			virtual void stopAll();
			virtual void end();

			virtual void init();

			virtual void periodic();
		};

	}
}
