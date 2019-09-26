#include "Autonomous.h"
#include "Robot.h"
#include "Blueprint.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::wpilibj::Timer;
		using com::ctre::phoenix::motorcontrol::ControlMode;

		Autonomous::StageDataElement::StageDataElement(Autonomous *outerInstance) : outerInstance(outerInstance)
		{
		}

		Autonomous::Autonomous(Robot *robot)
		{
			this->robot = robot;
		}

		void Autonomous::setBlueprints(std::vector<Blueprint*> &b)
		{
			stageData = std::vector<StageDataElement*>(b.size());

			for (int i = 0; i < b.size(); i++)
			{
				stageData[i] = new StageDataElement(this);

				stageData[i]->blueprint = b[i];
				stageData[i]->entered = false;
			}
		}

		void Autonomous::start()
		{

			for (int i = 0; i < stageData.size(); i++)
			{
				stageData[i]->entered = false;
			}
			robot->driveLeftEnc.reset();
			robot->driveRightEnc.reset();

			robot->leftController.reset();
			robot->rightController.reset();

			stage = 0;
			initTime = Timer::getFPGATimestamp();

			timer = new Timer();
			timer->reset();
			timer->start();
		}

		bool Autonomous::checkStageTimeout()
		{
			if (stageData.empty())
			{
				return true;
			}
			if (stage < 0 || stage >= stageData.size())
			{
				robot->topLeft.set(ControlMode::PercentOutput, 0.0);
				robot->bottomLeft.set(ControlMode::PercentOutput, 0.0);
				robot->topRight.set(ControlMode::PercentOutput, 0.0);
				robot->bottomRight.set(ControlMode::PercentOutput, 0.0);
				return true;
			}

			if (timer->get() > stageData[stage]->blueprint.m_timeout)
			{

				printf(L"stage %d timed out\n", stage);
				nextStage();
				return true;
			}
			return false;
		}

		void Autonomous::nextStage()
		{
			printf(L"Stage Finished: %d\tTime: %f\tTotal Time: %f\n",stage,timer->get(),Timer::getFPGATimestamp() - initTime);
			timer->reset();
			stage++;

			if (stage >= stageData.size())
			{
				end();
			}
		}

		void Autonomous::stopAll()
		{
			stage = stageData.size();
		}

		void Autonomous::end()
		{
			std::wcout << L"-----" << std::endl;
			printf(L"Auto Finished:\tTotal Time: %f\n",Timer::getFPGATimestamp() - initTime);

			robot->rightController.reset();
			robot->leftController.reset();
		}

		void Autonomous::init()
		{
			std::wcout << L"Override me!" << std::endl;
		}

		void Autonomous::periodic()
		{
			int stageAtEntry = stage;
			if (checkStageTimeout())
			{
				return;
			}
			if (!stageData[stageAtEntry]->entered)
			{
				stageData[stageAtEntry]->blueprint.m_start.getAsInt();
				stageData[stageAtEntry]->entered = true;
			}

			if (stage == stageAtEntry)
			{
				stageData[stage]->blueprint.m_periodic.getAsInt();
			}
		}
	}
}
