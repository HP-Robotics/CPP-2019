#include "StepAuto.h"
#include "Robot.h"
#include "Blueprint.h"

namespace frc
{
	namespace robot
	{
		using com::ctre::phoenix::motorcontrol::ControlMode;
		using edu::wpi::first::wpilibj::DriverStation;

		StepAuto::StepAuto(Robot *robot) : Autonomous(robot)
		{
		}

		void StepAuto::init()
		{

			std::vector<Blueprint*> blueprints = {new Blueprint(1.0, this::holdStart, this::holdPeriodic)};
			setBlueprints(blueprints);

			start();
		}

		int StepAuto::holdStart()
		{
			robot->hatchController.configureGoal(5 - robot->hatchPot->get(), 500, 500, true);
			robot->hatchController.enable();

			return 0;
		}

		int StepAuto::holdPeriodic()
		{
			nextStage();
			return 0;
		}

		int StepAuto::goStart()
		{

				robot->driveLeftEnc.reset();
				robot->driveRightEnc.reset();

				robot->leftController.configureGoal(50, robot->max_traj_v, robot->max_traj_a, false);
				robot->rightController.configureGoal(50, robot->max_traj_v, robot->max_traj_a, false);


				robot->leftController.enable();
				robot->rightController.enable();

			return 0;
		}

		int StepAuto::backStart()
		{

			robot->driveLeftEnc.reset();
				robot->driveRightEnc.reset();

				robot->leftController.configureGoal(-24.0, 50, 50, false);
				robot->rightController.configureGoal(-24.0, 50, 50, false);


				robot->leftController.enable();
				robot->rightController.enable();

			return 0;
		}

		int StepAuto::goPeriodic()
		{
			if (robot->leftController.isPlanFinished() && robot->rightController.isPlanFinished())
			{

				robot->leftController.reset();
				robot->rightController.reset();

				nextStage();
			}
			return 0;
		}

		int StepAuto::backPeriodic()
		{
			if (robot->leftController.isPlanFinished() && robot->rightController.isPlanFinished())
			{

				robot->leftController.reset();
				robot->rightController.reset();

				nextStage();
			}
			return 0;
		}
	}
}
