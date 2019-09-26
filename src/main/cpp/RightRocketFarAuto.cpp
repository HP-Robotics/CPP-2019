#include "RightRocketFarAuto.h"
#include "Robot.h"
#include "Blueprint.h"

namespace frc
{
	namespace robot
	{
		using com::ctre::phoenix::motorcontrol::ControlMode;
		using edu::wpi::first::wpilibj::DriverStation;

		RightRocketFarAuto::RightRocketFarAuto(Robot *robot) : Autonomous(robot)
		{
		}

		void RightRocketFarAuto::init()
		{

			std::vector<Blueprint*> blueprints =
			{
				new Blueprint(5.0, this::goStart, this::goPeriodic),
				new Blueprint(1.0, this::turnStart, this::turnPeriodic)
			};
			setBlueprints(blueprints);

			start();
		}

		int RightRocketFarAuto::goStart()
		{

				//robot.leftController.configureTrajectory(robot.rocketRightFarTraj.getLeftTrajectory(), false);
				//robot.rightController.configureTrajectory(robot.rocketRightFarTraj.getRightTrajectory(), false);


				robot->leftController.enable();
				robot->rightController.enable();

			return 0;
		}

		int RightRocketFarAuto::turnStart()
		{

			robot->leftController.configureGoal(32, 100, 100, false);
			robot->rightController.configureGoal(-32, 100, 100, false);


			robot->leftController.enable();
			robot->rightController.enable();

			return 0;
		}

		int RightRocketFarAuto::goPeriodic()
		{
			if (robot->leftController.isPlanFinished() && robot->rightController.isPlanFinished())
			{

				robot->leftController.reset();
				robot->rightController.reset();

				nextStage();
			}
			return 0;
		}

		int RightRocketFarAuto::turnPeriodic()
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
