#include "LeftRocketFarAuto.h"
#include "Robot.h"
#include "Blueprint.h"

namespace frc
{
	namespace robot
	{
		using com::ctre::phoenix::motorcontrol::ControlMode;
		using edu::wpi::first::wpilibj::DriverStation;

		LeftRocketFarAuto::LeftRocketFarAuto(Robot *robot) : Autonomous(robot)
		{
		}

		void LeftRocketFarAuto::init()
		{

			std::vector<Blueprint*> blueprints =
			{
				new Blueprint(5.0, this::goStart, this::goPeriodic),
				new Blueprint(1.0, this::turnStart, this::turnPeriodic)
			};
			setBlueprints(blueprints);

			start();
		}

		int LeftRocketFarAuto::goStart()
		{

				//robot.leftController.configureTrajectory(robot.rocketLeftFarTraj.getLeftTrajectory(), false);
				//robot.rightController.configureTrajectory(robot.rocketLeftFarTraj.getRightTrajectory(), false);


				robot->leftController.enable();
				robot->rightController.enable();

			return 0;
		}

		int LeftRocketFarAuto::turnStart()
		{

			robot->leftController.configureGoal(-32, 100, 100, false);
			robot->rightController.configureGoal(32, 100, 100, false);


			robot->leftController.enable();
			robot->rightController.enable();

			return 0;
		}

		int LeftRocketFarAuto::goPeriodic()
		{
			if (robot->leftController.isPlanFinished() && robot->rightController.isPlanFinished())
			{

				robot->leftController.reset();
				robot->rightController.reset();

				nextStage();
			}
			return 0;
		}

		int LeftRocketFarAuto::turnPeriodic()
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
