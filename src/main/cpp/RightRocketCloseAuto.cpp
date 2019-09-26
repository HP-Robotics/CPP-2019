#include "RightRocketCloseAuto.h"
#include "Robot.h"
#include "Blueprint.h"

namespace frc
{
	namespace robot
	{
		using com::ctre::phoenix::motorcontrol::ControlMode;
		using edu::wpi::first::wpilibj::DriverStation;

		RightRocketCloseAuto::RightRocketCloseAuto(Robot *robot) : Autonomous(robot)
		{
		}

		void RightRocketCloseAuto::init()
		{

			std::vector<Blueprint*> blueprints =
			{
				new Blueprint(0.5, this::winchStart, this::winchPeriodic),
				new Blueprint(5.0, this::goStart, this::goPeriodic),
				new Blueprint(45.0, this::winchWaitStart, this::winchWaitPeriodic),
				new Blueprint(1.0, this::elevatorStart, this::elevatorPeriodic),
				new Blueprint(2.0, this::secondMoveStart, this::secondMovePeriodic),
				new Blueprint(2.0, this::hatchDownStart, this::hatchDownPeriodic),
				new Blueprint(5.0, this::backStart, this::backPeriodic),
				new Blueprint(5.0, this::lowerStart, this::lowerPeriodic)
			};
			setBlueprints(blueprints);

			start();
		}

		int RightRocketCloseAuto::winchStart()
		{

			robot->winchController.configureGoal(robot->winchArray[1] - robot->winchEnc->get(), robot->winch_max_v, robot->winch_max_a, true);

			robot->winchController.enable();

			return 0;
		}

		int RightRocketCloseAuto::winchPeriodic()
		{
			nextStage();
			return 0;
		}

		int RightRocketCloseAuto::goStart()
		{
				robot->driveLeftEnc.reset();
				robot->driveRightEnc.reset();

				//robot.leftController.configureTrajectory(robot.rightRocketCloseTraj.getLeftTrajectory(), false);
				//robot.rightController.configureTrajectory(robot.rightRocketCloseTraj.getRightTrajectory(), false);


				robot->leftController.enable();
				robot->rightController.enable();

			return 0;
		}

		int RightRocketCloseAuto::goPeriodic()
		{
			if (robot->leftController.isPlanFinished() && robot->rightController.isPlanFinished())
			{

				robot->leftController.reset();
				robot->rightController.reset();

				nextStage();
			}
			return 0;
		}

		int RightRocketCloseAuto::winchWaitStart()
		{

			return 0;
		}

		int RightRocketCloseAuto::winchWaitPeriodic()
		{
			if (robot->winchDown->get())
			{ //TODO ATLAS REMOVE THE TRUE WE GOTTA SAFETY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				nextStage();
			}
			return 0;
		}

		int RightRocketCloseAuto::elevatorStart()
		{

			robot->elevatorController.configureGoal(robot->HATCH_LEVEL2 - robot->elevatorEnc->get(), robot->elevator_max_v, robot->elevator_max_a, true);

			robot->elevatorController.enable();

			return 0;
		}

		int RightRocketCloseAuto::elevatorPeriodic()
		{
			nextStage();

			return 0;
		}

		int RightRocketCloseAuto::secondMoveStart()
		{
			robot->driveLeftEnc.reset();
			robot->driveRightEnc.reset();


			robot->leftController.configureGoal(12, 100, 100, false);
			robot->rightController.configureGoal(12, 100, 100, false);


			robot->leftController.enable();
			robot->rightController.enable();

		return 0;
		}

		int RightRocketCloseAuto::secondMovePeriodic()
		{
			if (robot->leftController.isPlanFinished() && robot->rightController.isPlanFinished() && robot->elevatorController.isPlanFinished())
			{

				robot->leftController.reset();
				robot->rightController.reset();

				nextStage();
			}
			return 0;
		}

		int RightRocketCloseAuto::hatchDownStart()
		{
			robot->hatchController.configureGoal(robot->HATCH_DOWN - robot->hatchPot->get(), 500, 500, true);
			robot->hatchController.enable();
			return 0;
		}

		int RightRocketCloseAuto::hatchDownPeriodic()
		{
			if (robot->hatchController.isPlanFinished())
			{

				nextStage();
			}
			return 0;
		}

		int RightRocketCloseAuto::backStart()
		{
			robot->driveLeftEnc.reset();
			robot->driveRightEnc.reset();

			robot->leftController.configureGoal(-12.0, 100, 100, false);
			robot->rightController.configureGoal(-12.0, 100, 100, false);

			robot->leftController.enable();
			robot->rightController.enable();

			return 0;
		}

		int RightRocketCloseAuto::backPeriodic()
		{

			if (robot->leftController.isPlanFinished() && robot->rightController.isPlanFinished())
			{
				robot->leftController.reset();
				robot->rightController.reset();
				nextStage();
			}

			return 0;
		}

		int RightRocketCloseAuto::lowerStart()
		{

			robot->elevatorController.configureGoal(0 - robot->elevatorEnc->get(), robot->elevator_max_v, robot->elevator_max_a, false);

			robot->elevatorController.enable();

			return 0;
		}

		int RightRocketCloseAuto::lowerPeriodic()
		{
			nextStage();
			return 0;
		}
	}
}
