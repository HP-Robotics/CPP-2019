#include "LeftRocketToFeederAuto.h"
#include "Robot.h"
#include "Blueprint.h"

namespace frc
{
	namespace robot
	{
		using com::ctre::phoenix::motorcontrol::ControlMode;
		using edu::wpi::first::wpilibj::DriverStation;

		LeftRocketToFeederAuto::LeftRocketToFeederAuto(Robot *robot) : Autonomous(robot)
		{
		}

		void LeftRocketToFeederAuto::init()
		{

			std::vector<Blueprint*> blueprints =
			{
				new Blueprint(5.0, this::goStart, this::goPeriodic),
				new Blueprint(5.0, this::secondMoveStart, this::secondMovePeriodic)
			};
			setBlueprints(blueprints);

			start();
		}

		int LeftRocketToFeederAuto::goStart()
		{

			robot->driveLeftEnc.reset();
			robot->driveRightEnc.reset();


			//robot.rightController.configureTrajectory(robot.leftCloseToShipTraj.getInvertedLeftTrajectory(), false);
			//robot.leftController.configureTrajectory(robot.leftCloseToShipTraj.getInvertedRightTrajectory(), false);


			robot->leftController.enable();
			robot->rightController.enable();

			return 0;
		}

		int LeftRocketToFeederAuto::goPeriodic()
		{
			if (robot->leftController.isPlanFinished() && robot->rightController.isPlanFinished())
			{

				robot->leftController.reset();
				robot->rightController.reset();

				nextStage();
			}
			return 0;
		}

		int LeftRocketToFeederAuto::secondMoveStart()
		{
			robot->driveLeftEnc.reset();
			robot->driveRightEnc.reset();


			//robot.rightController.configureTrajectory(robot.leftShipToFeederTraj.getRightTrajectory(), false);
			//robot.leftController.configureTrajectory(robot.leftShipToFeederTraj.getLeftTrajectory(), false);


			robot->leftController.enable();
			robot->rightController.enable();

		return 0;
		}

		int LeftRocketToFeederAuto::secondMovePeriodic()
		{
			if (robot->leftController.isPlanFinished() && robot->rightController.isPlanFinished() && robot->elevatorController.isPlanFinished())
			{

				robot->leftController.reset();
				robot->rightController.reset();

				nextStage();
			}
			return 0;
		}
	}
}
