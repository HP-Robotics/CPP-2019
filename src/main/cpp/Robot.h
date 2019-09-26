#pragma once

//====================================================================================================
//The Free Edition of Java to C++ Converter limits conversion output to 100 lines per file.

//To subscribe to the Premium Edition, visit our website:
//https://www.tangiblesoftwaresolutions.com/order/order-java-to-cplus.html
//====================================================================================================

#include <vector>
#include <iostream>
#include <stdexcept>

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
namespace frc { namespace robot { class SnazzyMotionPlanner; } }
namespace frc { namespace robot { class TalonPIDOutput; } }
namespace frc { namespace robot { class SnazzyPIDCalculator; } }
namespace frc { namespace robot { class DrivePIDOutput; } }
namespace frc { namespace robot { class DrivePIDSourceInches; } }
namespace frc { namespace robot { class TurnPIDOutput; } }
namespace frc { namespace robot { class SpinnyPIDOutput; } }
namespace frc { namespace robot { class LimelightAnglePIDSource; } }
namespace frc { namespace robot { class SpinnyPIDSource; } }
namespace frc { namespace robot { class Button; } }
namespace frc { namespace robot { class AxisButton; } }
namespace frc { namespace robot { class ButtonGrouper; } }
namespace frc { namespace robot { class LiteButton; } }
namespace frc { namespace robot { class StepAuto; } }

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

namespace frc
{
	namespace robot
	{


		using com::ctre::phoenix::motorcontrol::can::TalonSRX;

		using edu::wpi::first::networktables::NetworkTable;
		using edu::wpi::first::wpilibj::DigitalInput;
		using edu::wpi::first::wpilibj::Encoder;
		using edu::wpi::first::wpilibj::Joystick;
		using edu::wpi::first::wpilibj::TimedRobot;
		using edu::wpi::first::wpilibj::interfaces::Potentiometer;
		using jaci::pathfinder::Trajectory;

		/**
		 * The VM is configured to automatically run this class, and to call the
		 * functions corresponding to each mode, as described in the IterativeRobot
		 * documentation. If you change the name of this class or the package after
		 * creating this project, you must also update the build.gradle file in the
		 * project.
		 */
		class Robot : public TimedRobot
		{

	  public:
		  SnazzyMotionPlanner *hatchController;
		  TalonPIDOutput *hatchPIDOutput;
		  SnazzyMotionPlanner *winchController;
		  TalonPIDOutput *winchPIDOutput;
		  SnazzyMotionPlanner *elevatorController;
		  TalonPIDOutput *elevatorPIDOutput;
		  SnazzyMotionPlanner *leftController;
		  SnazzyMotionPlanner *rightController;
		  SnazzyPIDCalculator *spinCalculator;
		  SnazzyMotionPlanner *spinnyController;
		  DrivePIDOutput *rightPIDOutput;
		  DrivePIDOutput *leftPIDOutput;
		  DrivePIDSourceInches *leftInInches;
		  DrivePIDSourceInches *rightInInches;
		  TurnPIDOutput *spinOutput;
		  SpinnyPIDOutput *spinnyOutput;
		  LimelightAnglePIDSource *spinInput;
		  SpinnyPIDSource *spinnyInput;

	  private:
		  double addToLeft = 0.0;
		  double addToRight = 0.0;

	  public:
		  static constexpr int DRIVER_STICK1 = 0;
		  static constexpr int DRIVER_STICK2 = 1;
		  static constexpr int OPERATOR_BOX = 2;

		  static constexpr double HATCH_UP = 50.0; //ATLAS 90.0
		  static constexpr double HATCH_DOWN = 107.5; //ATLAS 155.0
		  static constexpr double HATCH_SAFE_BOTTOM = 210.0;
		  static constexpr double HATCH_SAFE_TOP = -1.0;
		  static constexpr double HATCH_EMERGENCY_DOWN = 195.0;

		  static constexpr double ENC_ERROR = 5;
		  static constexpr double HATCH_LEVEL1 = 100;
		  static constexpr double HATCH_LEVEL2 = 4335 - 225;
		  static constexpr double HATCH_LEVEL3 = 7810 - 225;
		  static constexpr double SAFE_UP = 1900.0;
		  static constexpr double CARGO_LEVEL1 = 2870;
		  static constexpr double CARGO_LEVEL2 = 6432;
		  static constexpr double CARGO_LEVEL3 = 9570;
		  static constexpr double CARGO_CARGO = 4700;
		  static constexpr double HOP_ELEVATOR = 750;
		  static constexpr double ELEVATOR_ERROR = 100;

		  //ANTI-FRANK
		  static const double DRIVE_ENC_TO_INCH;
		  static const double DRIVE_INCH_TO_ENC;

		  //PRO-FRANK
		  //final static double DRIVE_ENC_TO_INCH = Math.PI * 6.0 * (24.0/60.0) * (1.0/3.0) * (1.0/256.0)*(156.0/160.0);
			//final static double DRIVE_INCH_TO_ENC = 1/DRIVE_ENC_TO_INCH;

		  NetworkTable *table;
		  int seqNumber = 1;
		  bool trajStarted = false;

		  bool isUsingIntake = false;


		  /* public static final double hatchkA = 0.0000501017;
		  public static final double hatchkV = 0.000634177;
		  * OLD POT VALUES
		  */

		  static constexpr double max_traj_v = 90;
		  static constexpr double max_traj_a = 100;
		  static constexpr double max_traj_j = 300;

		  static constexpr double hatchkA = 0; //0.000501017;
		  static constexpr double hatchkV = 0; //0.00634177;
		  static constexpr double hatchP = 0.02;
		  static constexpr double hatchI = 0.0003;

		  static constexpr double winchP = 0.003;
		  static constexpr double winchI = 0.00001;
		  static constexpr double winch_max_a = 10000;
		  static constexpr double winch_max_v = 4000;

		  static constexpr double elevatorP = 0.00108;
		  static constexpr double elevatorI = 0.0002;
		  static constexpr double elevatorD = 0.02;
		  static constexpr double elevatorkA = 0.0; //0.000005;//0.000095086;
		  static constexpr double elevatorkV = 0; //0.00183371;
		  static constexpr double elevator_max_a = 8000;
		  static constexpr double elevator_max_v = 5555;

		  bool firstHop = false;
		  bool hopping = false;
		   double hoptime = 0;

		  static constexpr double spinP = 0.05;
		  static constexpr double spinI = 0.00001;
		  static constexpr double spinD = 0.2;

		  //FRANK
		  /*final double driveP = 0.3+0.4;
			final double driveI = 0.005+0.01;
			final double driveD = 1.0;
			final double drivekV = 0.00246*1.15;
		  final double drivekA = 0.0108;
		  final double drivetkA = 0.044;
		  final double drivetkV = 0.178;*/




		  //CALYPSO + ATLAS
		  static constexpr double driveP = 0.5 * .75;
		  static constexpr double driveI = 0.025;
		  static constexpr double driveD = 0.2;

//====================================================================================================
//End of the allowed output for the Free Edition of Java to C++ Converter.

//To subscribe to the Premium Edition, visit our website:
//https://www.tangiblesoftwaresolutions.com/order/order-java-to-cplus.html
//====================================================================================================
