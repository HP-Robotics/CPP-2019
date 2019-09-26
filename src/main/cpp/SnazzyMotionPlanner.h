#pragma once

//====================================================================================================
//The Free Edition of Java to C++ Converter limits conversion output to 100 lines per file.

//To subscribe to the Premium Edition, visit our website:
//https://www.tangiblesoftwaresolutions.com/order/order-java-to-cplus.html
//====================================================================================================

#define _USE_MATH_DEFINES
#include "SnazzyPIDCalculator.h"
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include "stringhelper.h"

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
namespace frc { namespace robot { class SnazzyLog; } }
namespace frc { namespace robot { class MotionWayPoint; } }
namespace frc { namespace robot { class Robot; } }

namespace frc
{
	namespace robot
	{


		using edu::wpi::first::wpilibj::PIDOutput;
		using edu::wpi::first::wpilibj::PIDSource;
		using jaci::pathfinder::Trajectory;

		class SnazzyMotionPlanner : public SnazzyPIDCalculator
		{
				private:
					std::mutex lock_object;

			java::util::Timer *m_controlLoop;
			bool m_calibrating = false;
			bool m_calibrationStart = false;
			SnazzyLog *m_calLog;
			double m_calStart = 0;
			double m_lastCal = 0;
			double m_lastDist = 0;
			double m_lastV = 0;
			int m_count = 0;
			MotionWayPoint *m_currentWaypoint;
			bool m_motionPlanEnabled = false;
			bool m_motionTrajectoryEnabled = false;
			bool m_planFinished = false;
			bool m_dwell = false;
			double m_invertMultiplier = 1.0;
			double m_maxVelocity = 0;
			double m_maxAcceleration = 0;
			double m_timeUntilMaxVelocity = 0;
			double m_timeSpentCruising = 0;
			double m_positionAtMaxVelocity = 0;
			double m_positionAtEndOfCruise = 0;
			double m_timeAtEndOfCruise = 0;
			double m_initTime = 0;
			double m_initPos = 0;
			Trajectory *m_trajectory;
			std::vector<double> m_headingV;
			std::vector<double> m_headingA;
			double m_period = 0;

			double m_kA = 0;
			double m_kV = 0;
			double m_kAT = 0;
			double m_kVT = 0;

			double m_topCap = 0;
			double m_botCap = 0;
			double m_proErr = 0;
			bool m_protect = false;
			double m_ogMinimumOutput = 0;
			double m_ogMaximumOutput = 0;

			Robot *m_r;

		public:
			class MotionWayPoint
			{
				private:
					SnazzyMotionPlanner *outerInstance;

				public:
					virtual ~MotionWayPoint()
					{
						delete outerInstance;
					}

					MotionWayPoint(SnazzyMotionPlanner *outerInstance);

				double m_time = 0;
				double m_position = 0;
				double m_expectedVelocity = 0;
				double m_expectedAcceleration = 0;
				double m_expectedtA = 0;
				double m_expectedtV = 0;
				double m_heading = 0;
			};

		public:
			virtual ~SnazzyMotionPlanner()
			{
				delete m_controlLoop;
				delete m_calLog;
				delete m_currentWaypoint;
				delete m_trajectory;
				delete m_r;
			}

			SnazzyMotionPlanner(double Kp, double Ki, double Kd, double Kf, double kA, double kV, double kAT, double kVT, PIDSource *source, PIDOutput *output, double period, const std::wstring &fname, Robot *robot);

			virtual void setkAkV(double ka, double kv);

			virtual void setkATkVT(double kat, double kvt);



			virtual double getCurrentDistance();
			virtual void configureGoal(double goal, double max_v, double max_a, bool dwell);

			//overload configureGoal so that, if a dwell value is not given, it defaults to false
			virtual void configureGoal(double goal, double max_v, double max_a);

			virtual MotionWayPoint *getCurrentWaypoint(double t);
			void free() override;

			virtual void configureTrajectory(Trajectory *t, bool dwell);
			virtual void runCalibration();
			virtual void startCalibration();

			virtual void stopCalibration();

			virtual void runPlan();

			virtual void runTrajectory();


			virtual void computeHeadingStuff();
			virtual MotionWayPoint *getTrajectoryWaypoint(double t);

			virtual bool isPlanFinished();

		protected:
			double calculateFeedForward() override;

			double calculateTFeedForward() override;
			double getHeading() override;
			/*protected double getGyro() {
				if((m_motionPlanEnabled || m_motionTrajectoryEnabled) && m_currentWaypoint != null) {
					return m_r.gyro.getAngle();
				}
		
				return 0.0;
		
			}*/

		public:
			virtual double computeVelocityHeading(double currentH, double prevH, double t);
			virtual double computeAccelHeading(double currentHV, double prevHV, double t);

			virtual bool getCalibrate();

			virtual void setProtect(double bcap, double tcap, double err);

			virtual void protectThis();


//====================================================================================================
//End of the allowed output for the Free Edition of Java to C++ Converter.

//To subscribe to the Premium Edition, visit our website:
//https://www.tangiblesoftwaresolutions.com/order/order-java-to-cplus.html
//====================================================================================================
