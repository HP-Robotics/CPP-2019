//====================================================================================================
//The Free Edition of Java to C++ Converter limits conversion output to 100 lines per file.

//To subscribe to the Premium Edition, visit our website:
//https://www.tangiblesoftwaresolutions.com/order/order-java-to-cplus.html
//====================================================================================================

#include "SnazzyMotionPlanner.h"
#include "SnazzyLog.h"
#include "Robot.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::wpilibj::PIDOutput;
		using edu::wpi::first::wpilibj::PIDSource;
		using edu::wpi::first::wpilibj::Timer;
		using jaci::pathfinder::Trajectory;

		SnazzyMotionPlanner::MotionWayPoint::MotionWayPoint(SnazzyMotionPlanner *outerInstance) : outerInstance(outerInstance)
		{
		}

		SnazzyMotionPlanner::SnazzyMotionPlanner(double Kp, double Ki, double Kd, double Kf, double kA, double kV, double kAT, double kVT, PIDSource *source, PIDOutput *output, double period, const std::wstring &fname, Robot *robot) : SnazzyPIDCalculator(Kp, Ki, Kd, Kf, source, output, period, fname)
		{
			m_controlLoop = new java::util::Timer();
			// m_controlLoop.schedule(new PIDTask(),0L, (long) (period * 1000));
			PIDTask tempVar(this);
			m_controlLoop->scheduleAtFixedRate(&tempVar, 0LL, static_cast<long long>(1000 * period));
			m_calLog = new SnazzyLog();
			m_kA = kA;
			m_kV = kV;
			m_kAT = kAT;
			m_kVT = kVT;
			m_period = period;
			m_r = robot;
		}

		void SnazzyMotionPlanner::setkAkV(double ka, double kv)
		{
			m_kA = ka;
			m_kV = kv;
		}

		void SnazzyMotionPlanner::setkATkVT(double kat, double kvt)
		{
			m_kAT = kat;
			m_kVT = kvt;
		}

		double SnazzyMotionPlanner::getCurrentDistance()
		{
			return m_pidInput->pidGet();
		}

		void SnazzyMotionPlanner::configureGoal(double goal, double max_v, double max_a, bool dwell)
		{
			m_motionPlanEnabled = true;
			m_motionTrajectoryEnabled = false;

			m_planFinished = false;
			m_dwell = dwell;
			m_initTime = Timer::getFPGATimestamp();
			m_initPos = m_pidInput->pidGet();


			//check if goal is negative
			if (goal < 0)
			{
				m_invertMultiplier = -1.0;
				goal = std::abs(goal);
			}
			else
			{
				m_invertMultiplier = 1.0;
			}

			double midpoint = goal / 2;

			m_maxAcceleration = max_a;

			double t_until_midpoint = std::sqrt((midpoint * 2) / max_a);

			/* New formula:  v = at */

			double v_needed_to_get_to_midpoint = t_until_midpoint * max_a;

			/* Simple case:  we never hit max velocity */
			if (v_needed_to_get_to_midpoint <= max_v)
			{
				m_maxVelocity = v_needed_to_get_to_midpoint;
				m_timeUntilMaxVelocity = t_until_midpoint;
				m_timeSpentCruising = 0.0;
			}
			else
			{
				/* Complex case:  we accelerate up to max v, cruise for a while, and then decelerate */
				m_maxVelocity = max_v;

				/* v = at , so t = v/a */
				m_timeUntilMaxVelocity = max_v / max_a;

				/* d = 1/2 at^2 */
				double distance_while_accelerating = 0.5 * max_a * (m_timeUntilMaxVelocity * m_timeUntilMaxVelocity);

				double distance_while_cruising = goal - (2 * distance_while_accelerating);

				m_timeSpentCruising = distance_while_cruising / max_v;
			}
			m_positionAtMaxVelocity = 0.5 * m_maxAcceleration * (m_timeUntilMaxVelocity * m_timeUntilMaxVelocity);
			m_positionAtEndOfCruise = m_positionAtMaxVelocity + (m_timeSpentCruising * m_maxVelocity);
			m_timeAtEndOfCruise = m_timeUntilMaxVelocity + m_timeSpentCruising;


		}

		void SnazzyMotionPlanner::configureGoal(double goal, double max_v, double max_a)
		{
			configureGoal(goal, max_v, max_a, false);
		}

		SnazzyMotionPlanner::MotionWayPoint *SnazzyMotionPlanner::getCurrentWaypoint(double t)
		{

			if (t > (2 * m_timeUntilMaxVelocity) + m_timeSpentCruising)
			{
				return nullptr;
			}

			MotionWayPoint *p = new MotionWayPoint(this);
			p->m_time = t;
			if (t < m_timeUntilMaxVelocity)
			{
				/* We are still ramping up.  */
				p->m_expectedVelocity = t * m_maxAcceleration;
				p->m_expectedAcceleration = m_maxAcceleration;
				p->m_position = 0.5 * m_maxAcceleration * t * t;
			}
			else
			{
				/* We are either cruising, or ramping down */
				if (t < m_timeAtEndOfCruise)
				{
					/* We are cruising */
					p->m_expectedVelocity = m_maxVelocity;
					p->m_expectedAcceleration = 0;
					p->m_position = m_positionAtMaxVelocity + (t - m_timeUntilMaxVelocity) * m_maxVelocity;
				}
				else
				{
					/* We are ramping down */
					double t_decel = (t - m_timeAtEndOfCruise);
					p->m_expectedAcceleration = -1.0 * m_maxAcceleration;
					p->m_expectedVelocity = m_maxVelocity - (t_decel * m_maxAcceleration);

					/* d = d0 + v0*t + 1/2 a t^2   */
					p->m_position = m_positionAtEndOfCruise + (m_maxVelocity * t_decel) + (0.5 * -1.0 * m_maxAcceleration * (t_decel * t_decel));
				}
			}

//JAVA TO C++ CONVERTER TODO TASK: A 'delete p' statement was not added since p was used in a 'return' or 'throw' statement.
			return p;
		}

		void SnazzyMotionPlanner::free()
		{
			m_controlLoop->cancel();
			{
						std::scoped_lock<std::mutex> lock(lock_object);
				m_controlLoop = nullptr;
			}
			SnazzyPIDCalculator::free();
		}

		void SnazzyMotionPlanner::configureTrajectory(Trajectory *t, bool dwell)
		{
			m_motionTrajectoryEnabled = true;
			m_motionPlanEnabled = false;
			m_planFinished = false;
			m_dwell = dwell;
			m_initTime = Timer::getFPGATimestamp();
			m_initPos = m_pidInput->pidGet();

			m_trajectory = t;
			computeHeadingStuff();
		}

		void SnazzyMotionPlanner::runCalibration()
		{
			double currentDist;
			double currentCal;
			double currentV;
			{
						std::scoped_lock<std::mutex> lock(lock_object);
				m_pidOutput->pidWrite(1.0);
				currentCal = Timer::getFPGATimestamp();
				currentDist = m_pidInput->pidGet();
			}

			if (m_calibrationStart)
			{

//====================================================================================================
//End of the allowed output for the Free Edition of Java to C++ Converter.

//To subscribe to the Premium Edition, visit our website:
//https://www.tangiblesoftwaresolutions.com/order/order-java-to-cplus.html
//====================================================================================================
