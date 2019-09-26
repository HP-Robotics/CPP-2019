#pragma once

#include <string>
#include <vector>
#include <iostream>
#include "stringhelper.h"

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
namespace frc { namespace robot { class SnazzyLog; } }

namespace frc
{
	namespace robot
	{


		using jaci::pathfinder::Trajectory;

		class TrajectoryPlanner
		{
		public:
			SnazzyLog *log = new SnazzyLog();
		private:
			Trajectory *m_left;
			Trajectory *m_right;
		public:
			File *rightFile;
			File *leftFile;
			std::vector<std::vector<double>> arrayPoints;
		private:
			Trajectory *m_trajectory;
			double m_maxA = 0;
			double m_maxV = 0;
			double m_maxJ = 0;
			double wheelbase = 21.75 + 1.6 + .32;
			//private double wheelbase =  25.125 + 3.05;// FRANK
			std::wstring m_name;

		public:
			virtual ~TrajectoryPlanner()
			{
				delete log;
				delete m_left;
				delete m_right;
				delete rightFile;
				delete leftFile;
				delete m_trajectory;
			}

			TrajectoryPlanner(std::vector<std::vector<double>> &ap, double max_v, double max_a, double max_j, const std::wstring &name);

			TrajectoryPlanner(Trajectory *t, const std::wstring &name);

			virtual void generate();

			virtual void regenerate();

			virtual Trajectory *getLeftTrajectory();

			virtual Trajectory *getInvertTrajectory(Trajectory *t);

			virtual Trajectory *getInvertedLeftTrajectory();

			virtual Trajectory *getInvertedRightTrajectory();

			virtual Trajectory *getRightTrajectory();

			virtual std::wstring getFileName();

		   virtual void frankenstein(TrajectoryPlanner *traj2, double v);

		   virtual void rename(const std::wstring &new_name);

		   virtual void flattenkAkV();
		};
	}
}
