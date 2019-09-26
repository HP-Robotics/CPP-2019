//====================================================================================================
//The Free Edition of Java to C++ Converter limits conversion output to 100 lines per file.

//To subscribe to the Premium Edition, visit our website:
//https://www.tangiblesoftwaresolutions.com/order/order-java-to-cplus.html
//====================================================================================================

#include "TrajectoryPlanner.h"
#include "SnazzyLog.h"

namespace frc
{
	namespace robot
	{
		using jaci::pathfinder::Pathfinder;
		using jaci::pathfinder::Trajectory;
		using jaci::pathfinder::Waypoint;
		using jaci::pathfinder::modifiers::TankModifier;

		TrajectoryPlanner::TrajectoryPlanner(std::vector<std::vector<double>> &ap, double max_v, double max_a, double max_j, const std::wstring &name)
		{
			arrayPoints = ap;
			m_maxA = max_a;
			m_maxV = max_v;
			m_maxJ = max_j;
			m_name = name;

		}

		TrajectoryPlanner::TrajectoryPlanner(Trajectory *t, const std::wstring &name)
		{
			m_trajectory = t;
			m_name = name;
		}

		void TrajectoryPlanner::generate()
		{
			Trajectory::Config *config = new Trajectory::Config(Trajectory::FitMethod::HERMITE_CUBIC, Trajectory::Config::SAMPLES_HIGH, 0.015, m_maxV, m_maxA, m_maxJ);
			std::vector<Waypoint*> points(arrayPoints.size());

			for (int i = 0; i < arrayPoints.size();i++)
			{
				points[i] = new Waypoint(arrayPoints[i][0], arrayPoints[i][1],Pathfinder::d2r(arrayPoints[i][2]));
			}
			File *myFile = new File(getFileName());

			if (myFile->exists())
			{
			m_trajectory = Pathfinder::readFromCSV(myFile);
			}
			else
			{
				m_trajectory = Pathfinder::generate(points, config);
				Pathfinder::writeToCSV(myFile, m_trajectory);
			}
			std::wcout << m_trajectory->length() << std::endl;

			regenerate();

//JAVA TO C++ CONVERTER TODO TASK: A 'delete myFile' statement was not added since myFile was passed to a method or constructor. Handle memory management manually.
//JAVA TO C++ CONVERTER TODO TASK: A 'delete config' statement was not added since config was passed to a method or constructor. Handle memory management manually.
		}

		void TrajectoryPlanner::regenerate()
		{
			TankModifier tempVar(m_trajectory);
			TankModifier *modifier = (&tempVar)->modify(wheelbase);
			// Do something with the new Trajectories...
			m_left = modifier->getLeftTrajectory();
			m_right = modifier->getRightTrajectory();

			for (int i = 0; i < m_trajectory->length(); i++)
			{
				Trajectory::Segment *seg = m_trajectory->get(i);

				//log.open(m_name+"Trajectory.csv","Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
				log->open(StringHelper::formatSimple(L"%sTrajectory.csv", m_name),L"Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
				//log.write(seg.dt + "," + seg.x + "," + seg.y + "," + seg.position + "," + seg.velocity + "," + 
						//seg.acceleration + "," + seg.jerk + "," + seg.heading + "\n");
				log->write(std::wstring::format(L"%g,%g,%g,%g,%g,%g,%g,%g\n", seg->dt, seg->x, seg->y, seg->position, seg->velocity, seg->acceleration, seg->jerk, seg->heading));
			}
			log->close();
			for (int i = 0; i < m_trajectory->length(); i++)
			{
				Trajectory::Segment *seg = m_left->get(i);

				log->open(StringHelper::formatSimple(L"%sLeftTrajectory.csv", m_name),L"Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
				log->write(std::wstring::format(L"%g,%g,%g,%g,%g,%g,%g,%g\n", seg->dt, seg->x, seg->y, seg->position, seg->velocity, seg->acceleration, seg->jerk, seg->heading));
			}
			log->close();
			for (int i = 0; i < m_trajectory->length(); i++)
			{
				Trajectory::Segment *seg = m_right->get(i);

				log->open(StringHelper::formatSimple(L"%sRightTrajectory.csv", m_name),L"Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
				log->write(std::wstring::format(L"%g,%g,%g,%g,%g,%g,%g,%g\n", seg->dt, seg->x, seg->y, seg->position, seg->velocity, seg->acceleration, seg->jerk, seg->heading));
			}
			log->close();
		}

		Trajectory *TrajectoryPlanner::getLeftTrajectory()
		{
			return m_left;
		}

		Trajectory *TrajectoryPlanner::getInvertTrajectory(Trajectory *t)
		{
			Trajectory *inverted = new Trajectory(t->length());
			for (int i = 0; i < t->length();i++)
			{
				inverted->segments[i] = t->segments[i].copy();
				inverted->segments[i]->x = t->segments[i].x * -1;
				inverted->segments[i]->y = t->segments[i].y * -1;
				inverted->segments[i]->acceleration = t->segments[i].acceleration * -1;
				inverted->segments[i]->jerk = t->segments[i].jerk * -1;
				inverted->segments[i]->position = t->segments[i].position * -1;
				inverted->segments[i]->velocity = t->segments[i].velocity * -1;
				//TODO invert heading?
			}

//JAVA TO C++ CONVERTER TODO TASK: A 'delete inverted' statement was not added since inverted was used in a 'return' or 'throw' statement.
			return inverted;
		}

		Trajectory *TrajectoryPlanner::getInvertedLeftTrajectory()
		{
			return getInvertTrajectory(m_right);
		}

		Trajectory *TrajectoryPlanner::getInvertedRightTrajectory()
		{
			return getInvertTrajectory(m_left);
		}

		Trajectory *TrajectoryPlanner::getRightTrajectory()
		{
			return m_right;
		}

		std::wstring TrajectoryPlanner::getFileName()
		{
			//String name = new String("/home/lvuser/"+m_name+"Trajectory.");
			std::wstring name = StringHelper::formatSimple(L"/home/lvuser/%sTrajectory.", m_name);
			for (int i = 0;i < arrayPoints.size();i++)
			{
				//name += Double.toString(arrayPoints[i][0])+"."+Double.toString(arrayPoints[i][1])+"."+Double.toString(arrayPoints[i][2]);
				name = std::wstring::format(L"%s%g.%g.%g", name, arrayPoints[i][0], arrayPoints[i][1], arrayPoints[i][2]);
			}
			//name += Double.toString(m_maxV)+Double.toString(m_maxA)+Double.toString(m_maxJ)+".csv";
			name = std::wstring::format(L"%s%g%g%g.csv", name, m_maxV, m_maxA, m_maxJ);
			return name;
		}

		void TrajectoryPlanner::frankenstein(TrajectoryPlanner *traj2, double v)
		{
			 Trajectory *newTraj;
			 rename(L"Frankenstein" + m_name);
			 int chopOne = 0;
			 int chopTwo = traj2->m_trajectory->segments->length;

			 for (int i = m_trajectory->segments->length - 1; i >= 0 && m_trajectory->segments[i].velocity < v ; i--)
			 {
				 chopOne = i;

			 }
			 for (int i = 0;i < traj2->m_trajectory->segments->length && traj2->m_trajectory->segments[i].velocity < v; i++)
			 {
				 chopTwo = i;

			 }

			 newTraj = new Trajectory(chopOne + (traj2->m_trajectory->segments->length - chopTwo));
			 double maxv1 = 0;
			 for (int i = 0; i < chopOne ; i++)
			 {
				 newTraj->segments[i] = m_trajectory->segments[i].copy();
				 if (m_trajectory->segments[i].velocity > maxv1)
				 {
					 maxv1 = m_trajectory->segments[i].velocity;
				 }
			 }
			 double maxv2 = 0;
			 for (int i = chopTwo;i < traj2->m_trajectory->segments->length; i++)
			 {

				 newTraj->segments[chopOne + i - chopTwo] = traj2->m_trajectory->segments[i].copy();

				 newTraj->segments[chopOne + i - chopTwo].position += m_trajectory->segments[chopOne].position - traj2->m_trajectory->segments[chopTwo].position;

				 if (traj2->m_trajectory->segments[i].velocity > maxv2)
				 {
					 maxv2 = traj2->m_trajectory->segments[i].velocity;
				 }

			 }

			 std::wcout << L"chopOne " << chopOne << L" " << m_trajectory->segments[chopOne].position << L"; chopTwo " << chopTwo << L" " << traj2->m_trajectory->segments[chopTwo].position << std::endl;
			 std::wcout << L"Max V1: " << maxv1 << L"Max V2: " << maxv2 << std::endl;
			 std::wcout << getFileName() << std::endl;
			 std::wcout << traj2->getFileName() << std::endl;
			 m_trajectory = newTraj;

			 regenerate();

//====================================================================================================
//End of the allowed output for the Free Edition of Java to C++ Converter.

//To subscribe to the Premium Edition, visit our website:
//https://www.tangiblesoftwaresolutions.com/order/order-java-to-cplus.html
//====================================================================================================
