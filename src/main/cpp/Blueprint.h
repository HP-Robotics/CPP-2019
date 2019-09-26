#pragma once

#include <functional>

namespace frc
{
	namespace robot
	{


		class Blueprint
		{
		public:
			double m_timeout = 0;
			std::function<int()> m_start;
			std::function<int()> m_periodic;

			virtual ~Blueprint()
			{
				delete m_start;
				delete m_periodic;
			}

			Blueprint(double timeout, std::function<int()> &start, std::function<int()> &periodic);
		};

	}
}
