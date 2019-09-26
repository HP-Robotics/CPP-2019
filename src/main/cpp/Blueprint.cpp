#include "Blueprint.h"

namespace frc
{
	namespace robot
	{

		Blueprint::Blueprint(double timeout, std::function<int()> &start, std::function<int()> &periodic)
		{
			m_timeout = timeout;
			m_start = start;
			m_periodic = periodic;

		}
	}
}
