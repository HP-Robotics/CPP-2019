#include "SnazzyPIDController.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::wpilibj::PIDOutput;
		using edu::wpi::first::wpilibj::PIDSource;

		SnazzyPIDController::SnazzyPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource *source, PIDOutput *output, double period, const std::wstring &fname) : SnazzyPIDCalculator(Kp, Ki, Kd, Kf, source, output, period, fname)
		{
			 m_controlLoop = new java::util::Timer();
			 PIDTask tempVar(this, this);
			 m_controlLoop->schedule(&tempVar, 0LL, static_cast<long long>(period * 1000));

		}

		void SnazzyPIDController::free()
		{
			  m_controlLoop->cancel();
			  {
						std::scoped_lock<std::mutex> lock(lock_object);
				m_controlLoop = nullptr;
			  }
			SnazzyPIDCalculator::free();
		}

		SnazzyPIDController::PIDTask::PIDTask(SnazzyPIDController *outerInstance, SnazzyPIDController *snazzyPIDController) : outerInstance(outerInstance)
		{
		  if (snazzyPIDController == nullptr)
		  {
			throw NullPointerException(L"Given PIDController was null");
		  }
		  m_controller = snazzyPIDController;
		}

		void SnazzyPIDController::PIDTask::run()
		{
		  m_controller->calculate();

		  {
			  std::scoped_lock<std::mutex> lock(lock_object);
			  if (outerInstance->isEnabled())
			  {
				  outerInstance->m_pidOutput->pidWrite(outerInstance->m_result);
			  }
		  }
		}
	}
}
