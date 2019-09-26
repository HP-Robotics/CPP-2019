#pragma once

#include "SnazzyPIDCalculator.h"
#include <string>
#include <mutex>
#include "exceptionhelper.h"

namespace frc
{
	namespace robot
	{


		using edu::wpi::first::wpilibj::PIDOutput;
		using edu::wpi::first::wpilibj::PIDSource;

		class SnazzyPIDController : public SnazzyPIDCalculator
		{
				private:
					std::mutex lock_object;

			java::util::Timer *m_controlLoop;

		public:
			virtual ~SnazzyPIDController()
			{
				delete m_controlLoop;
			}

			SnazzyPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource *source, PIDOutput *output, double period, const std::wstring &fname);
			  void free() override;

		 private:
			 class PIDTask : public TimerTask
			 {
					 private:
						 std::mutex lock_object;

					 SnazzyPIDController *outerInstance;


					SnazzyPIDController *m_controller;

				public:
					virtual ~PIDTask()
					{
						delete m_controller;
						delete outerInstance;
					}

					PIDTask(SnazzyPIDController *outerInstance, SnazzyPIDController *snazzyPIDController);

					void run() override;
			 };
		};

	}
}
