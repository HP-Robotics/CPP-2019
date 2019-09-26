#include "DrivePIDSourceInches.h"
#include "Robot.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::wpilibj::Encoder;
		using edu::wpi::first::wpilibj::PIDSource;
		using edu::wpi::first::wpilibj::PIDSourceType;

		DrivePIDSourceInches::DrivePIDSourceInches(Encoder *e)
		{
			this->e = e;
		}

		void DrivePIDSourceInches::setPIDSourceType(PIDSourceType *pidSource)
		{
			// TODO Auto-generated method stub

		}

		PIDSourceType *DrivePIDSourceInches::getPIDSourceType()
		{
			// TODO Auto-generated method stub
			return PIDSourceType::kDisplacement;
		}

		double DrivePIDSourceInches::pidGet()
		{
			return e->get() * Robot::DRIVE_ENC_TO_INCH;
		}
	}
}
