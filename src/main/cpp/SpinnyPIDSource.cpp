#include "SpinnyPIDSource.h"
#include "Robot.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::wpilibj::Encoder;
		using edu::wpi::first::wpilibj::PIDSource;
		using edu::wpi::first::wpilibj::PIDSourceType;

		SpinnyPIDSource::SpinnyPIDSource(Encoder *enc1, Encoder *enc2)
		{
			e1 = enc1;
			e2 = enc2;
		}

		void SpinnyPIDSource::setPIDSourceType(PIDSourceType *pidSource)
		{
			// TODO Auto-generated method stub

		}

		PIDSourceType *SpinnyPIDSource::getPIDSourceType()
		{
			// TODO Auto-generated method stub
			return PIDSourceType::kDisplacement;
		}

		double SpinnyPIDSource::pidGet()
		{
			return (e1->pidGet() - e2->pidGet()) * Robot::DRIVE_ENC_TO_INCH;
		}
	}
}
