#include "LimelightAnglePIDSource.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::networktables::NetworkTableEntry;
		using edu::wpi::first::networktables::NetworkTableInstance;
		using edu::wpi::first::wpilibj::Encoder;
		using edu::wpi::first::wpilibj::PIDSource;
		using edu::wpi::first::wpilibj::PIDSourceType;

		void LimelightAnglePIDSource::setPIDSourceType(PIDSourceType *pidSource)
		{
			// TODO Auto-generated method stub

		}

		PIDSourceType *LimelightAnglePIDSource::getPIDSourceType()
		{
			// TODO Auto-generated method stub
			return PIDSourceType::kDisplacement;
		}

		double LimelightAnglePIDSource::pidGet()
		{
			return txEntry->getDouble(0.0);
		}

		bool LimelightAnglePIDSource::isValid()
		{
			/*System.out.println("tx: " + txEntry.getDouble(0.0) + " tv: " + tvEntry.getDouble(0.0) + " tz: " + tzEntry.getDouble(0.0));*/
			return tvEntry->getDouble(0.0) == 1.0; //&& Math.abs(txEntry.getDouble(0.0))< 40.0 && tzEntry.getDouble(0.0) < 120.0;
		}
	}
}
