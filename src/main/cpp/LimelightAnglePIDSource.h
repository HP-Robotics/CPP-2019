#pragma once

namespace frc
{
	namespace robot
	{

		using edu::wpi::first::networktables::NetworkTableEntry;
		using edu::wpi::first::networktables::NetworkTableInstance;
		using edu::wpi::first::wpilibj::PIDSource;
		using edu::wpi::first::wpilibj::PIDSourceType;

		class LimelightAnglePIDSource : public PIDSource
		{


		private:
			NetworkTableEntry *txEntry = NetworkTableInstance::getDefault().getTable(L"limelight").getEntry(L"tx");
			/*private NetworkTableEntry tzEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tz");*/
			NetworkTableEntry *tvEntry = NetworkTableInstance::getDefault().getTable(L"limelight").getEntry(L"tv");

		public:
			virtual ~LimelightAnglePIDSource()
			{
				delete txEntry;
				delete tvEntry;
			}

			void setPIDSourceType(PIDSourceType *pidSource) override;

			PIDSourceType *getPIDSourceType() override;

			double pidGet() override;

			virtual bool isValid();
		};
	}
}
