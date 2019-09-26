#pragma once

namespace frc
{
	namespace robot
	{

		using edu::wpi::first::wpilibj::Encoder;
		using edu::wpi::first::wpilibj::PIDSource;
		using edu::wpi::first::wpilibj::PIDSourceType;

		class DrivePIDSourceInches : public PIDSource
		{
		public:
			Encoder *e;

			virtual ~DrivePIDSourceInches()
			{
				delete e;
			}

			DrivePIDSourceInches(Encoder *e);

			void setPIDSourceType(PIDSourceType *pidSource) override;

			PIDSourceType *getPIDSourceType() override;

			double pidGet() override;
		};
	}
}
