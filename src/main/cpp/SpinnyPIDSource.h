#pragma once

namespace frc
{
	namespace robot
	{

		using edu::wpi::first::wpilibj::Encoder;
		using edu::wpi::first::wpilibj::PIDSource;
		using edu::wpi::first::wpilibj::PIDSourceType;

		class SpinnyPIDSource : public PIDSource
		{

		public:
			Encoder *e1;
			Encoder *e2;

			virtual ~SpinnyPIDSource()
			{
				delete e1;
				delete e2;
			}

			SpinnyPIDSource(Encoder *enc1, Encoder *enc2);

			void setPIDSourceType(PIDSourceType *pidSource) override;

			PIDSourceType *getPIDSourceType() override;

			double pidGet() override;


		};
	}
}
