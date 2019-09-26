//====================================================================================================
//The Free Edition of Java to C++ Converter limits conversion output to 100 lines per file.

//To subscribe to the Premium Edition, visit our website:
//https://www.tangiblesoftwaresolutions.com/order/order-java-to-cplus.html
//====================================================================================================

#include "SnazzyPIDCalculator.h"
#include "SnazzyLog.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::hal::util::BoundaryException;
		using edu::wpi::first::wpilibj::PIDInterface;
		using edu::wpi::first::wpilibj::PIDOutput;
		using edu::wpi::first::wpilibj::PIDSource;
		using edu::wpi::first::wpilibj::PIDSourceType;
		using edu::wpi::first::wpilibj::Timer;
		using edu::wpi::first::wpilibj::livewindow::LiveWindowSendable;
		using edu::wpi::first::wpilibj::tables::ITable;
		using edu::wpi::first::wpilibj::tables::ITableListener;
		using edu::wpi::first::networktables::NetworkTable;
int SnazzyPIDCalculator::instances = 0;

		SnazzyPIDCalculator::NullTolerance::NullTolerance(SnazzyPIDCalculator *outerInstance) : outerInstance(outerInstance)
		{
		}

		bool SnazzyPIDCalculator::NullTolerance::onTarget()
		{
		  throw std::runtime_error("No tolerance value set when calling onTarget().");
		}

		SnazzyPIDCalculator::PercentageTolerance::PercentageTolerance(SnazzyPIDCalculator *outerInstance, double value) : m_percentage(value), outerInstance(outerInstance)
		{
		}

		bool SnazzyPIDCalculator::PercentageTolerance::onTarget()
		{
		  return outerInstance->isAvgErrorValid() && (std::abs(outerInstance->getAvgError()) < m_percentage / 100 * (outerInstance->m_maximumInput - outerInstance->m_minimumInput));
		}

		SnazzyPIDCalculator::AbsoluteTolerance::AbsoluteTolerance(SnazzyPIDCalculator *outerInstance, double value) : m_value(value), outerInstance(outerInstance)
		{
		}

		bool SnazzyPIDCalculator::AbsoluteTolerance::onTarget()
		{
		  return outerInstance->isAvgErrorValid() && std::abs(outerInstance->getAvgError()) < m_value;
		}

//JAVA TO C++ CONVERTER TODO TASK: Most Java annotations will not have direct C++ equivalents:
//ORIGINAL LINE: @SuppressWarnings("ParameterName") public SnazzyPIDCalculator(double Kp, double Ki, double Kd, double Kf, edu.wpi.first.wpilibj.PIDSource source, edu.wpi.first.wpilibj.PIDOutput output, double period, String fname)
		SnazzyPIDCalculator::SnazzyPIDCalculator(double Kp, double Ki, double Kd, double Kf, PIDSource *source, PIDOutput *output, double period, const std::wstring &fname)
		{

		  if (source == nullptr)
		  {
			throw NullPointerException(L"Null PIDSource was given");
		  }
		  if (output == nullptr)
		  {
			throw NullPointerException(L"Null PIDOutput was given");
		  }


		  m_setpointTimer = new Timer();
		  m_setpointTimer->start();

		  m_P = Kp;
		  m_I = Ki;
		  m_D = Kd;
		  m_F = Kf;

		  m_file = fname;

		  m_pidInput = source;
		  m_pidOutput = output;
		  m_period = period;



		  instances++;
		  //HLUsageReporting.reportPIDController(instances);
		  m_tolerance = new NullTolerance(this);

		  m_buf = new ArrayDeque<double>(m_bufLength + 1);
		}

//JAVA TO C++ CONVERTER TODO TASK: Most Java annotations will not have direct C++ equivalents:
//ORIGINAL LINE: @SuppressWarnings("ParameterName") public SnazzyPIDCalculator(double Kp, double Ki, double Kd, edu.wpi.first.wpilibj.PIDSource source, edu.wpi.first.wpilibj.PIDOutput output, double period)
		SnazzyPIDCalculator::SnazzyPIDCalculator(double Kp, double Ki, double Kd, PIDSource *source, PIDOutput *output, double period) : SnazzyPIDCalculator(Kp, Ki, Kd, 0.0, source, output, period, L"")
		{
		}

//JAVA TO C++ CONVERTER TODO TASK: Most Java annotations will not have direct C++ equivalents:
//ORIGINAL LINE: @SuppressWarnings("ParameterName") public SnazzyPIDCalculator(double Kp, double Ki, double Kd, edu.wpi.first.wpilibj.PIDSource source, edu.wpi.first.wpilibj.PIDOutput output)
		SnazzyPIDCalculator::SnazzyPIDCalculator(double Kp, double Ki, double Kd, PIDSource *source, PIDOutput *output) : SnazzyPIDCalculator(Kp, Ki, Kd, source, output, kDefaultPeriod)
		{
		}

//JAVA TO C++ CONVERTER TODO TASK: Most Java annotations will not have direct C++ equivalents:
//ORIGINAL LINE: @SuppressWarnings("ParameterName") public SnazzyPIDCalculator(double Kp, double Ki, double Kd, double Kf, edu.wpi.first.wpilibj.PIDSource source, edu.wpi.first.wpilibj.PIDOutput output)
		SnazzyPIDCalculator::SnazzyPIDCalculator(double Kp, double Ki, double Kd, double Kf, PIDSource *source, PIDOutput *output) : SnazzyPIDCalculator(Kp, Ki, Kd, Kf, source, output, kDefaultPeriod, L"")
		{
		}

		void SnazzyPIDCalculator::free()
		{
		  {
						std::scoped_lock<std::mutex> lock(lock_object);
			m_freed = true;
			m_pidOutput = nullptr;
			m_pidInput = nullptr;
		  }
		  if (m_table != nullptr)
		  {
			m_table->removeTableListener(m_listener);
		  }
		}

		void SnazzyPIDCalculator::calculate()
		{
		  bool enabled;
		  PIDSource *pidInput;
		  double pterm = 0;
		  double iterm = 0;
		  double dterm = 0;
		  double fterm = 0;
		  double tfterm = 0;

		  /* Gremlin hunting code */
		  double now;
		  now = Timer::getFPGATimestamp();
		  if (lastCalcTime != 0.0)
		  {
			if (now - lastCalcTime > m_period * 3)
			{
			  //System.out.println("Gremlin " + m_file + " " + (now-lastCalcTime) + "s, now " + now + "s, lastCalcTime " + lastCalcTime + "s");
			  printf(L"Gremlin %s %gs, now %gs, lastCalcTime %gs\n", m_file, (now - lastCalcTime), now, lastCalcTime);
			}
		  }
		  lastCalcTime = now;

		  {
						std::scoped_lock<std::mutex> lock(lock_object);
			if (m_pidInput == nullptr)
			{
			  return;
			}
			if (m_pidOutput == nullptr)
			{
			  return;
			}
			enabled = m_enabled; // take snapshot of these values...
			pidInput = m_pidInput;
		  }

		  if (enabled)
		  {
			double input;
			double timestamp;
			double result;
//JAVA TO C++ CONVERTER WARNING: The original Java variable was marked 'final':
//ORIGINAL LINE: final edu.wpi.first.wpilibj.PIDOutput pidOutput;
			PIDOutput *pidOutput;
			{
						  std::scoped_lock<std::mutex> lock(lock_object);
			  input = pidInput->pidGet();
			  timestamp = Timer::getFPGATimestamp();
			}
			{
						  std::scoped_lock<std::mutex> lock(lock_object);
			  m_error = getContinuousError(m_setpoint - input);

			  if (m_pidInput->getPIDSourceType().equals(PIDSourceType::kRate))
			  {
				if (m_P != 0)
				{
				  double potentialPGain = (m_totalError + m_error) * m_P;
				  if (potentialPGain < m_maximumOutput)
				  {
					if (potentialPGain > m_minimumOutput)
					{
					  m_totalError += m_error;
					}
					else
					{
					  m_totalError = m_minimumOutput / m_P;
					}
				  }
				  else
				  {
					m_totalError = m_maximumOutput / m_P;
				  }
				  pterm = m_P * m_error;
				  dterm = m_D * (m_error - m_prevError);
				  fterm = calculateFeedForward();
				  m_result = pterm + dterm + fterm;
				}
			  }
			  else
			  {
				if (m_I != 0)
				{
				  double potentialIGain = (m_totalError + m_error) * m_I;
				  if (potentialIGain < m_maximumOutput)
				  {
					if (potentialIGain > m_minimumOutput)
					{

//====================================================================================================
//End of the allowed output for the Free Edition of Java to C++ Converter.

//To subscribe to the Premium Edition, visit our website:
//https://www.tangiblesoftwaresolutions.com/order/order-java-to-cplus.html
//====================================================================================================
