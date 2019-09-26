#include "SnazzyLog.h"

namespace frc
{
	namespace robot
	{
		using edu::wpi::first::wpilibj::Timer;

		bool SnazzyLog::open(const std::wstring &file, const std::wstring &header)
		{
			if (file == L"")
			{
				return false;
			}
			m_header = header;
			if (m_open)
			{
				return true;
			}
			m_file = file;
			try
			{
				if (Files::exists(Paths->get(L"/home/lvuser")))
				{
					m_f = new File(StringHelper::formatSimple(L"/home/lvuser/%s", file));
				}
				else
				{
					m_f = new File(StringHelper::formatSimple(L"/tmp/%s",file));
				}
				if (!m_f->exists())
				{
					m_f->createNewFile();
				}
				m_fw = new FileWriter(m_f);

			}
			catch (const IOException &e)
			{
				e->printStackTrace();
				return false;
			}
			m_bw = new BufferedWriter(m_fw);
			m_open = true;

			if (m_header != L"")
			{
				return write(header);
			}
			return true;
		}

		SnazzyLog::~SnazzyLog()
		{
			if (m_open)
			{
				try
				{
					m_bw->close();
					m_fw->close();

				}
				catch (const IOException &e)
				{

				}
				m_open = false;
			}
		}

		bool SnazzyLog::write(const std::wstring &s)
		{
			if (!m_open)
			{
				return false;
			}
			try
			{
				m_bw->write(s);
				m_bw->flush();

			}
			catch (const IOException &e)
			{
				std::wcout << L"Unexpected exception in " << m_file << std::endl;
				e->printStackTrace();
				return false;
			}
			return true;
		}

		void SnazzyLog::reset()
		{
			close();
			open(m_file, m_header);
		}
	}
}
