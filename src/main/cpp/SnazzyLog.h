#pragma once

#include <string>
#include <iostream>
#include "exceptionhelper.h"
#include "stringhelper.h"

namespace frc
{
	namespace robot
	{



		class SnazzyLog
		{
		public:
			File *m_f;
			FileWriter *m_fw;
			BufferedWriter *m_bw;
			std::wstring m_file;
			std::wstring m_header;
			bool m_open = false;

			virtual bool open(const std::wstring &file, const std::wstring &header);

			virtual ~SnazzyLog();

			virtual bool write(const std::wstring &s);
			virtual void reset();
		};

	}
}
