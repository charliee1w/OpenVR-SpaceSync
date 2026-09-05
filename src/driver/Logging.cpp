// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"
#include <chrono>

FILE *LogFile = stderr;

void OpenLogFile()
{
	CloseLogFile();
	LogFile = fopen("spacesync_driver.log", "a");
	if (LogFile == nullptr)
	{
		LogFile = stderr;
	}
}

void CloseLogFile()
{
	// Producers are drained by Cleanup before closing. The fallback stream is
	// owned by the host, and a failed diagnostic flush must not exit vrserver.
	FILE* owned = LogFile;
	LogFile = stderr;
	if (owned && owned != stderr)
		(void)fclose(owned);
}

tm TimeForLog()
{
	auto now = std::chrono::system_clock::now();
	auto nowTime = std::chrono::system_clock::to_time_t(now);
	tm value;
	auto tm = localtime_s(&value, &nowTime);
	return value;
}

void LogFlush()
{
	fflush(LogFile);
}
