// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-08-25. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

namespace sound
{
	void Init();
	void Shutdown();
	void Play(const char* name);
	void ClearQueue();
	void Stop();
}
