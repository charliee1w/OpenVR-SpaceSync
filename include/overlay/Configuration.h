// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Calibration.h"

void LoadProfile(CalibrationContext &ctx);
// False means persistence failed; callers committing a new calibration must roll back.
bool SaveProfile(CalibrationContext &ctx);
