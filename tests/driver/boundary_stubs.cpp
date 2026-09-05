// SPDX-License-Identifier: AGPL-3.0-only
// Only replace process-wide hook installation and disk logging. Tests execute
// the production provider and IPC object, without calling Init or opening a pipe.
#include "Logging.h"
#include "InterfaceHookInjector.h"

FILE* LogFile = nullptr;
void OpenLogFile() { }
void CloseLogFile() { }
tm TimeForLog() { return {}; }
void LogFlush() { }
bool InjectHooks(vr::IVRDriverContext*) { return true; }
void DisableHooks() { }
