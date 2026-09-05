// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Protocol.h"

#include <thread>
#include <mutex>
#include <memory>
#include <string>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

class ServerTrackedDeviceProvider;

class IPCServer
{
public:
	explicit IPCServer(ServerTrackedDeviceProvider *driver, std::string pipeName = SPACESYNC_PIPE_NAME)
		: driver(driver), pipeName(std::move(pipeName)) { }
	~IPCServer();

	bool Run();
	void Stop();

private:
	void HandleRequest(const protocol::Request &request, protocol::Response &response);

	struct PipeInstance;
	std::unique_ptr<PipeInstance> CreatePipeInstance(bool first);
	void RunThread(std::unique_ptr<PipeInstance> initial);
	bool Advance(PipeInstance& instance);
	// Run/Stop serialize ownership; only RunThread owns pipe I/O and buffers.
	std::mutex lifecycleMutex;
	std::thread mainThread;
	HANDLE stopEvent = nullptr;
	ServerTrackedDeviceProvider *driver;
	std::string pipeName;
};
