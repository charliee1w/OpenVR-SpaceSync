// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "IPCServer.h"
#include "Logging.h"
#include "ServerTrackedDeviceProvider.h"

void IPCServer::HandleRequest(const protocol::Request &request, protocol::Response &response)
{
	switch (request.type)
	{
	case protocol::RequestHandshake:
		response.type = protocol::ResponseHandshake;
		response.protocol.version = protocol::Version;
		break;

	case protocol::RequestSetDeviceTransform:
		driver->SetDeviceTransform(request.setDeviceTransform);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestSetHmdTracker:
		driver->SetHmdTracker(request.setHmdTracker);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestSetSlamSync:
		driver->SetSlamSync(request.setSlamSync);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestSetOneEuro:
		driver->SetOneEuro(request.setOneEuro);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestGetStatus:
		driver->GetStatus(response.status);
		response.type = protocol::ResponseStatus;
		break;

	default:
		LOG("Invalid IPC request: %d", request.type);
		break;
	}
}

#include <vector>
#include <cstring>

struct IPCServer::PipeInstance
{
    enum class Operation { Connect, Read, Write } operation = Operation::Connect;
    HANDLE pipe = INVALID_HANDLE_VALUE;
    OVERLAPPED overlap{};
    bool pending = false;
    bool connected = false;
    DWORD completedBytes = 0;
    protocol::Request request;
    protocol::Response response;

    ~PipeInstance()
    {
        // The kernel must release the OVERLAPPED and buffer before destruction.
        if (pipe != INVALID_HANDLE_VALUE)
        {
            if (pending)
            {
                CancelIoEx(pipe, &overlap);
                DWORD bytes = 0;
                GetOverlappedResult(pipe, &overlap, &bytes, TRUE);
            }
            if (connected) DisconnectNamedPipe(pipe);
            CloseHandle(pipe);
        }
        if (overlap.hEvent) CloseHandle(overlap.hEvent);
    }
};

IPCServer::~IPCServer() { Stop(); }

bool IPCServer::Run()
{
    std::lock_guard<std::mutex> lock(lifecycleMutex);
    if (mainThread.joinable()) return true;
    stopEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    if (!stopEvent) return false;
    try
    {
        auto initial = CreatePipeInstance(true);
        if (!initial)
        {
            CloseHandle(stopEvent);
            stopEvent = nullptr;
            return false;
        }
        mainThread = std::thread([this, first = std::move(initial)]() mutable {
            RunThread(std::move(first));
        });
    }
    catch (...)
    {
        CloseHandle(stopEvent);
        stopEvent = nullptr;
        return false;
    }
    return true;
}

void IPCServer::Stop()
{
    std::lock_guard<std::mutex> lock(lifecycleMutex);
    if (!mainThread.joinable()) return;
    SetEvent(stopEvent);
    mainThread.join();
    CloseHandle(stopEvent);
    stopEvent = nullptr;
}

std::unique_ptr<IPCServer::PipeInstance> IPCServer::CreatePipeInstance(bool first)
{
    auto instance = std::make_unique<PipeInstance>();
    instance->overlap.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    if (!instance->overlap.hEvent) return nullptr;
    instance->pipe = CreateNamedPipeA(pipeName.c_str(), PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED
        | (first ? FILE_FLAG_FIRST_PIPE_INSTANCE : 0),
        PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT | PIPE_REJECT_REMOTE_CLIENTS,
        PIPE_UNLIMITED_INSTANCES, sizeof(protocol::Response), sizeof(protocol::Request), 1000, nullptr);
    if (instance->pipe == INVALID_HANDLE_VALUE)
    {
        LOG("CreateNamedPipe failed: %lu", GetLastError());
        return nullptr;
    }
    if (ConnectNamedPipe(instance->pipe, &instance->overlap))
        SetEvent(instance->overlap.hEvent);
    else
    {
        const DWORD error = GetLastError();
        if (error == ERROR_IO_PENDING) instance->pending = true;
        else if (error == ERROR_PIPE_CONNECTED) SetEvent(instance->overlap.hEvent);
        else return nullptr;
    }
    return instance;
}

bool IPCServer::Advance(PipeInstance& instance)
{
    DWORD bytes = instance.completedBytes;
    if (instance.pending)
    {
        // Event completion guarantees this never waits on a client.
        const BOOL complete = GetOverlappedResult(instance.pipe, &instance.overlap, &bytes, FALSE);
        instance.pending = false;
        if (!complete) return false;
    }
    switch (instance.operation)
    {
    case PipeInstance::Operation::Connect:
        instance.connected = true;
        instance.operation = PipeInstance::Operation::Read;
        break;
    case PipeInstance::Operation::Read:
        // Never dispatch a partial request or reuse fields from the prior message.
        if (bytes != sizeof(protocol::Request)) return false;
        std::memset(&instance.response, 0, sizeof instance.response);
        try { HandleRequest(instance.request, instance.response); }
        catch (...) { return false; }
        instance.operation = PipeInstance::Operation::Write;
        break;
    case PipeInstance::Operation::Write:
        if (bytes != sizeof(protocol::Response)) return false;
        instance.operation = PipeInstance::Operation::Read;
        break;
    }

    HANDLE event = instance.overlap.hEvent;
    instance.overlap = {};
    instance.overlap.hEvent = event;
    ResetEvent(event);
    BOOL started;
    if (instance.operation == PipeInstance::Operation::Read)
    {
        std::memset(&instance.request, 0, sizeof instance.request);
        started = ReadFile(instance.pipe, &instance.request, sizeof instance.request, &bytes, &instance.overlap);
    }
    else
        started = WriteFile(instance.pipe, &instance.response, sizeof instance.response, &bytes, &instance.overlap);
    if (started)
    {
        instance.completedBytes = bytes;
        SetEvent(event);
        return true;
    }
    if (GetLastError() != ERROR_IO_PENDING) return false;
    instance.pending = true;
    return true;
}

void IPCServer::RunThread(std::unique_ptr<PipeInstance> initial)
{
    // One event per client plus the stop event. An absent listener provides
    // backpressure at the Win32 wait-set limit instead of spawning unbounded threads.
    std::vector<std::unique_ptr<PipeInstance>> pipes;
    try
    {
        pipes.push_back(std::move(initial));
        while (WaitForSingleObject(stopEvent, 0) != WAIT_OBJECT_0)
        {
            bool listening = false;
            for (const auto& pipe : pipes)
                listening |= pipe->operation == PipeInstance::Operation::Connect;
            if (!listening && pipes.size() < MAXIMUM_WAIT_OBJECTS - 1)
            {
                auto listener = CreatePipeInstance(false);
                if (listener) pipes.push_back(std::move(listener));
            }
            std::vector<HANDLE> events{stopEvent};
            for (const auto& pipe : pipes) events.push_back(pipe->overlap.hEvent);
            // Retry a failed listener creation, while keeping active clients usable.
            DWORD result = WaitForMultipleObjects(static_cast<DWORD>(events.size()), events.data(), FALSE, 250);
            if (result == WAIT_OBJECT_0 || result == WAIT_FAILED) break;
            if (result == WAIT_TIMEOUT) continue;
            const size_t index = result - WAIT_OBJECT_0 - 1;
            if (index >= pipes.size()) break;
            if (!Advance(*pipes[index])) pipes.erase(pipes.begin() + index);
        }
    }
    catch (...) { LOG("IPC worker stopped after an internal failure%s", ""); }
    // RAII cancels and drains each pending operation before freeing its buffers.
    pipes.clear();
}
