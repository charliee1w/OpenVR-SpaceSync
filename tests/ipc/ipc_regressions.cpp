// Native transport tests: only the provider request boundary is replaced.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <crtdbg.h>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>
#include <atomic>
#include "Protocol.h"

static char testPipeName[128];
static std::atomic<bool> observedPendingWrite = false;
// Observe the real Win32 boundary without changing its behavior. This proves
// the pending-write case actually fills the server's outbound pipe quota.
static BOOL ObserveWriteFile(HANDLE pipe, LPCVOID buffer, DWORD bytes, LPDWORD written, LPOVERLAPPED overlap) {
    const BOOL result = ::WriteFile(pipe, buffer, bytes, written, overlap);
    const DWORD error = GetLastError();
    if (!result && error == ERROR_IO_PENDING && overlap) observedPendingWrite = true;
    SetLastError(error);
    return result;
}
#undef SPACESYNC_PIPE_NAME
#define SPACESYNC_PIPE_NAME testPipeName
#define WriteFile ObserveWriteFile
#include "../../src/driver/IPCServer.cpp"
#undef WriteFile

FILE* LogFile = nullptr;
tm TimeForLog() { return {}; }
void LogFlush() { fflush(LogFile); }
void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform&) {}
void ServerTrackedDeviceProvider::SetHmdTracker(const protocol::SetHmdTracker&) {}
void ServerTrackedDeviceProvider::SetSlamSync(const protocol::SetSlamSync&) {}
void ServerTrackedDeviceProvider::SetOneEuro(const protocol::SetOneEuro&) {}
void ServerTrackedDeviceProvider::GetStatus(protocol::DriverStatus&) { throw std::runtime_error("short request dispatched"); }

static void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

static HANDLE connectClient() {
    const ULONGLONG deadline = GetTickCount64() + 3000;
    while (GetTickCount64() < deadline) {
        HANDLE client = CreateFileA(testPipeName, GENERIC_READ | GENERIC_WRITE,
                                    0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (client != INVALID_HANDLE_VALUE) {
            DWORD mode = PIPE_READMODE_MESSAGE;
            check(SetNamedPipeHandleState(client, &mode, nullptr, nullptr), "client mode");
            return client;
        }
        Sleep(5);
    }
    throw std::runtime_error("listener was not available");
}

static void handshake(HANDLE client, bool receive = true) {
    protocol::Request request(protocol::RequestHandshake);
    DWORD count = 0;
    check(WriteFile(client, &request, sizeof request, &count, nullptr) && count == sizeof request,
          "handshake request");
    if (!receive) return;
    protocol::Response response;
    check(ReadFile(client, &response, sizeof response, &count, nullptr) && count == sizeof response,
          "handshake response");
    check(response.type == protocol::ResponseHandshake && response.protocol.version == protocol::Version,
          "handshake content");
}

static void runCycle(unsigned clients, bool receive, bool shortRequest) {
    IPCServer server(nullptr);
    check(server.Run(), "server startup");
    std::vector<HANDLE> connections;
    for (unsigned i = 0; i < clients; ++i) {
        HANDLE client = connectClient();
        connections.push_back(client);
        if (shortRequest) {
            const auto request = protocol::RequestGetStatus;
            DWORD count;
            check(WriteFile(client, &request, sizeof request, &count, nullptr), "short write");
            Sleep(30);
            char byte;
            check(!ReadFile(client, &byte, 1, &count, nullptr), "short request should close connection");
        } else {
            handshake(client, receive);
        }
    }
    if (!clients) Sleep(10);
    const auto start = GetTickCount64();
    server.Stop();
    check(GetTickCount64() - start < 3000, "stop did not cancel pending I/O promptly");
    server.Stop();
    for (HANDLE connection : connections) CloseHandle(connection);
}

int main(int argc, char** argv) {
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
    snprintf(testPipeName, sizeof testPipeName, "\\\\.\\pipe\\SpaceSyncTest_%lu", GetCurrentProcessId());
    LogFile = tmpfile();
    if (!LogFile) return 2;
    try {
        const std::string scenario = argc > 1 ? argv[1] : "one";
        if (scenario == "rapid") {
            for (int i = 0; i < 100; ++i) { IPCServer server(nullptr); check(server.Run(), "rapid startup"); server.Stop(); }
        } else if (scenario == "restart") {
            IPCServer server(nullptr);
            for (int i = 0; i < 25; ++i) {
                check(server.Run() && server.Run(), "idempotent startup");
                HANDLE client = connectClient();
                handshake(client);
                std::thread first([&] { server.Stop(); });
                std::thread second([&] { server.Stop(); });
                first.join();
                second.join();
                CloseHandle(client);
            }
        } else if (scenario == "startup-failure") {
            IPCServer owner(nullptr);
            check(owner.Run(), "owner startup");
            IPCServer collision(nullptr);
            check(!collision.Run(), "duplicate server must fail without claiming success");
            collision.Stop();
            HANDLE client = connectClient();
            handshake(client);
            CloseHandle(client);
            owner.Stop();
            check(collision.Run(), "startup can recover after the owner exits");
            client = connectClient();
            handshake(client);
            CloseHandle(client);
            collision.Stop();
            IPCServer invalid(nullptr, "invalid pipe name");
            check(!invalid.Run(), "invalid endpoint must report failure");
            invalid.Stop();
        } else if (scenario == "pending-write") {
            IPCServer server(nullptr);
            check(server.Run(), "pending-write startup");
            HANDLE client = connectClient();
            observedPendingWrite = false;
            std::thread writer([&] {
                try { for (int i = 0; i < 64; ++i) handshake(client, false); }
                catch (const std::exception&) { } // Stop disconnects a blocked writer.
            });
            const auto deadline = GetTickCount64() + 3000;
            while (!observedPendingWrite && GetTickCount64() < deadline) Sleep(1);
            server.Stop();
            writer.join();
            CloseHandle(client);
            check(observedPendingWrite, "test did not reach an actual pending WriteFile");
        } else if (scenario == "handles") {
            runCycle(0, true, false);
            DWORD before, after;
            GetProcessHandleCount(GetCurrentProcess(), &before);
            for (int i = 0; i < 25; ++i) runCycle(1, true, false);
            GetProcessHandleCount(GetCurrentProcess(), &after);
            check(after <= before + 2, "server handles leaked across restart");
        } else {
            unsigned clients = scenario == "empty" ? 0 : (scenario == "three" ? 3 : 1);
            runCycle(clients, scenario != "pending-write", scenario == "short-request");
        }
        printf("PASS %s\n", scenario.c_str());
    } catch (const std::exception& error) {
        fprintf(stderr, "%s\n", error.what());
        fclose(LogFile);
        return 1;
    }
    fclose(LogFile);
}
