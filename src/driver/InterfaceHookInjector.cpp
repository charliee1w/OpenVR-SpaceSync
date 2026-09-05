// SPDX-License-Identifier: AGPL-3.0-only

#include "Logging.h"
#include "Hooking.h"
#include "InterfaceHookInjector.h"
#include "ServerTrackedDeviceProvider.h"
#include "Main.h"
#include "HookRundown.h"

static HookRundown detours;

static Hook<void*(*)(void*, const char *, vr::EVRInitError *)>
	GetGenericInterfaceHook("IVRDriverContext::GetGenericInterface");

static Hook<void(*)(void*, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook005("IVRServerDriverHost005::TrackedDevicePoseUpdated");

static Hook<void(*)(void*, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook006("IVRServerDriverHost006::TrackedDevicePoseUpdated");

static void DetourTrackedDevicePoseUpdated005(void* _this, uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	HookRundown::Lease lease(detours);
	if (!lease)
	{
		auto original = lease.UseRestoredTarget() ? TrackedDevicePoseUpdatedHook005.TargetFunction() : TrackedDevicePoseUpdatedHook005.originalFunc;
		original(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}
	if (sizeof(vr::DriverPose_t) != unPoseStructSize)
		return;
	//TRACE("ServerTrackedDeviceProvider::DetourTrackedDevicePoseUpdated(%d)", unWhichDevice);
	auto pose = newPose;
	if (g_server.HandleDevicePoseUpdated(unWhichDevice, pose))
	{
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
	}
}

static void DetourTrackedDevicePoseUpdated006(void* _this, uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	HookRundown::Lease lease(detours);
	if (!lease)
	{
		auto original = lease.UseRestoredTarget() ? TrackedDevicePoseUpdatedHook006.TargetFunction() : TrackedDevicePoseUpdatedHook006.originalFunc;
		original(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}
	if (sizeof(vr::DriverPose_t) != unPoseStructSize)
		return;
	//TRACE("ServerTrackedDeviceProvider::DetourTrackedDevicePoseUpdated(%d)", unWhichDevice);
	auto pose = newPose;
	if (g_server.HandleDevicePoseUpdated(unWhichDevice, pose))
	{
		TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
	}
}

static void *DetourGetGenericInterface(void* _this, const char *pchInterfaceVersion, vr::EVRInitError *peError)
{
	HookRundown::Lease lease(detours);
	if (!lease)
	{
		auto original = lease.UseRestoredTarget() ? GetGenericInterfaceHook.TargetFunction() : GetGenericInterfaceHook.originalFunc;
		return original(_this, pchInterfaceVersion, peError);
	}
	TRACE("ServerTrackedDeviceProvider::DetourGetGenericInterface(%s)", pchInterfaceVersion);
	auto originalInterface = GetGenericInterfaceHook.originalFunc(_this, pchInterfaceVersion, peError);

	detours.RegisterWhileOpen([&] {
	std::string iface(pchInterfaceVersion);
	if (iface == "IVRServerDriverHost_005")
	{
		if (!IHook::Exists(TrackedDevicePoseUpdatedHook005.name))
		{
			if (TrackedDevicePoseUpdatedHook005.CreateHookInObjectVTable(originalInterface, 1, &DetourTrackedDevicePoseUpdated005))
				IHook::Register(&TrackedDevicePoseUpdatedHook005);
		}
	}
	else if (iface == "IVRServerDriverHost_006")
	{
		if (!IHook::Exists(TrackedDevicePoseUpdatedHook006.name))
		{
			if (TrackedDevicePoseUpdatedHook006.CreateHookInObjectVTable(originalInterface, 1, &DetourTrackedDevicePoseUpdated006))
				IHook::Register(&TrackedDevicePoseUpdatedHook006);
		}
	}

	});
	return originalInterface;
}

bool InjectHooks(vr::IVRDriverContext *pDriverContext)
{
	// MinHook can redirect a thread just before entry jumps are disabled, before
	// it reaches our lease. Keep this module mapped until process exit so that a
	// late pass-through detour remains executable even after OpenVR unloads us.
	HMODULE pinnedModule = nullptr;
	if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
		reinterpret_cast<LPCWSTR>(&InjectHooks), &pinnedModule))
	{
		LOG("Could not retain detour module: %lu", GetLastError());
		return false;
	}
	auto err = MH_Initialize();
	if (err == MH_OK)
	{
		bool created = false;
		detours.RegisterWhileOpen([&] {
			created = GetGenericInterfaceHook.CreateHookInObjectVTable(pDriverContext, 0, &DetourGetGenericInterface);
			if (created) IHook::Register(&GetGenericInterfaceHook);
		});
		return created;
	}
	else
	{
		LOG("MH_Initialize error: %s", MH_StatusToString(err));
	}
	return false;
}

void DisableHooks()
{
	const bool detached = detours.StopAndDrain([] {
		const auto result = MH_DisableHook(MH_ALL_HOOKS);
		return result == MH_OK || result == MH_ERROR_NOT_INITIALIZED;
	});
	if (!detached)
	{
		// Retain trampolines if entry jumps could not be detached. Closed detours
		// only forward poses; they no longer touch provider state or logging.
		LOG("Could not disable hooks; retaining trampoline storage%s", "");
		return;
	}
	IHook::DestroyAll();
	MH_Uninitialize();
}
