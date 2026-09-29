// Intercepts IVRServerDriverHost_006::TrackedDevicePoseUpdated, the call every driver (lighthouse
// included) makes to hand a new pose to vrserver.
//
// The hook patches the interface's vtable slot instead of the function's code. OpenVR-SpaceCalibrator
// (installed on this PC as 01spacecalibrator) inline-hooks the function body itself with MinHook, so the
// two compose: vtable slot -> our detour -> original pointer, which lands in Space Calibrator's patched
// code. Neither hook overwrites the other, and either one can be removed independently.
#include "common.h"

#include <windows.h>

namespace svr {

using PoseFn = void (*)(vr::IVRServerDriverHost *, uint32_t, const vr::DriverPose_t &, uint32_t);

// IVRServerDriverHost_006 vtable order (openvr_driver.h): [0] TrackedDeviceAdded, [1] TrackedDevicePoseUpdated.
static const int kPoseSlot = 1;

static std::mutex g_installM;
static bool g_tried = false;
static void **g_slot = nullptr;
static PoseFn g_orig = nullptr;
static std::atomic<bool> g_active{ false };
static std::atomic<int> g_inflight{ 0 };
static std::atomic<uint64_t> g_calls{ 0 };
static std::atomic<int> g_effective{ -1 };

static void Detour(vr::IVRServerDriverHost *self, uint32_t idx, const vr::DriverPose_t &pose, uint32_t size)
{
	g_inflight.fetch_add(1, std::memory_order_acq_rel);
	g_calls.fetch_add(1, std::memory_order_relaxed);
	if (g_active.load(std::memory_order_acquire) && idx < kMaxDevices && size == sizeof(vr::DriverPose_t))
	{
		vr::DriverPose_t p = pose;
		ProcessPose(idx, p);
		g_orig(self, idx, p, size);
	}
	else
	{
		g_orig(self, idx, pose, size);
	}
	g_inflight.fetch_sub(1, std::memory_order_acq_rel);
}

static bool WriteSlot(void **slot, void *value)
{
	DWORD old = 0;
	if (!VirtualProtect(slot, sizeof(void *), PAGE_READWRITE, &old)) return false;
	InterlockedExchangePointer(slot, value);
	DWORD tmp = 0;
	VirtualProtect(slot, sizeof(void *), old, &tmp);
	return true;
}

void ResetHookState()
{
	std::lock_guard<std::mutex> lk(g_installM);
	if (g_slot) return; // a previous Init's hook is still live; Cleanup removes it first
	g_tried = false;
	g_effective.store(-1);
	g_calls.store(0);
}

void EnsurePoseHook()
{
	std::lock_guard<std::mutex> lk(g_installM);
	if (g_tried) return;
	g_tried = true;

	// Other drivers' threads may have loaded our vtable entry an instant before Cleanup restores
	// it, or may be inside g_orig when we return. Pinning the module keeps Detour mapped for the
	// life of the process so that such a call can never land in unmapped memory.
	HMODULE self = nullptr;
	if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
			reinterpret_cast<LPCWSTR>(&Detour), &self))
	{
		Log("ERROR: could not pin the driver module (%lu); running as monitor only", GetLastError());
		return;
	}

	vr::IVRServerDriverHost *host = vr::VRServerDriverHost();
	if (!host)
	{
		Log("ERROR: no %s interface; running as monitor only", vr::IVRServerDriverHost_Version);
		return;
	}
	void **vtable = *reinterpret_cast<void ***>(host);
	void **slot = &vtable[kPoseSlot];
	void *cur = *slot;
	if (cur == reinterpret_cast<void *>(&Detour)) return;

	const unsigned char *code = static_cast<const unsigned char *>(cur);
	bool alreadyPatched = code[0] == 0xE9 || (code[0] == 0xFF && code[1] == 0x25);

	g_orig = reinterpret_cast<PoseFn>(cur);
	g_active.store(true, std::memory_order_release);
	if (!WriteSlot(slot, reinterpret_cast<void *>(&Detour)))
	{
		g_active.store(false);
		Log("ERROR: VirtualProtect on the vtable failed (%lu); running as monitor only", GetLastError());
		return;
	}
	g_slot = slot;
	Log("pose hook installed: host %p vtable %p slot[%d] %p -> %p%s", host, vtable, kPoseSlot, cur,
		reinterpret_cast<void *>(&Detour), alreadyPatched ? " (target already inline-hooked, e.g. Space Calibrator; chaining)" : "");
}

void RemovePoseHook()
{
	std::lock_guard<std::mutex> lk(g_installM);
	g_tried = true; // the telemetry thread must not install it again while we unload
	if (!g_slot) return;
	if (*g_slot == reinterpret_cast<void *>(&Detour))
		WriteSlot(g_slot, reinterpret_cast<void *>(g_orig));
	else
		Log("WARNING: something replaced our vtable entry (%p); leaving it as is", *g_slot);
	g_active.store(false, std::memory_order_release);
	// The module is pinned, so a straggler entering Detour now just passes the pose to g_orig.
	// Give threads already inside ProcessPose a moment to leave before the device state is reset.
	for (int i = 0; i < 100 && g_inflight.load(std::memory_order_acquire) > 0; i++) Sleep(1);
	g_slot = nullptr;
	Log("pose hook removed after %llu calls", static_cast<unsigned long long>(g_calls.load()));
}

uint64_t HookCalls() { return g_calls.load(std::memory_order_relaxed); }
bool HookInstalled() { return g_slot != nullptr; }
int HookEffective() { return g_effective.load(); }

// If devices are tracking but no pose ever passed through the detour, the vtable we patched is not
// the one other drivers call through. Report that plainly rather than pretending to filter.
void CheckHookEffective()
{
	if (!g_slot || g_effective.load() == 1) return;
	if (g_calls.load() > 0)
	{
		// Also clears an earlier 0: devices may have started tracking after the first check.
		g_effective.store(1);
		Log("pose hook verified: poses from other drivers are flowing through it");
		return;
	}
	if (g_effective.load() == 0) return;
	vr::IVRServerDriverHost *host = vr::VRServerDriverHost();
	if (!host) return;
	vr::TrackedDevicePose_t poses[kMaxDevices] = {};
	host->GetRawTrackedDevicePoses(0.0f, poses, kMaxDevices);
	int tracking = 0;
	for (auto &p : poses)
		if (p.bDeviceIsConnected && p.bPoseIsValid) tracking++;
	if (tracking > 0)
	{
		g_effective.store(0);
		Log("WARNING: %d devices are tracking but no pose reached the hook; filtering is NOT active", tracking);
	}
}

} // namespace svr
