// SteamVR driver entry point. The driver adds no devices of its own; it loads alongside the others
// ("alwaysActivate") and works on their poses through the hook in hook.cpp.
#include "common.h"

#include <cstring>

#ifndef SVR_VERSION
#define SVR_VERSION "0.2.0"
#endif

namespace svr {

class Provider : public vr::IServerTrackedDeviceProvider
{
public:
	vr::EVRInitError Init(vr::IVRDriverContext *ctx) override
	{
		VR_INIT_SERVER_DRIVER_CONTEXT(ctx);
		// The module stays pinned after a Cleanup, so statics may carry over into a second Init.
		ResetHookState();
		ResetDevices();
		m_tHook = m_tSettings = m_tCheck = 0;
		LoadSettings();
		StartTelemetry();
		Log("SteamVR Tracking Enhancement %s loaded (filter_enable=%d)", SVR_VERSION, int(CurSettings().filterEnable));
		return vr::VRInitError_None;
	}

	void Cleanup() override
	{
		RemovePoseHook();
		StopTelemetry();
		VR_CLEANUP_SERVER_DRIVER_CONTEXT();
	}

	const char *const *GetInterfaceVersions() override { return vr::k_InterfaceVersions; }

	void RunFrame() override
	{
		double now = Now();
		// First RunFrame: every driver has finished Init by now, so any hook Space Calibrator
		// installs during its Init is already in place underneath ours.
		if (m_tHook == 0)
		{
			EnsurePoseHook();
			m_tHook = now;
		}
		ResolveDeviceInfo();
		if (now - m_tSettings > 2.0)
		{
			LoadSettings();
			m_tSettings = now;
		}
		if (now - m_tHook > 20.0 && now - m_tCheck > 1.0)
		{
			CheckHookEffective();
			m_tCheck = now;
		}
	}

	bool ShouldBlockStandbyMode() override { return false; }
	void EnterStandby() override {}
	void LeaveStandby() override {}

private:
	double m_tHook = 0;
	double m_tSettings = 0;
	double m_tCheck = 0;
};

static Provider g_provider;

} // namespace svr

extern "C" __declspec(dllexport) void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
	if (std::strcmp(pInterfaceName, vr::IServerTrackedDeviceProvider_Version) == 0) return &svr::g_provider;
	if (pReturnCode) *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
	return nullptr;
}
