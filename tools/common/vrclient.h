// OpenVR client access for the room-setup tool: loads the SteamVR runtime's openvr_api.dll at run
// time (no import library), plus the small amount of 3x4 rigid-transform math the tool needs.
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <tlhelp32.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include <openvr.h>

#include "geom.h"

namespace rs {

struct M34
{
	double m[3][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } };
};

inline M34 FromVr(const vr::HmdMatrix34_t &a)
{
	M34 r;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 4; j++) r.m[i][j] = a.m[i][j];
	return r;
}

inline vr::HmdMatrix34_t ToVr(const M34 &a)
{
	vr::HmdMatrix34_t r;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 4; j++) r.m[i][j] = float(a.m[i][j]);
	return r;
}

inline V3 Apply(const M34 &a, V3 p)
{
	return { a.m[0][0] * p.x + a.m[0][1] * p.y + a.m[0][2] * p.z + a.m[0][3], a.m[1][0] * p.x + a.m[1][1] * p.y + a.m[1][2] * p.z + a.m[1][3],
		a.m[2][0] * p.x + a.m[2][1] * p.y + a.m[2][2] * p.z + a.m[2][3] };
}

// Inverse of a rigid transform: R^T, -R^T t.
inline M34 Inverse(const M34 &a)
{
	M34 r;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) r.m[i][j] = a.m[j][i];
	for (int i = 0; i < 3; i++) r.m[i][3] = -(r.m[i][0] * a.m[0][3] + r.m[i][1] * a.m[1][3] + r.m[i][2] * a.m[2][3]);
	return r;
}

// Rotation about +Y by `yaw` followed by translation t. Columns are the rotated frame's axes:
// X = (c, 0, -s), Z = (s, 0, c), so the frame's forward (-Z) is (-s, 0, -c).
inline M34 YawTrans(double yaw, V3 t)
{
	double c = std::cos(yaw), s = std::sin(yaw);
	M34 r;
	r.m[0][0] = c;  r.m[0][1] = 0; r.m[0][2] = s; r.m[0][3] = t.x;
	r.m[1][0] = 0;  r.m[1][1] = 1; r.m[1][2] = 0; r.m[1][3] = t.y;
	r.m[2][0] = -s; r.m[2][1] = 0; r.m[2][2] = c; r.m[2][3] = t.z;
	return r;
}

// The yaw that makes YawTrans's forward equal the horizontal direction f.
inline double YawFromForward(V3 f) { return std::atan2(-f.x, -f.z); }

// 2D versions of YawTrans (standing -> raw) and its inverse, on the floor plane.
inline P2 StdToRaw2(P2 p, double yaw, P2 origin)
{
	double c = std::cos(yaw), s = std::sin(yaw);
	return { c * p.x + s * p.z + origin.x, -s * p.x + c * p.z + origin.z };
}

inline P2 RawToStd2(P2 p, double yaw, P2 origin)
{
	double c = std::cos(yaw), s = std::sin(yaw), dx = p.x - origin.x, dz = p.z - origin.z;
	return { c * dx - s * dz, s * dx + c * dz };
}

inline V3 Pos(const vr::HmdMatrix34_t &m) { return { m.m[0][3], m.m[1][3], m.m[2][3] }; }
inline V3 Forward(const vr::HmdMatrix34_t &m) { return { -m.m[0][2], -m.m[1][2], -m.m[2][2] }; }
inline double Len(V3 a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
inline V3 Sub(V3 a, V3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }

inline std::string RuntimePath()
{
	// %LOCALAPPDATA%\openvr\openvrpaths.vrpath lists the runtime; fall back to the default install.
	std::string fallback = "C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR";
	char base[MAX_PATH] = {};
	if (!GetEnvironmentVariableA("LOCALAPPDATA", base, MAX_PATH)) return fallback;
	std::ifstream f(std::string(base) + "\\openvr\\openvrpaths.vrpath");
	if (!f) return fallback;
	std::stringstream ss;
	ss << f.rdbuf();
	std::string j = ss.str();
	size_t k = j.find("\"runtime\"");
	if (k == std::string::npos) return fallback;
	size_t q1 = j.find('"', j.find('[', k) + 1);
	if (q1 == std::string::npos) return fallback;
	size_t q2 = j.find('"', q1 + 1);
	if (q2 == std::string::npos) return fallback;
	std::string raw = j.substr(q1 + 1, q2 - q1 - 1), out;
	for (size_t i = 0; i < raw.size(); i++)
	{
		if (raw[i] == '\\' && i + 1 < raw.size() && raw[i + 1] == '\\') { out += '\\'; i++; }
		else out += raw[i];
	}
	return out.empty() ? fallback : out;
}

inline bool ProcessRunning(const wchar_t *exe)
{
	HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (snap == INVALID_HANDLE_VALUE) return false;
	PROCESSENTRY32W pe = {};
	pe.dwSize = sizeof pe;
	bool found = false;
	for (BOOL ok = Process32FirstW(snap, &pe); ok; ok = Process32NextW(snap, &pe))
		if (_wcsicmp(pe.szExeFile, exe) == 0) { found = true; break; }
	CloseHandle(snap);
	return found;
}

class VrClient
{
public:
	vr::IVRSystem *sys = nullptr;
	vr::IVRChaperone *chap = nullptr;
	vr::IVRChaperoneSetup *setup = nullptr;
	vr::IVROverlay *overlay = nullptr;
	vr::IVRSettings *settings = nullptr;         // optional: null when the runtime does not offer it
	vr::IVRApplications *applications = nullptr; // optional
	std::string dllPath;

	bool Init(std::string &err)
	{
		dllPath = RuntimePath() + "\\bin\\win64\\openvr_api.dll";
		m_dll = LoadLibraryA(dllPath.c_str());
		if (!m_dll) { err = "cannot load " + dllPath; return false; }
		auto init = reinterpret_cast<InitFn>(GetProcAddress(m_dll, "VR_InitInternal2"));
		m_shutdown = reinterpret_cast<ShutdownFn>(GetProcAddress(m_dll, "VR_ShutdownInternal"));
		auto getIface = reinterpret_cast<GetIfaceFn>(GetProcAddress(m_dll, "VR_GetGenericInterface"));
		auto valid = reinterpret_cast<ValidFn>(GetProcAddress(m_dll, "VR_IsInterfaceVersionValid"));
		if (!init || !m_shutdown || !getIface || !valid) { err = "openvr_api.dll is missing an export"; return false; }

		// VR_Init starts SteamVR when it cannot reach a running one, and keeps retrying for two
		// minutes. This tool must never do either: if SteamVR is not already up and accepting
		// applications, stop before touching it. WaitNamedPipe only asks whether the server is
		// listening; it does not open a connection.
		if (!ProcessRunning(L"vrserver.exe"))
		{
			err = "SteamVR is not running. Start SteamVR, wait until the headset is tracking, then run this again.";
			return false;
		}
		if (!WaitNamedPipeA("\\\\.\\pipe\\SteamVR_Namespace", 2000))
		{
			err = "SteamVR is running but is not accepting new applications (pipe error " + std::to_string(GetLastError()) +
				"). Restart SteamVR and run this again. Nothing was changed.";
			return false;
		}

		vr::EVRInitError e = vr::VRInitError_None;
		init(&e, vr::VRApplication_Overlay, nullptr);
		if (e != vr::VRInitError_None) { err = "VR_Init failed: " + std::to_string(int(e)); return false; }
		const char *needed[] = { vr::IVRSystem_Version, vr::IVRChaperone_Version, vr::IVRChaperoneSetup_Version, vr::IVROverlay_Version };
		for (const char *v : needed)
			if (!valid(v)) { err = std::string("SteamVR does not provide ") + v; m_shutdown(); return false; }
		sys = static_cast<vr::IVRSystem *>(getIface(vr::IVRSystem_Version, &e));
		chap = static_cast<vr::IVRChaperone *>(getIface(vr::IVRChaperone_Version, &e));
		setup = static_cast<vr::IVRChaperoneSetup *>(getIface(vr::IVRChaperoneSetup_Version, &e));
		overlay = static_cast<vr::IVROverlay *>(getIface(vr::IVROverlay_Version, &e));
		if (!sys || !chap || !setup || !overlay) { err = "could not get an interface"; m_shutdown(); return false; }
		if (valid(vr::IVRSettings_Version)) settings = static_cast<vr::IVRSettings *>(getIface(vr::IVRSettings_Version, &e));
		if (valid(vr::IVRApplications_Version)) applications = static_cast<vr::IVRApplications *>(getIface(vr::IVRApplications_Version, &e));
		// The SDK's own VR_Init does this right after acquiring IVRSystem.
		e = sys->SetSDKVersion(vr::k_nSteamVRVersionMajor, vr::k_nSteamVRVersionMinor, vr::k_nSteamVRVersionBuild);
		if (e != vr::VRInitError_None) { err = "SetSDKVersion failed: " + std::to_string(int(e)); m_shutdown(); return false; }
		return true;
	}

	void Shutdown()
	{
		if (m_shutdown) m_shutdown();
		sys = nullptr; chap = nullptr; setup = nullptr; overlay = nullptr; settings = nullptr; applications = nullptr;
		if (m_dll) FreeLibrary(m_dll);
		m_dll = nullptr;
	}

private:
	using InitFn = uint32_t (*)(vr::EVRInitError *, vr::EVRApplicationType, const char *);
	using ShutdownFn = void (*)();
	using GetIfaceFn = void *(*)(const char *, vr::EVRInitError *);
	using ValidFn = bool (*)(const char *);
	HMODULE m_dll = nullptr;
	ShutdownFn m_shutdown = nullptr;
};

} // namespace rs
