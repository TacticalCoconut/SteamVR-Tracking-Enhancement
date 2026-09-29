// Chaperone helpers shared by the tools: which way SteamVR's standing matrix goes (tested, never
// assumed), a backup of the live setup, and the floor shift.
#pragma once

#include "telemetry_read.h"
#include "vrclient.h"

namespace rs {

// SteamVR names the matrix "StandingZeroPoseToRawTrackingPose". Whether it maps standing to raw or
// raw to standing is decided by testing both against the headset's own raw and standing poses.
// Returns false when neither fits within 2 cm.
inline bool DetectConvention(const vr::HmdMatrix34_t &live, V3 hmdRaw, V3 hmdStanding, bool &stdToRaw, double &e1, double &e2)
{
	M34 M = FromVr(live);
	e1 = Len(Sub(Apply(Inverse(M), hmdRaw), hmdStanding));
	e2 = Len(Sub(Apply(M, hmdRaw), hmdStanding));
	if (e1 < 0.02) { stdToRaw = true; return true; }
	if (e2 < 0.02) { stdToRaw = false; return true; }
	return false;
}

inline M34 StandingToRaw(const vr::HmdMatrix34_t &m, bool stdToRaw) { return stdToRaw ? FromVr(m) : Inverse(FromVr(m)); }
inline vr::HmdMatrix34_t ToSteamVr(const M34 &standingToRaw, bool stdToRaw) { return ToVr(stdToRaw ? standingToRaw : Inverse(standingToRaw)); }

// SteamVR's config directory from openvrpaths.vrpath, with the default install as fallback.
inline std::string ConfigDir()
{
	std::string fallback = "C:\\Program Files (x86)\\Steam\\config";
	char base[MAX_PATH] = {};
	if (!GetEnvironmentVariableA("LOCALAPPDATA", base, MAX_PATH)) return fallback;
	std::string j = ReadFile(std::string(base) + "\\openvr\\openvrpaths.vrpath");
	size_t k = j.find("\"config\"");
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

// Saves the live chaperone to %LOCALAPPDATA%\SVREnhance\backup: SteamVR's own export, and a copy of
// the file. Returns true when at least one of them was written.
inline bool BackupChaperone(vr::IVRChaperoneSetup *setup, std::string &where)
{
	std::string dir = TelemetryDir() + "\\backup";
	CreateDirectoryA(TelemetryDir().c_str(), nullptr);
	CreateDirectoryA(dir.c_str(), nullptr);
	std::string stamp = Stamp();
	uint32_t len = 0;
	bool any = false;
	if (setup->ExportLiveToBuffer(nullptr, &len) && len > 0)
	{
		std::vector<char> buf(size_t(len) + 1, 0);
		if (setup->ExportLiveToBuffer(buf.data(), &len))
		{
			std::ofstream f(dir + "\\chaperone_export_" + stamp + ".json", std::ios::binary);
			f.write(buf.data(), len);
			any = true;
		}
	}
	std::string src = ConfigDir() + "\\chaperone_info.vrchap";
	std::string dst = dir + "\\chaperone_info_" + stamp + ".vrchap";
	if (CopyFileA(src.c_str(), dst.c_str(), FALSE)) any = true;
	where = dir;
	return any;
}

// Raises the virtual floor by dy metres (negative lowers it): the standing origin moves up in raw
// space, so every height an application sees drops by dy. Bounds and yaw are untouched.
inline bool ShiftFloor(vr::IVRChaperoneSetup *setup, bool stdToRaw, double dy)
{
	setup->RevertWorkingCopy();
	vr::HmdMatrix34_t m;
	if (!setup->GetWorkingStandingZeroPoseToRawTrackingPose(&m)) return false;
	M34 s2r = StandingToRaw(m, stdToRaw);
	s2r.m[1][3] += dy;
	vr::HmdMatrix34_t out = ToSteamVr(s2r, stdToRaw);
	setup->SetWorkingStandingZeroPoseToRawTrackingPose(&out);
	return setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}

} // namespace rs
