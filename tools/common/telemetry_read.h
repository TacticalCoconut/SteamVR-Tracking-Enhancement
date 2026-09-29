// Readers for what the SteamVR Tracking Enhancement driver writes to %LOCALAPPDATA%\SVREnhance (status.json,
// events.csv, coverage.csv), plus small text helpers. The formats are the driver's own.
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace rs {

inline std::string F(const char *fmt, ...)
{
	char buf[2048];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof buf, fmt, ap);
	va_end(ap);
	return buf;
}

inline double Now()
{
	static const double inv = [] { LARGE_INTEGER f; QueryPerformanceFrequency(&f); return 1.0 / double(f.QuadPart); }();
	LARGE_INTEGER c;
	QueryPerformanceCounter(&c);
	return double(c.QuadPart) * inv;
}

inline std::string TelemetryDir()
{
	char base[MAX_PATH] = {};
	if (!GetEnvironmentVariableA("LOCALAPPDATA", base, MAX_PATH)) return ".";
	return std::string(base) + "\\SVREnhance";
}

inline std::string IsoHoursAgo(int hours)
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	FILETIME ft;
	SystemTimeToFileTime(&st, &ft);
	ULARGE_INTEGER u;
	u.LowPart = ft.dwLowDateTime;
	u.HighPart = ft.dwHighDateTime;
	u.QuadPart -= 36000000000ULL * uint64_t(hours);
	ft.dwLowDateTime = u.LowPart;
	ft.dwHighDateTime = u.HighPart;
	FileTimeToSystemTime(&ft, &st);
	return F("%04d-%02d-%02dT%02d:%02d:%02d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
}

inline std::string Stamp()
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	return F("%04d%02d%02d-%02d%02d%02d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
}

inline std::string ReadFile(const std::string &path)
{
	std::ifstream f(path, std::ios::binary);
	if (!f) return "";
	std::stringstream ss;
	ss << f.rdbuf();
	return ss.str();
}

struct DevStatus { std::string state; double jitterMm = 0; };
struct DriverStatus
{
	bool present = false;
	int hookEffective = -1;
	bool worldFixActive = false;
	double worldFixTiltDeg = 0;
	std::map<std::string, DevStatus> dev;
	std::vector<std::string> warnings;
};

inline std::string JStr(const std::string &obj, const char *key)
{
	std::string k = F("\"%s\": \"", key);
	size_t a = obj.find(k);
	if (a == std::string::npos) return "";
	a += k.size();
	size_t b = obj.find('"', a);
	return b == std::string::npos ? "" : obj.substr(a, b - a);
}

inline double JNum(const std::string &obj, const char *key)
{
	std::string k = F("\"%s\": ", key);
	size_t a = obj.find(k);
	if (a == std::string::npos) return NAN;
	return strtod(obj.c_str() + a + k.size(), nullptr);
}

inline DriverStatus ReadDriverStatus()
{
	DriverStatus s;
	std::string j = ReadFile(TelemetryDir() + "\\status.json");
	if (j.empty()) return s;
	s.present = true;
	size_t h = j.find("\"effective\": ");
	if (h != std::string::npos) s.hookEffective = j.compare(h + 13, 4, "true") == 0 ? 1 : (j.compare(h + 13, 5, "false") == 0 ? 0 : -1);
	size_t wf = j.find("\"world_fix\": {");
	if (wf != std::string::npos)
	{
		std::string obj = j.substr(wf, j.find('}', wf) - wf);
		s.worldFixActive = obj.find("\"active\": true") != std::string::npos;
		double t = JNum(obj, "tilt_deg");
		s.worldFixTiltDeg = std::isnan(t) ? 0 : t;
	}
	for (size_t p = j.find("{\"index\":"); p != std::string::npos; p = j.find("{\"index\":", p + 1))
	{
		size_t e = j.find('}', p);
		if (e == std::string::npos) break;
		std::string obj = j.substr(p, e - p);
		std::string serial = JStr(obj, "serial"), state = JStr(obj, "state");
		if (serial.empty() || state.empty()) continue;
		DevStatus d;
		d.state = state;
		double jm = JNum(obj, "jitter_mm");
		d.jitterMm = std::isnan(jm) ? 0 : jm;
		s.dev[serial] = d;
	}
	size_t w = j.find("\"warnings\": [");
	if (w != std::string::npos)
	{
		size_t end = j.find(']', w);
		for (size_t q = j.find('"', w + 13); q != std::string::npos && q < end; q = j.find('"', q + 1))
		{
			size_t q2 = j.find('"', q + 1);
			if (q2 == std::string::npos || q2 > end) break;
			s.warnings.push_back(j.substr(q + 1, q2 - q - 1));
			q = q2;
		}
	}
	return s;
}

inline std::vector<std::vector<std::string>> ReadCsv(const std::string &name, const std::string &since)
{
	std::vector<std::vector<std::string>> rows;
	std::ifstream f(TelemetryDir() + "\\" + name);
	std::string line;
	bool header = true;
	while (std::getline(f, line))
	{
		if (header) { header = false; continue; }
		if (line.compare(0, since.size(), since) < 0) continue;
		std::vector<std::string> cols;
		std::stringstream ss(line);
		std::string c;
		while (std::getline(ss, c, ',')) cols.push_back(c);
		rows.push_back(cols);
	}
	return rows;
}

} // namespace rs
