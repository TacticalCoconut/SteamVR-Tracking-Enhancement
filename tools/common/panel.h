// A clickable panel for a SteamVR dashboard overlay: GDI draws the title, a message and a set of
// buttons into a bitmap that is handed to SteamVR as raw pixels; the laser pointer's clicks come
// back as mouse events and are matched against the button rectangles.
#pragma once

#include "vrclient.h"

#include <cstdint>
#include <string>
#include <vector>

namespace rs {

struct Button
{
	int id = 0;
	RECT r = {};
	std::wstring label;
	COLORREF color = RGB(58, 96, 150);
	bool enabled = true;
};

class Panel
{
public:
	static const int kW = 1024, kH = 768;

	bool Init()
	{
		BITMAPINFO bi = {};
		bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
		bi.bmiHeader.biWidth = kW;
		bi.bmiHeader.biHeight = -kH; // top-down
		bi.bmiHeader.biPlanes = 1;
		bi.bmiHeader.biBitCount = 32;
		bi.bmiHeader.biCompression = BI_RGB;
		m_dc = CreateCompatibleDC(nullptr);
		m_bmp = CreateDIBSection(m_dc, &bi, DIB_RGB_COLORS, &m_bits, nullptr, 0);
		if (!m_dc || !m_bmp || !m_bits) return false;
		SelectObject(m_dc, m_bmp);
		m_title = MakeFont(-46, FW_SEMIBOLD);
		m_text = MakeFont(-30, FW_NORMAL);
		m_btn = MakeFont(-30, FW_SEMIBOLD);
		m_rgba.resize(size_t(kW) * kH * 4);
		return m_title && m_text && m_btn;
	}

	void Render(const std::wstring &title, const std::wstring &status, const std::wstring &message, const std::vector<Button> &buttons)
	{
		if (!m_dc) return;
		Fill({ 0, 0, kW, kH }, RGB(30, 30, 34));
		SetBkMode(m_dc, TRANSPARENT);
		Text(m_title, RGB(245, 245, 245), { 36, 22, kW - 36, 84 }, title, DT_LEFT | DT_SINGLELINE);
		Text(m_text, RGB(170, 170, 176), { 36, 86, kW - 36, 160 }, status, DT_LEFT | DT_WORDBREAK);
		for (const Button &b : buttons)
		{
			Fill(b.r, b.enabled ? b.color : RGB(60, 60, 66));
			RECT inner = { b.r.left + 10, b.r.top + 6, b.r.right - 10, b.r.bottom - 6 };
			Text(m_btn, b.enabled ? RGB(255, 255, 255) : RGB(130, 130, 136), inner, b.label, DT_CENTER | DT_VCENTER | DT_WORDBREAK);
		}
		Fill({ 36, kMessageTop, kW - 36, kH - 30 }, RGB(22, 22, 25));
		Text(m_text, RGB(235, 235, 235), { 56, kMessageTop + 16, kW - 56, kH - 44 }, message, DT_LEFT | DT_WORDBREAK);
		GdiFlush();
		const uint8_t *src = static_cast<const uint8_t *>(m_bits);
		for (size_t i = 0; i < size_t(kW) * kH; i++)
		{
			m_rgba[i * 4 + 0] = src[i * 4 + 2];
			m_rgba[i * 4 + 1] = src[i * 4 + 1];
			m_rgba[i * 4 + 2] = src[i * 4 + 0];
			m_rgba[i * 4 + 3] = 255;
		}
	}

	std::vector<uint8_t> &Pixels() { return m_rgba; }

	// Mouse events are in pixels with the origin at the BOTTOM left (openvr.h, VREvent_Mouse_t).
	static int Hit(const std::vector<Button> &buttons, float x, float yFromBottom)
	{
		LONG px = LONG(x), py = LONG(float(kH) - yFromBottom);
		for (const Button &b : buttons)
			if (b.enabled && px >= b.r.left && px < b.r.right && py >= b.r.top && py < b.r.bottom) return b.id;
		return -1;
	}

	void Destroy()
	{
		for (HFONT f : { m_title, m_text, m_btn })
			if (f) DeleteObject(f);
		if (m_bmp) DeleteObject(m_bmp);
		if (m_dc) DeleteDC(m_dc);
		m_title = m_text = m_btn = nullptr;
		m_bmp = nullptr;
		m_dc = nullptr;
		m_bits = nullptr;
	}

	static const int kMessageTop = 470;

private:
	static HFONT MakeFont(int h, int weight)
	{
		return CreateFontW(h, 0, 0, 0, weight, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
			DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
	}
	void Fill(RECT r, COLORREF c)
	{
		HBRUSH b = CreateSolidBrush(c);
		FillRect(m_dc, &r, b);
		DeleteObject(b);
	}
	void Text(HFONT f, COLORREF c, RECT r, const std::wstring &s, UINT fmt)
	{
		HGDIOBJ old = SelectObject(m_dc, f);
		SetTextColor(m_dc, c);
		if ((fmt & DT_VCENTER) && (fmt & DT_WORDBREAK))
		{
			// DT_VCENTER only works for single lines: measure, then centre by hand.
			RECT m = r;
			DrawTextW(m_dc, s.c_str(), -1, &m, (fmt & ~DT_VCENTER) | DT_CALCRECT | DT_NOPREFIX);
			LONG h = m.bottom - m.top, pad = ((r.bottom - r.top) - h) / 2;
			if (pad > 0) r.top += pad;
			fmt &= ~UINT(DT_VCENTER);
		}
		DrawTextW(m_dc, s.c_str(), -1, &r, fmt | DT_NOPREFIX);
		SelectObject(m_dc, old);
	}

	HDC m_dc = nullptr;
	HBITMAP m_bmp = nullptr;
	void *m_bits = nullptr;
	HFONT m_title = nullptr, m_text = nullptr, m_btn = nullptr;
	std::vector<uint8_t> m_rgba;
};

} // namespace rs
