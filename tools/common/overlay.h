// A text panel in the headset: GDI renders the text into a DIB, which is uploaded with SetOverlayRaw.
// Text is white on grey so the RGBA/BGRA channel order cannot matter.
#pragma once

#include "vrclient.h"

#include <cstdint>
#include <vector>

namespace rs {

class TextOverlay
{
public:
	// key must be unique among all running overlays: each tool passes its own.
	bool Create(vr::IVROverlay *ov, const char *key, const char *name, std::string &err)
	{
		m_ov = ov;
		vr::EVROverlayError e = ov->CreateOverlay(key, name, &m_h);
		if (e != vr::VROverlayError_None) { err = "CreateOverlay failed: " + std::to_string(int(e)); return false; }
		ov->SetOverlayWidthInMeters(m_h, 1.1f);
		ov->SetOverlayAlpha(m_h, 1.0f);
		// 1.3 m in front of the eyes, a little below the eye line.
		M34 t;
		t.m[1][3] = -0.12;
		t.m[2][3] = -1.3;
		vr::HmdMatrix34_t vt = ToVr(t);
		ov->SetOverlayTransformTrackedDeviceRelative(m_h, vr::k_unTrackedDeviceIndex_Hmd, &vt);

		BITMAPINFO bi = {};
		bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
		bi.bmiHeader.biWidth = kW;
		bi.bmiHeader.biHeight = -kH; // top-down
		bi.bmiHeader.biPlanes = 1;
		bi.bmiHeader.biBitCount = 32;
		bi.bmiHeader.biCompression = BI_RGB;
		m_dc = CreateCompatibleDC(nullptr);
		m_bmp = CreateDIBSection(m_dc, &bi, DIB_RGB_COLORS, &m_bits, nullptr, 0);
		if (!m_dc || !m_bmp || !m_bits) { err = "GDI setup failed"; return false; }
		SelectObject(m_dc, m_bmp);
		m_font = CreateFontW(-44, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
			CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
		m_rgba.resize(size_t(kW) * kH * 4);
		ov->ShowOverlay(m_h);
		return true;
	}

	void SetText(const std::wstring &text)
	{
		if (!m_dc) return;
		RECT full = { 0, 0, kW, kH };
		HBRUSH bg = CreateSolidBrush(RGB(34, 34, 38));
		FillRect(m_dc, &full, bg);
		DeleteObject(bg);
		HGDIOBJ old = SelectObject(m_dc, m_font);
		SetBkMode(m_dc, TRANSPARENT);
		SetTextColor(m_dc, RGB(240, 240, 240));
		RECT r = { 40, 30, kW - 40, kH - 30 };
		DrawTextW(m_dc, text.c_str(), -1, &r, DT_LEFT | DT_TOP | DT_WORDBREAK | DT_NOPREFIX);
		SelectObject(m_dc, old);
		GdiFlush();
		const uint8_t *src = static_cast<const uint8_t *>(m_bits);
		for (size_t i = 0; i < size_t(kW) * kH; i++)
		{
			m_rgba[i * 4 + 0] = src[i * 4 + 2];
			m_rgba[i * 4 + 1] = src[i * 4 + 1];
			m_rgba[i * 4 + 2] = src[i * 4 + 0];
			m_rgba[i * 4 + 3] = 255;
		}
		m_ov->SetOverlayRaw(m_h, m_rgba.data(), kW, kH, 4);
	}

	void Visible(bool on)
	{
		if (!m_ov || m_h == vr::k_ulOverlayHandleInvalid) return;
		if (on) m_ov->ShowOverlay(m_h);
		else m_ov->HideOverlay(m_h);
	}

	void Destroy()
	{
		if (m_ov && m_h != vr::k_ulOverlayHandleInvalid) m_ov->DestroyOverlay(m_h);
		m_h = vr::k_ulOverlayHandleInvalid;
		if (m_font) DeleteObject(m_font);
		if (m_bmp) DeleteObject(m_bmp);
		if (m_dc) DeleteDC(m_dc);
		m_font = nullptr; m_bmp = nullptr; m_dc = nullptr; m_bits = nullptr;
	}

private:
	static const int kW = 1024, kH = 640;
	vr::IVROverlay *m_ov = nullptr;
	vr::VROverlayHandle_t m_h = vr::k_ulOverlayHandleInvalid;
	HDC m_dc = nullptr;
	HBITMAP m_bmp = nullptr;
	void *m_bits = nullptr;
	HFONT m_font = nullptr;
	std::vector<uint8_t> m_rgba;
};

} // namespace rs
