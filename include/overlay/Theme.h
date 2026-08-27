// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include <imgui.h>

namespace ui
{
	struct Palette
	{
		unsigned pageBg       = 0x101215;
		unsigned card         = 0x191c21;
		unsigned cardActive   = 0x1c2027;
		unsigned titleBar     = 0x15181c;
		unsigned border       = 0x24282e;
		unsigned borderStrong = 0x2a2e35;
		unsigned borderButton = 0x2f343c;
		unsigned inputBg      = 0x12151a;
		unsigned button       = 0x22262c;
		unsigned buttonHover  = 0x282d34;
		unsigned stepHover    = 0x2a3038;
		unsigned accent       = 0x3f6ee0;
		unsigned accentHover  = 0x4a78e8;
		unsigned textBright   = 0xf0f2f5;
		unsigned textStrong   = 0xe4e7ec;
		unsigned text         = 0xd5dae1;
		unsigned textTitle    = 0xc9ced6;
		unsigned textButton   = 0xa4abb5;
		unsigned textMuted    = 0x8b929c;
		unsigned textDim      = 0x767d88;
		unsigned textIcon     = 0x7c828d;
		unsigned textFooter   = 0x5e646e;
		unsigned textDisabled = 0x6a707a;
		unsigned green        = 0x7aa88c;
		unsigned greenBorder  = 0x3a4a44;
		unsigned blueBorder   = 0x2f3d5c;
		unsigned link         = 0x6f9dea;
		unsigned linkHover    = 0x8fb4f0;
		unsigned danger       = 0xb23b3b;
		unsigned dangerHover  = 0xc04343;
		unsigned yellow       = 0xc9a95c;
		unsigned knob         = 0xdfe4ea;
		unsigned checkBorder  = 0x3a4048;
		unsigned rowRule      = 0x1f2328;
		unsigned overlay      = 0x0c0e11;
	};

	struct Fonts
	{
		ImFont* regular = nullptr;
		ImFont* medium = nullptr;
		ImFont* semibold = nullptr;
		ImFont* bold = nullptr;
		ImFont* mono = nullptr;
		ImFont* monoMedium = nullptr;
	};

	extern Palette P;
	extern Fonts F;

	void Init(float displayScale, bool srgbFramebuffer);

	void SetDisplayScale(float scale);
	void SetContentScale(float scale);
	float DisplayScale();
	float ContentScale();
	float S();
	float px(float designPx);

	ImU32 Col(unsigned rgb, float alpha = 1.0f);
	ImVec4 ColV(unsigned rgb, float alpha = 1.0f);

	float FontPx(float designSize);
	void PushFont(ImFont* font, float designSize);
	void PopFont();
	ImVec2 TextSize(ImFont* font, float designSize, const char* text, float wrapDesignWidth = 0.0f);
	void Text(ImFont* font, float designSize, unsigned rgb, const char* text);
	void TextWrapped(ImFont* font, float designSize, unsigned rgb, float wrapDesignWidth, const char* text);
	void DrawText(ImDrawList* dl, ImFont* font, float designSize, ImVec2 pos, unsigned rgb, const char* text);
	void DrawTextCentered(ImDrawList* dl, ImFont* font, float designSize, ImVec2 center, unsigned rgb, const char* text);

	enum class Icon { Check, Ring, Dot, ArrowLeft, ArrowRight, ArrowUp, ArrowDown, Cross, Minus, Plus, Square };
	void DrawIcon(ImDrawList* dl, Icon icon, ImVec2 center, float designSize, ImU32 col, float designThickness = 1.6f);

	bool TabItem(const char* label, bool active);

	enum class ButtonKind { Secondary, Primary, Danger, Ghost };
	struct ButtonOpts
	{
		ButtonKind kind = ButtonKind::Secondary;
		float padX = 18.0f, padY = 9.0f;
		float minWidth = 0.0f;
		float width = 0.0f;
		float fontSize = 13.0f;
		ImFont* font = nullptr;
		bool enabled = true;
	};
	bool Button(const char* label, const ButtonOpts& opts = {});
	bool Link(const char* label, float fontSize = 13.0f);

	bool Checkbox(const char* label, bool* value, float rowPadY = 0.0f);
	bool Radio(const char* label, bool active);

	bool CheckboxRow(const char* label, const char* hint, bool* value, float designWidth);
	bool RadioRow(const char* label, const char* hint, bool active, float designWidth);
	void Hint(const char* text, float designWidth);

	bool Slider(const char* id, double* value, double minValue, double maxValue, float designWidth);
	bool Stepper(const char* id, double* value, double step, int decimals, float designWidth, bool enabled = true);
	bool Pill(const char* label, bool active, ImFont* font = nullptr, float fontSize = 12.0f);

	void SectionHeader(const char* label, float designWidth);
	void HLine(float designWidth, unsigned rgb = 0);
	void VSpace(float designPx);

	bool HoverHand();
}
