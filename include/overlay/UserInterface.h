// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <string>
#include <vector>

#include <openvr.h>

struct VRDevice
{
	int id = -1;
	vr::TrackedDeviceClass deviceClass;
	std::string model = "";
	std::string serial = "";
	std::string trackingSystem = "";
	vr::ETrackedControllerRole controllerRole = vr::TrackedControllerRole_Invalid;
};

struct VRState
{
	std::vector<std::string> trackingSystems;
	std::vector<VRDevice> devices;
};

class UserInterface
{
public:
	// Title bar buttons (desktop only).
	enum class WindowAction { None, Minimize, Close };

	// Design size in design px (times ui::S()).
	static constexpr float DesignWidth = 1080.0f;
	static constexpr float DesignHeight = 700.0f;
	static constexpr float TitleBarHeight = 40.0f;
	static constexpr float TitleBarButtonWidth = 40.0f;
	static constexpr int TitleBarButtonCount = 2;

	WindowAction Render(bool runningInOverlay);

private:
	enum class Tab { Calibration, Smoothing, Settings };

	struct Status
	{
		const VRDevice* hmd = nullptr;
		const VRDevice* tracker = nullptr;
		std::string headline;   // e.g. "Follow mode active"
		std::string detail;     // e.g. tracker serial
		unsigned color = 0;     // ui palette colour
		bool ok = false;
	};

	void CollectDevices(VRState& state) const;
	Status BuildStatus(const VRState& state) const;

	WindowAction RenderTitleBar();
	void RenderTabs();
	void RenderCalibration(const Status& status);
	void RenderEdit(const Status& status);
	void RenderSmoothing();
	void RenderSettings();
	void RenderFooter();
	void RenderWizard();
	void RenderConfirm();

	Tab tab_ = Tab::Calibration;
	bool editView_ = false;
	double editStep_ = 0.1;
	bool confirmRemove_ = false;
	bool wizardOpen_ = false;
	float contentTop_ = 0.0f;      // y where the content area starts
	float contentHeight_ = 0.0f;   // content area height in px
};
