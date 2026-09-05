// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "Configuration.h"

#include <Windows.h>

#include <picojson.h>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <algorithm>

static picojson::array FloatArray(const float *buf, int numFloats)
{
	picojson::array arr;

	for (int i = 0; i < numFloats; i++)
		arr.push_back(picojson::value(double(buf[i])));

	return arr;
}

// Parse into a candidate: a malformed field must never leave a partly replaced
// tracker identity, mount, or chaperone in the active profile.
static const picojson::value &Required(const picojson::object &obj, const char *key)
{
	auto it = obj.find(key);
	if (it == obj.end())
		throw std::runtime_error(std::string("missing profile field: ") + key);
	return it->second;
}

template<typename T>
static T Typed(const picojson::value &value, const char *key)
{
	if (!value.is<T>())
		throw std::runtime_error(std::string("invalid type for profile field: ") + key);
	return value.get<T>();
}

template<typename T>
static T Optional(const picojson::object &obj, const char *key, const T &fallback)
{
	auto it = obj.find(key);
	return it == obj.end() ? fallback : Typed<T>(it->second, key);
}

static double FiniteNumber(const picojson::value &value, const char *key)
{
	double number = Typed<double>(value, key);
	if (!std::isfinite(number))
		throw std::runtime_error(std::string("non-finite profile field: ") + key);
	return number;
}

static double Number(const picojson::object &obj, const char *key, double fallback)
{
	auto it = obj.find(key);
	return it == obj.end() ? fallback : FiniteNumber(it->second, key);
}

static double Bounded(double value, double low, double high, const char *key)
{
	if (!std::isfinite(value) || value < low || value > high)
		throw std::runtime_error(std::string("out-of-range profile field: ") + key);
	return value;
}

static void LoadFloatArray(const picojson::value &obj, float *buf, size_t numFloats)
{
	const auto &arr = Typed<picojson::array>(obj, "float array");
	if (arr.size() != numFloats)
		throw std::runtime_error("wrong buffer size");
	for (size_t i = 0; i < numFloats; ++i)
	{
		double value = FiniteNumber(arr[i], "float array element");
		if (std::abs(value) > (std::numeric_limits<float>::max)())
			throw std::runtime_error("float array element exceeds float range");
		buf[i] = static_cast<float>(value);
	}
}

static void ParseProfile(CalibrationContext &ctx, std::istream &stream)
{
	picojson::value value;
	std::string error = picojson::parse(value, stream);
	if (!error.empty())
		throw std::runtime_error(error);
	auto profiles = Typed<picojson::array>(value, "profiles");
	if (profiles.empty())
		throw std::runtime_error("no profiles in file");
	auto obj = Typed<picojson::object>(profiles[0], "profile");

	CalibrationContext candidate = ctx;
	candidate.Clear();
	candidate.targetTrackingSystem = Typed<std::string>(Required(obj, "target_tracking_system"), "target_tracking_system");
	candidate.hmdSerial = Optional<std::string>(obj, "hmd_serial", "");
	candidate.trackerSerial = Optional<std::string>(obj, "tracker_serial", "");
	for (const auto *text : { &candidate.targetTrackingSystem, &candidate.hmdSerial, &candidate.trackerSerial })
		if (text->size() >= vr::k_unMaxPropertyStringSize || text->find('\0') != std::string::npos)
			throw std::runtime_error("invalid tracking-system or serial string");
	if (candidate.targetTrackingSystem.empty())
		throw std::runtime_error("empty target tracking system");

	const char *rotationKeys[] = { "roll", "yaw", "pitch" };
	const char *translationKeys[] = { "x", "y", "z" };
	for (int axis = 0; axis < 3; ++axis)
	{
		candidate.calibratedRotation(axis) = FiniteNumber(Required(obj, rotationKeys[axis]), rotationKeys[axis]);
		candidate.calibratedTranslation(axis) = FiniteNumber(Required(obj, translationKeys[axis]), translationKeys[axis]);
	}
	// Broad physical bounds reject corrupt/extreme ratios while retaining manual
	// scale edits and older profiles, whose missing fields keep their defaults.
	candidate.calibratedScale = Bounded(Number(obj, "scale", 1.0), 0.01, 100.0, "scale");
	candidate.targetModelScale = Bounded(Number(obj, "targetModelScale", candidate.calibratedScale), 0.01, 100.0, "targetModelScale");
	candidate.hmdScale = Bounded(Number(obj, "hmdScale", 1.0), 0.01, 100.0, "hmdScale");
	candidate.fallbackToSlam = Typed<bool>(Required(obj, "fallbackSlam"), "fallbackSlam");
	candidate.enableAngularVelocity = Typed<bool>(Required(obj, "eAngVel"), "eAngVel");
	candidate.continuousSync = Optional<bool>(obj, "continuousSync", true);
	candidate.followSlamHmd = Optional<bool>(obj, "followSlam", false);
	candidate.noHeadTracker = Optional<bool>(obj, "noHeadTracker", false);
	candidate.hideHeadTracker = Optional<bool>(obj, "hideHeadTracker", false);
	candidate.uiScale = static_cast<float>((std::max)(0.8, (std::min)(2.0, Number(obj, "uiScale", 1.25))));
	candidate.predictionTime = static_cast<float>(Bounded(Number(obj, "predictionTime", 1.0), 0.0, 10.0, "predictionTime"));

	auto loadOneEuro = [&](const char *key, protocol::OneEuroParams defaults) {
		auto it = obj.find(key);
		if (it == obj.end()) return defaults;
		auto filter = Typed<picojson::object>(it->second, key);
		return protocol::OneEuroParams {
			Bounded(Number(filter, "minCutoff", defaults.minCutoff), 0.001, 1000.0, "minCutoff"),
			Bounded(Number(filter, "beta", defaults.beta), 0.0, 1000.0, "beta"),
			Bounded(Number(filter, "dCutoff", defaults.dCutoff), 0.001, 1000.0, "dCutoff")
		};
	};
	candidate.headFilterEnabled = Optional<bool>(obj, "headFilterEnabled", true);
	candidate.headFilterParams = loadOneEuro("headFilter", {2.0, 0.5, 1.0});
	candidate.driftFilterParams = loadOneEuro("driftFilter", {1.0, 0.4, 0.85});

	const char *relativeKeys[] = { "rel_qw", "rel_qx", "rel_qy", "rel_qz", "rel_tx", "rel_ty", "rel_tz" };
	bool hasRelative = false;
	for (auto key : relativeKeys) hasRelative |= obj.find(key) != obj.end();
	if (hasRelative)
	{
		double q[4];
		for (int i = 0; i < 4; ++i) q[i] = FiniteNumber(Required(obj, relativeKeys[i]), relativeKeys[i]);
		double norm = std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3]));
		if (!std::isfinite(norm) || norm < 1e-6 || std::abs(norm - 1.0) > 0.01)
			throw std::runtime_error("relative rotation is not a unit quaternion");
		candidate.relativeRotation = { q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm };
		for (int axis = 0; axis < 3; ++axis)
			candidate.relativeTranslation.v[axis] = FiniteNumber(Required(obj, relativeKeys[axis + 4]), relativeKeys[axis + 4]);
		candidate.validRelativeOffset = true;
	}
	double speed = Number(obj, "calibration_speed", static_cast<double>(candidate.calibrationSpeed));
	if (speed != std::floor(speed) || speed < static_cast<double>(CalibrationContext::FAST)
		|| speed > static_cast<double>(CalibrationContext::VERY_SLOW))
		throw std::runtime_error("invalid calibration speed");
	candidate.calibrationSpeed = static_cast<CalibrationContext::Speed>(static_cast<int>(speed));

	auto chaperoneIt = obj.find("chaperone");
	if (chaperoneIt != obj.end())
	{
		auto chaperone = Typed<picojson::object>(chaperoneIt->second, "chaperone");
		candidate.chaperone.autoApply = Typed<bool>(Required(chaperone, "auto_apply"), "auto_apply");
		LoadFloatArray(Required(chaperone, "play_space_size"), candidate.chaperone.playSpaceSize.v, 2);
		if (candidate.chaperone.playSpaceSize.v[0] < 0 || candidate.chaperone.playSpaceSize.v[1] < 0)
			throw std::runtime_error("negative play space size");
		LoadFloatArray(Required(chaperone, "standing_center"), &candidate.chaperone.standingCenter.m[0][0], 12);
		auto geometry = Typed<picojson::array>(Required(chaperone, "geometry"), "geometry");
		constexpr size_t floatsPerQuad = 12;
		constexpr size_t maxQuads = 4096;
		if (geometry.size() % floatsPerQuad != 0 || geometry.size() / floatsPerQuad > maxQuads)
			throw std::runtime_error("chaperone geometry must contain complete quads within the size limit");
		candidate.chaperone.geometry.resize(geometry.size() / floatsPerQuad);
		// Address each actual vertex array: do not walk across HmdQuad_t objects
		// through a float pointer or assume padding/layout beyond OpenVR's API.
		for (size_t quad = 0; quad < candidate.chaperone.geometry.size(); ++quad)
			for (size_t corner = 0; corner < 4; ++corner)
				for (size_t axis = 0; axis < 3; ++axis)
				{
					double coordinate = FiniteNumber(geometry[quad * 12 + corner * 3 + axis], "geometry coordinate");
					if (std::abs(coordinate) > (std::numeric_limits<float>::max)())
						throw std::runtime_error("geometry coordinate exceeds float range");
					candidate.chaperone.geometry[quad].vCorners[corner].v[axis] = static_cast<float>(coordinate);
				}
		candidate.chaperone.valid = true;
	}
	candidate.validProfile = true;
	ctx = std::move(candidate);
}

static void WriteProfile(CalibrationContext &ctx, std::ostream &out)
{
	if (!ctx.validProfile)
		return;

	picojson::object profile;
	profile["target_tracking_system"].set<std::string>(ctx.targetTrackingSystem);
	profile["hmd_serial"].set<std::string>(ctx.hmdSerial);
	profile["tracker_serial"].set<std::string>(ctx.trackerSerial);
	profile["roll"].set<double>(ctx.calibratedRotation(0));
	profile["yaw"].set<double>(ctx.calibratedRotation(1));
	profile["pitch"].set<double>(ctx.calibratedRotation(2));
	profile["x"].set<double>(ctx.calibratedTranslation(0));
	profile["y"].set<double>(ctx.calibratedTranslation(1));
	profile["z"].set<double>(ctx.calibratedTranslation(2));
	profile["scale"].set<double>(ctx.calibratedScale);
	profile["targetModelScale"].set<double>(ctx.targetModelScale);
	profile["hmdScale"].set<double>(ctx.hmdScale);

	profile["fallbackSlam"].set<bool>(ctx.fallbackToSlam);
	profile["eAngVel"].set<bool>(ctx.enableAngularVelocity);
	profile["continuousSync"].set<bool>(ctx.continuousSync);
	profile["followSlam"].set<bool>(ctx.followSlamHmd);
	profile["noHeadTracker"].set<bool>(ctx.noHeadTracker);
	profile["hideHeadTracker"].set<bool>(ctx.hideHeadTracker);
	double uiScale = ctx.uiScale;
	profile["uiScale"].set<double>(uiScale);

	double time = ctx.predictionTime;
	profile["predictionTime"].set<double>(time);

	profile["headFilterEnabled"].set<bool>(ctx.headFilterEnabled);

	auto saveOneEuro = [](const protocol::OneEuroParams &p) {
		picojson::object o;
		o["minCutoff"].set<double>(p.minCutoff);
		o["beta"].set<double>(p.beta);
		o["dCutoff"].set<double>(p.dCutoff);
		return o;
	};
	profile["headFilter"].set<picojson::object>(saveOneEuro(ctx.headFilterParams));
	profile["driftFilter"].set<picojson::object>(saveOneEuro(ctx.driftFilterParams));

	if (ctx.validRelativeOffset)
	{
		profile["rel_qw"].set<double>(ctx.relativeRotation.w);
		profile["rel_qx"].set<double>(ctx.relativeRotation.x);
		profile["rel_qy"].set<double>(ctx.relativeRotation.y);
		profile["rel_qz"].set<double>(ctx.relativeRotation.z);
		profile["rel_tx"].set<double>(ctx.relativeTranslation.v[0]);
		profile["rel_ty"].set<double>(ctx.relativeTranslation.v[1]);
		profile["rel_tz"].set<double>(ctx.relativeTranslation.v[2]);
	}

	double speed = (int) ctx.calibrationSpeed;
	profile["calibration_speed"].set<double>(speed);

	if (ctx.chaperone.valid)
	{
		picojson::object chaperone;
		chaperone["auto_apply"].set<bool>(ctx.chaperone.autoApply);
		chaperone["play_space_size"].set<picojson::array>(FloatArray(ctx.chaperone.playSpaceSize.v, 2));

		chaperone["standing_center"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		));

		chaperone["geometry"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.geometry.data(),
			sizeof(ctx.chaperone.geometry[0]) / sizeof(float) * ctx.chaperone.geometry.size()
		));

		profile["chaperone"].set<picojson::object>(chaperone);
	}

	picojson::value profileV;
	profileV.set<picojson::object>(profile);

	picojson::array profiles;
	profiles.push_back(profileV);

	picojson::value profilesV;
	profilesV.set<picojson::array>(profiles);

	out << profilesV.serialize(true);
}

static void LogRegistryResult(LSTATUS result)
{
	char *message;
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, 0, result, LANG_USER_DEFAULT, (LPSTR)&message, 0, NULL);
	std::cerr << "Opening registry key: " << message << std::endl;
}

static const char *RegistryKey = "Software\\SpaceSync";
// Old OpenVR-SpaceOverride profiles are read as fallback so nobody has to recalibrate.
static const char *LegacyRegistryKey = "Software\\OpenVR-SpaceOverride";

static std::string ReadRegistryValue(const char *key)
{
	DWORD size = 0;
	auto result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, key, "Config", RRF_RT_REG_SZ, 0, 0, &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return "";
	}

	std::string str;
	str.resize(size);

	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, key, "Config", RRF_RT_REG_SZ, 0, &str[0], &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return "";
	}

	str.resize(size - 1);
	return str;
}

static std::string ReadRegistryKey()
{
	std::string str = ReadRegistryValue(RegistryKey);
	if (str.empty())
		str = ReadRegistryValue(LegacyRegistryKey);
	return str;
}

static bool WriteRegistryKey(const std::string &str)
{
	HKEY hkey;
	auto result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0, KEY_ALL_ACCESS, 0, &hkey, 0);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return false;
	}

	DWORD size = str.size() + 1;

	result = RegSetValueExA(hkey, "Config", 0, REG_SZ, reinterpret_cast<const BYTE*>(str.c_str()), size);
	if (result != ERROR_SUCCESS)
		LogRegistryResult(result);

	RegCloseKey(hkey);
	return result == ERROR_SUCCESS;
}

void LoadProfile(CalibrationContext &ctx)
{
	auto str = ReadRegistryKey();
	if (str == "")
	{
		std::cout << "Profile is empty" << std::endl;
		ctx.Clear();
		return;
	}

	try
	{
		std::stringstream io(str);
		ParseProfile(ctx, io);
		std::cout << "Loaded profile" << std::endl;
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Error loading profile: " << e.what() << std::endl;
	}
}

bool SaveProfile(CalibrationContext &ctx)
{
	std::cout << "Saving profile to registry" << std::endl;

	std::stringstream io;
	WriteProfile(ctx, io);
	return WriteRegistryKey(io.str());
}
