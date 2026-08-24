# Notice of modification

OpenVR-SpaceSync is a modified version of OpenVR-SpaceOverride by Nyabsi
(https://github.com/Nyabsi/OpenVR-SpaceOverride). Modifications to the original work were made
beginning on 2026-08-23 by Shinyflvres. The original project is licensed under the GNU Affero
General Public License v3.0 (AGPL-3.0-only), and so is this one. Nothing about the license changed.

- Original work: OpenVR-SpaceOverride, Copyright (C) 2026 Nyabsi
- Modified by: Shinyflvres (https://github.com/shinyflvre/OpenVR-SpaceSync)
- First modified release: 2026-08-23, work continues since then
- License: AGPL-3.0-only, see LICENSE

OpenVR-SpaceOverride itself is based on OpenVR-SpaceCalibrator by pushrax (MIT, see LICENSE.MIT).

## What I changed

Starting point was OpenVR-SpaceOverride commit `6604e42`. Since then:

- Driver: the head tracker only counts while it reports `TrackingResult_Running_OK`, the drift
  estimate only learns from clean frames and keeps its rotation and translation consistent,
  head motion is weighted down while measuring, a leftover latency between headset and tracker
  is learned on the fly, a couple of races and missing bounds checks got fixed, and the driver
  now logs tracker state changes and drift jumps.
- New "Follow SLAM HMD" mode: the headset keeps its own SLAM pose and the lighthouse devices
  (head tracker included) get re-aligned to it every frame.
- Overlay: one transform message per device instead of disable/enable, a new UI (Theme.cpp and
  UserInterface.cpp, with Manrope and JetBrains Mono embedded), a UI scale setting, a calibration
  wizard, descriptions under every setting, and cancel/result handling for the calibration.
- Rebrand to SpaceSync: exe, driver, IPC pipe, registry key (old OpenVR-SpaceOverride profiles are
  still picked up), installer (removes an old OpenVR-SpaceOverride install), build.bat, icon, README.

Every source file I touched or added has a "Modified by" or "Added by" line under its SPDX header.
The full history is in this repo's git log.
