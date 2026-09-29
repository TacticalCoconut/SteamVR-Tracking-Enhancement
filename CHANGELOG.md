# Changelog

All notable changes to SteamVR Tracking Enhancement.

## 0.2.0

The project is named **SteamVR Tracking Enhancement** (working name until now: SVREnhance). File
names, the settings section `driver_svrenhance` and the folder `%LOCALAPPDATA%\SVREnhance` keep the
short name, so an existing installation, its settings and its records carry over unchanged.

### Added

- **Quick Fix** (`svrenhance_fix.exe`): a SteamVR dashboard panel to report a floating, clipping or
  tilted world and get the matching fix, with Undo and Reset.
- **World correction** in the driver: levels a tilted tracking universe by rotating it rigidly about
  the centre of the play space. Eased in, limited to 5°, controlled through `world_fix_*` settings.
- **Room Setup** (`svrenhance_roomsetup.exe`): floor from a multi-point plane fit, boundary snapped to
  right-angled walls, play area as the largest rectangle that fits. Experimental.
- `install.cmd` and `uninstall.cmd`.
- `blend_max_m`: corrections larger than this (default 50 cm) snap instead of being eased in.
- The report ranks glitch hotspots by glitches per time present, over the last 12 hours.
- `status.json` reports the active world correction.

### Changed

- Base stations are judged by their geometry relative to each other, after a settling time, so that
  SteamVR re-aligning its universe at start-up is no longer reported as stations moving.
- Coverage is also recorded while a device rests (at least every 5 s), so that it reflects time
  spent in a place.
- Update rate is reported as poses per second; it was a mean of inverse intervals, which bursts
  inflated.
- Update gaps are only reported for devices that were tracking.
- The tools refuse to start unless SteamVR is already running and accepting applications; they never
  start SteamVR themselves.
- One `build.cmd` and one `test.cmd` for everything.

### Fixed

- A pose time stepping backwards during a hold could throw the held pose far away.
- A not-a-number pose time offset could put not-a-number poses out for a smoothed tracker.
- The driver's DLL is pinned, so a pose call in flight during unload cannot land in unmapped memory.
- The setting `enable` collided with SteamVR's own switch for loading a driver; it is now
  `filter_enable`.
- A dropout during a hold no longer restarts the coast from the coasted position.

## 0.1.0

- First version: pose hook, glitch gate, dropout bridge, correction easing, tracker smoothing,
  base-station monitor, telemetry, report tool.
