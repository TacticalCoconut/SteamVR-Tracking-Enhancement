# How SteamVR Tracking Enhancement works

This page explains the mechanisms in enough detail to judge them, tune them, or change them. File
names refer to `driver/src/` unless stated otherwise.

## Where the driver sits

SteamVR loads tracking drivers as DLLs into its server process, `vrserver.exe`. Each driver hands
poses to SteamVR by calling `IVRServerDriverHost::TrackedDevicePoseUpdated(device, pose, size)`.

The driver adds no devices of its own. After every driver has initialised (on the first
`RunFrame`) it replaces one entry in the interface's function table — the slot for
`TrackedDevicePoseUpdated` — with its own function (`hook.cpp`). From then on every pose from every
driver passes through it:

```
lighthouse driver ──► Tracking Enhancement ──► (Space Calibrator, if installed) ──► vrserver
```

Space Calibrator patches the *code* of the same function; this driver patches the *table entry* that
points to it and calls through to whatever was there. Neither overwrites the other.

Safety measures in the hook:

- Poses are only touched when the device index and the size of the pose structure are what the
  driver was compiled for. Anything else passes through unchanged.
- The driver's DLL is pinned in memory, so that a call already on its way in when SteamVR unloads the
  driver can never land in unmapped memory.
- Twenty seconds after start the driver checks that poses really flow through the hook. If devices
  are tracking and none did, it says so in the log and in `status.json` instead of pretending to work.

## The pose pipeline (`filter.cpp`)

Each device has its own state, protected by its own lock. The pipeline allocates nothing and does no
file or network access.

### 1. World correction

See [below](#the-world-correction). It changes the pose's world-from-driver transform only; the
remaining stages work in driver space and are unaffected by it.

### 2. The glitch gate

The driver keeps a velocity estimate for each device (a low-pass with a 20 ms time constant over
accepted samples) and predicts the next position from it. The prediction can be wrong by at most

```
allowed = jump_m + max_accel × (0.02 × dt + dt² / 2)
```

for any motion whose acceleration stays under `max_accel` — the first term in the bracket is the lag
of the velocity estimate, the second the acceleration that was not modelled. At the usual update rate
of about 1 kHz this is essentially `jump_m` (3 cm); after a radio stall of 30 ms it widens, which is
why stalls during fast swings do not produce false alarms. Rotation has the same test, with the
device's own angular velocity as the allowance.

A pose outside the allowance makes the device **suspect**. While suspect:

- The output holds the last good position, coasting on the last velocity, which decays with a 40 ms
  time constant. The travel of a hold is therefore bounded by `speed × 0.04 s`.
- Orientation keeps following the device if its rotation is plausible.
- Every new sample is compared with the old path and with the previous suspect sample.

Two outcomes:

| | Condition | Result |
|---|---|---|
| **Glitch** | A sample is back within `jump_m` of the old path | The jump is discarded; the output eases back onto the raw pose |
| **Relocation** | Samples stay consistent with each other for `confirm_ms`, or `suspect_max_ms` runs out | The new position is real; the output rejoins it |

### 3. The dropout bridge

When a pose arrives marked as not tracking (`result` other than `Running_OK`, or `poseIsValid`
false), and the device's class has `bridge` on, the driver holds the pose as above and reports it as
valid. After `bridge_max_ms` it stops and passes SteamVR's own pose through.

The bridge is off for headsets and cannot be relied on to hide head tracking loss even if switched
on: a held head pose is a safety problem.

### 4. Rejoining

After a hold, the difference between the output and the raw pose is either **eased** out over
`blend_ms` with a smooth step, while the device's real motion continues to pass through, or — when it
is larger than `blend_max_m` — **snapped**. A device that was lost and found 1.5 m away would
otherwise slide there at 10 m/s.

### 5. Jitter filter

For classes with `smooth` on (trackers by default), a One Euro filter: a low-pass whose cutoff rises
with speed, so that a resting tracker is smoothed hard and a moving one hardly at all. In the tests
it reduces 1.7 mm RMS of noise to 0.2 mm and lags 2.3 mm behind at 1 m/s.

### Time

Poses carry a `poseTimeOffset`. It may be not-a-number or step backwards between samples; the
pipeline's clock is forced to move forward, because every decay in it assumes that.

## The world correction

A rigid transform `p' = R·p + T` applied to the world-from-driver transform of every lighthouse
device, base stations included. `R` is the shortest rotation that takes the measured floor normal to
"up"; `T = pivot − R·pivot` makes the pivot a fixed point.

- The same transform for every device means nothing moves relative to anything else: a controller
  that was 40 cm in front of your head still is.
- The pivot is the standing origin at the time of the first correction, so the centre of your play
  space keeps its position and height.
- A change is interpolated over `world_fix_ease_ms` (rotation by slerp, translation linearly, both
  with the same smooth step). Every intermediate state is rigid.
- Tilts above 5°, a pivot more than 100 m away, and values that are not numbers are refused, and the
  correction is switched off. A tilt that large is a fault in the setup, not a calibration error.
- A device the driver has not identified yet cannot be corrected. While a correction is in force,
  such a device is reported as still calibrating — for a few frames, two seconds at most — rather than
  shown uncorrected and then moved.

Quick Fix measures in the space applications see, which already contains the active correction. It
takes the active correction out of the measured normal before storing, so that a second fix
*replaces* the first with the total instead of stacking a remainder on top.

## The station monitor (`telemetry.cpp`)

Runs once a second on the telemetry thread, not in the pose path.

For each pair of base stations that have both been valid for `station_baseline_s`, the driver stores
their distance and relative orientation. Later it compares. A station is reported as **moved** when
more than half of its pairs changed by more than `station_move_mm` or `station_move_deg`.

Why pairs: SteamVR re-aligns the whole tracking universe while it starts, which moves every station's
reported pose at once. Against a fixed reference that looks like four stations moving. Between
stations, nothing changed.

With four stations the moved one is named and the others stay clear. With two, a change cannot be
attributed and both are reported.

## Telemetry

Written by a separate thread to `%LOCALAPPDATA%\SVREnhance\`:

| File | Content |
|---|---|
| `driver.log` | Start-up, devices found, hook state, settings changes |
| `status.json` | Live state, rewritten every second: per device and per station |
| `events.csv` | Every glitch, relocation, bridge, gap, disconnect, with time and position |
| `coverage.csv` | Where devices have been: a row per 5 cm moved and at least every 5 s |

The CSV files stop growing at 20 MB each.

## Quick Fix (`tools/fix/`)

A SteamVR dashboard overlay. Its picture is drawn with GDI and handed over as raw pixels; laser
pointer clicks come back as mouse events. Decisions are in `fixmath.h`, free of any SteamVR code, and
are what the tests cover.

**Floor.** A controller lying flat on a correct floor shows a small height, because the point
SteamVR tracks is inside the controller, not on its underside. Quick Fix learns that height per
controller model while the floor is right. Later, the difference between what it reads and what it
learned is the floor's error. The floor is moved by shifting the standing origin through
`IVRChaperoneSetup`.

**Tilt.** Three or more samples of the same controller on the floor, spread over at least a metre
with one of them 40 cm off the line of the others, define the floor's plane by least squares. Samples
that do not lie on one plane within a centimetre are refused — one of them was not on the floor.

**The standing matrix.** SteamVR documents the matrix as "standing zero pose to raw tracking pose".
The tools do not take the direction on trust: at start they test both readings against the headset's
own raw and standing poses and stop if neither fits within 2 cm.

**Which device is measured.** Only a controller that moved at least 10 cm after the button was
pressed and then lay still for a second (within 2 mm), within 60 cm of the floor. That rules out a
controller that was resting on furniture all along. Trackers are never measured: one strapped to a
foot rests near the floor all the time. A floor error above 15 cm is applied only after confirmation.

**Talking to the driver.** Quick Fix writes the correction into SteamVR's settings
(`driver_svrenhance.world_fix_*`); the driver reads settings every two seconds. Because the
correction is several settings written one after another, it carries a revision number: the writer
makes `world_fix_rev` odd before it starts and even when it is done, and the driver applies the
values only when the revision is the same before and after reading them, even, and new. A
half-written correction is never applied. Setting `world_fix_enable` to `false` by hand is always
honoured. Quick Fix then watches `status.json` until the driver reports the correction.

**Someone else changes the room setup.** Quick Fix re-reads the room setup before every measurement.
When a room setup is saved by anything else, it forgets its own floor bookkeeping: the new floor is
the starting point.

## Room Setup (`tools/roomsetup/`)

Geometry in `tools/common/geom.h`:

| Step | Method |
|---|---|
| Floor | Least-squares plane through the samples |
| Boundary clean-up | Drop samples closer than 2 cm, then Douglas–Peucker with a 3 cm tolerance on the closed loop |
| Wall direction | Length-weighted mean of the edge directions modulo 90° |
| Snapping | Edges classified along/across the wall direction; runs merged into walls; walls under 10 cm absorbed |
| Play area | Largest axis-aligned rectangle inside the boundary, found on a 2.5 cm grid |

## Tests

`test.cmd` builds and runs three programs; none needs SteamVR.

| Program | Covers |
|---|---|
| `tests/filter_test.cpp` | The pose pipeline on simulated 1 kHz streams with injected spikes, dropouts, stalls, noise, broken values; the station monitor; the world correction |
| `tools/roomsetup/geom_test.cpp` | Plane fit, trace clean-up, snapping, play rectangle |
| `tools/fix/fixmath_test.cpp` | Floor decisions, tilt measurement and composition, sample validation |

What they cannot cover is the conversation with a live SteamVR. That part is checked by the tools
themselves at start (`--check`) and by the driver's own verification of its hook.
