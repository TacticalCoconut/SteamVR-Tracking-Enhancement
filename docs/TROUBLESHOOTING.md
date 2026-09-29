# Troubleshooting

Start with two things: the driver's log at `%LOCALAPPDATA%\SVREnhance\driver.log`, and the report:

```bat
python tools\svrenhance.py report --html report.html
```

## The driver

### `driver.log` does not exist, or has no entry for today

SteamVR did not load the driver.

1. Close SteamVR and run `install.cmd` again. It refuses to run while SteamVR is open.
2. In SteamVR: *Settings → Startup / Shutdown → Manage Add-ons*. SVREnhance must be on. SteamVR
   switches add-ons off after a crash ("safe mode"); switch it back on there.
3. The driver's folder must still be where it was when you installed. If you moved it, run
   `uninstall.cmd` from the old place (or remove the entry from
   `%LOCALAPPDATA%\openvr\openvrpaths.vrpath`), then `install.cmd` from the new one.

### The log says `filtering is NOT active`

The driver installed its hook, devices are tracking, and no pose came through. The interface this
version patches is not the one your SteamVR's drivers call. Nothing is harmed; the driver only
monitors. Please open an issue with your SteamVR version (top of `vrserver.txt` in Steam's `logs`
folder).

### The log says `device N: could not read its properties`

The driver leaves a device alone when it cannot find out what it is. Usually harmless (a virtual
device from another add-on).

### A device lags behind fast movement, or "rubber-bands"

The glitch gate is treating real motion as suspect. In `events.csv`, look for `glitch_rejected` and
`relocation_eased` rows for that device at the times it happened.

- Raise `<class>_jump_m` (for example `controller_jump_m` from `0.03` to `0.05`).
- Raise `<class>_max_accel`.
- To rule the gate out altogether, set `<class>_gate` to `false`.

### A controller or tracker freezes for a moment, then catches up

A bridged dropout: the device lost optical tracking and the driver held it. Without the driver it
would have drifted or vanished instead. If you prefer that, lower `<class>_bridge_max_ms` or set
`<class>_bridge` to `false`. The report shows how often it happens and where; that is the thing to
cure.

### Trackers feel "floaty"

The jitter filter. Raise `tracker_smooth_min_cutoff` (less smoothing at rest) or
`tracker_smooth_beta` (less smoothing in motion), or set `tracker_smooth` to `false`.

### Turning everything off quickly

Set `"filter_enable": false` under `"driver_svrenhance"` in `steamvr.vrsettings`. The driver reads it
within two seconds and passes every pose through untouched.

## The tools

### "SteamVR is not running"

Start SteamVR and wait until the headset tracks. The tools never start SteamVR themselves.

### "SteamVR is running but is not accepting new applications"

SteamVR's connection service has stopped. Restart SteamVR.

### SteamVR shows "Shared IPC Namespace Unavailable (310)"

The same thing, seen from SteamVR's side: an application could not connect. Restart SteamVR.

This has been seen after one of the tools was started from inside a sandboxed program. Start the
tools from Explorer or an ordinary terminal.

### "Could not verify SteamVR's standing frame"

The tool compares SteamVR's room setup with the headset's position and they did not agree within
2 cm, so it stopped before changing anything. The headset must be tracking and must have a completed
Room Setup. Run SteamVR's Room Setup once, then try again.

### Quick Fix: "I do not know yet how high your controllers sit"

Measuring a wrong floor needs to know how high your controller model's tracked point is when the
controller lies flat. Quick Fix learns that from your own system: when your floor is right, lay a
controller on it and press *Learn controller height*. Until then, use the 1 cm buttons.

### Quick Fix: "No controller came to rest on the floor"

Press the button first, then put the controller down. A controller that was already lying there when
you pressed is ignored on purpose. It must lie still for a full second, flat, on the bare floor, and
be tracked by the base stations while it lies there.

### Quick Fix: the panel does not appear in the dashboard

Use the keys in its console window instead (`svrenhance_fix.exe --help` lists them), and please
report it.

### Quick Fix: the measurement contradicts what I reported

Trust the measurement if the controller was lying flat on the bare floor. Carpet, a rug, or a
controller resting on its ring give a different height than the one that was learned. Learn the
height again on the surface you actually use.

### After levelling, the floor is too high or low

Levelling turns the world about the centre of your play space and keeps the floor height there. Away
from the centre the floor moves — that is the correction. If the height is now wrong everywhere,
report *floating* or *clipping*.

### After levelling, devices from another tracking system are off

The correction is applied to lighthouse devices. Re-run Space Calibrator's calibration.

### Undoing everything a tool did

- Quick Fix: *Reset* (twice).
- Room Setup or Quick Fix floor changes: copy the newest `chaperone_info_<time>.vrchap` from
  `%LOCALAPPDATA%\SVREnhance\backup\` over `chaperone_info.vrchap` in Steam's `config` folder, with
  SteamVR closed. Or simply run SteamVR's Room Setup.
- The tilt correction alone: set `"world_fix_enable": false`.

## What the report's findings mean

| Finding | Meaning | What to do |
|---|---|---|
| **Reflections** | SteamVR discarded many laser hits that arrived at sensors facing away from every station | Cover or turn away mirrors, windows, screens, glass, glossy floors |
| **Glitch hotspots** | Places with the most glitches per time spent there | Stand there and look for something shiny in view of a station |
| **Sync decoding** | Devices received timing data they could not match | Usually reflections or interference; check station channels |
| **Base station channels** | Two stations share a channel | Give each a unique channel |
| **Base station stability** | One station restarted far more often than the others | Its power supply, cable or mount |
| **Base station moved** | One station changed position relative to the others | Tighten its mount, then run Room Setup |
| **Wireless link** | A device's connection to its dongle dropped repeatedly | Move the dongle: a USB extension, in view of the play space, away from USB 3 ports and Wi-Fi routers |
| **IMU saturation** | A device was swung harder than its motion sensor can measure | Nothing; SteamVR recovers optically |

One device with far more tracking losses than its twin (left against right controller) points at the
device. Swap hands for a session: if the losses follow the controller, it is the controller.

## Reporting a problem

Open an issue with:

- `driver.log`
- the text output of `python tools\svrenhance.py report`
- your SteamVR version, headset, controllers and number of base stations
- what you saw, and when (the time lets it be matched to `events.csv`)

`driver.log`, `status.json` and `events.csv` contain the serial numbers of your devices. Remove them
if you would rather not publish them.
