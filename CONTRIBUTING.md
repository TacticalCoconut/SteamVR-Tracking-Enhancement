# Contributing

Thank you for wanting to help. The most valuable contributions, in order:

1. **Reports from real setups** — especially "it did not work on my SteamVR version" and "this
   threshold is wrong for my hardware", with logs.
2. **First-run reports for Quick Fix and Room Setup.** Both are new and have been tested only on
   simulated data. If you try one, say how it went, even if it simply worked.
3. **Code.**

## Reporting a problem

Use the issue templates. Attach `driver.log` and the text output of
`python tools\svrenhance.py report`. Both contain your devices' serial numbers; remove them if you
prefer.

## Building and testing

You need Build Tools for Visual Studio 2019 or newer with the *Desktop development with C++* workload.

```bat
test.cmd     :: builds and runs every test; no SteamVR needed
build.cmd    :: builds the driver and the tools into dist\
```

Both compile with `/W4 /WX`: a warning fails the build.

## Rules for changes

- **Nothing about SteamVR is assumed.** Interface layouts, structure fields, file formats and
  directions of transforms are read from `third_party/openvr/*.h`, observed in SteamVR's own files, or
  tested at run time. If you cannot verify something, make the code check it and stop when it is wrong.
- **The pose path stays cheap.** No allocation, file access, logging or blocking call inside
  `ProcessPose` and what it calls. Locks are held for a few copies only.
- **Fail towards doing nothing.** Any value the driver does not understand — a wrong structure size,
  a not-a-number, a device of unknown class — means the pose passes through untouched.
- **The headset is special.** No holding, no smoothing, no bridging of head poses by default.
  Changes that move the user's view are eased and bounded.
- **Every behaviour has a test.** A change to the pipeline comes with a test on a simulated stream in
  `tests/filter_test.cpp`; a change to a tool's decisions with one in its `*_test.cpp`.
- **Report honestly.** The README's status table says what has been tried in a headset and what has
  not. Keep it true.
- **Never start an OpenVR client from an automated or sandboxed environment** against someone's
  running SteamVR. It can stop SteamVR's connection service for that session.

## Style

Match the surrounding code: tabs, braces on their own line, comments that say *why*. C++17, no
dependencies beyond the Windows SDK and Valve's headers. Python: standard library only.

## Pull requests

- One topic per pull request.
- `test.cmd` and `build.cmd` pass.
- Say what you tested and how — simulated only, or in a headset, and with what hardware.

## License

By contributing you agree that your contribution is licensed under the
[GNU General Public License v3.0](LICENSE).
