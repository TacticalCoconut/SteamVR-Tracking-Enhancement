# Security

## What this software does on your computer

So that you can judge it:

- The **driver** is a DLL that SteamVR loads into `vrserver.exe`. It changes one entry in an
  interface table inside that process to see and alter tracking poses. It reads SteamVR's settings
  and writes log and data files to `%LOCALAPPDATA%\SVREnhance\`. It opens no network connection,
  starts no process, and reads no file outside SteamVR's settings.
- The **tools** connect to a running SteamVR as ordinary applications. Quick Fix and Room Setup change
  your room setup through SteamVR's own interface and write SteamVR settings under
  `driver_svrenhance`. They back up the room setup first. They open no network connection.
- The **report** script reads SteamVR's logs and the driver's files and writes the HTML file you name.
- Nothing runs with administrator rights, and nothing installs a service, a scheduled task or a
  start-up entry. `svrenhance_fix.exe --register` asks SteamVR to start Quick Fix with SteamVR; it
  does that only when you run it, and `--unregister` reverses it.

## Reporting a vulnerability

Please do not open a public issue for a security problem.

Use GitHub's private reporting: on the repository page, **Security → Report a vulnerability**. Say
what you found, how to reproduce it, and which version.

You should get a first answer within a week. Fixes are released as a new version; you will be
credited unless you ask not to be.

## Supported versions

Only the latest release receives fixes.

## Downloads

Official builds are attached to the [releases](https://github.com/TacticalCoconut/SteamVR-Tracking-Enhancement/releases) of this repository
and are produced by the workflow in `.github/workflows/build.yml` from the tagged source. A DLL from
anywhere else that is loaded into SteamVR can do whatever SteamVR can; do not install one.
