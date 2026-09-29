# OpenVR headers

`openvr_driver.h` and `openvr.h` are unmodified copies from Valve's OpenVR SDK
(<https://github.com/ValveSoftware/openvr>, `headers/`), fetched on 2026-09-29.

| File | git blob sha1 |
|---|---|
| `openvr_driver.h` | `fcd675583622df0ea1a54d4f88c2b34c195c199b` |
| `openvr.h` | `7b25ab4e2c0048db92458ef5f6dcf03929cb809d` |

Copyright (c) 2015, Valve Corporation. Licensed under the BSD 3-Clause license; its text is in
[LICENSE](LICENSE) in this folder.

To update: replace both files with the same revision of Valve's repository, record the new hashes
here, and run `test.cmd` and `build.cmd`. The tools check at start that the running SteamVR offers
the interface versions these headers name, and stop if it does not.
