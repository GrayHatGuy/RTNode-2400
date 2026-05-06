# Release/esptool/

Place the standalone `esptool` binary here before packaging a release.

| Platform      | Filename          |
|---------------|-------------------|
| macOS / Linux | `esptool`         |
| Windows       | `esptool.exe`     |

The binary is **not** committed to this repository (it is platform-specific and
listed in `.gitignore`).  Obtain it from the official esptool releases:
<https://github.com/espressif/esptool/releases>

`flash.py` will detect and invoke the binary automatically when it is present.
No Python or `pyserial` installation is required for end users.
