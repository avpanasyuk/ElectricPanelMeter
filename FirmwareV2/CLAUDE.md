# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

ESP8266 (NodeMCU) firmware for a home Electric Panel Meter. The board uses two 74HC4051 analog multiplexers to time-share the single ESP8266 ADC across many current sensors plus one voltage pickup, computes per-port real power, and exposes the values over HTTP/Wi-Fi.

This directory (`FirmwareV2`) is one piece of a larger project tree (`../MATLAB`, `../Altium`, etc.). The MATLAB scripts in `../MATLAB/` and `../conf_sub_v2.m` consume what this firmware serves.

## Build, upload, monitor (PlatformIO)

```
pio run -e main                  # build main-panel firmware (NAME=EPM_main, VERSION=1, CONF_VERSION=0)
pio run -e sub                   # build sub-panel  firmware (NAME=EPM_sub,  VERSION=2, CONF_VERSION=2)
pio run -e main -t upload        # flash main-panel over USB on COM13 @ 921600
pio run -e espota_main -t upload # OTA upload to EPM_main
pio run -e espota_sub  -t upload # OTA upload to EPM_sub
pio device monitor               # serial @ 115200, esp8266_exception_decoder filter on
```

Toolchain extras worth knowing:
- Two parallel envs: `main` and `sub`. Each owns its `netname` (`EPM_main`/`EPM_sub`), `VERSION` (1/2 → hardware variant include), and `CONF_VERSION` (0/2 → filename digit in the BSD push). `default_envs = main`; set `default_envs = main, sub` to build both with one `pio run`.
- `espota_main` and `espota_sub` extend the respective variant env and only swap upload protocol/port.
- Filesystem is `littlefs` (Wi-Fi SSID/password are persisted to `/net_auth.txt` by `StaticWiFi_Conn`, not via WiFiManager autoConnect).
- Common build flags (`[common]`): `-std=gnu++17 -DDO_OTA=1 -DNO_STL=1 -DAVP_RAM_ATTR=IRAM_ATTR -mtext-section-literals`. `NO_STL=1` is set by the submodule libraries — do not pull in `<vector>`, `<string>` etc.; use `String` and the in-repo containers (`avp::Vector`, `CircBuffer`, …).
- `-DNAME=...`, `-DVERSION=...`, `-DCONF_VERSION=...` are defined per-variant env, not in `[common]`.
- `monitor_dtr=0 monitor_rts=0` in `[env:nodemcu]` — without these, `pio device monitor` toggles DTR/RTS on the nodemcu reset circuit and the chip looks like it's bootlooping.

There is no test runner wired up (`test/` is the default PlatformIO placeholder) and `.travis.yml` is entirely commented out.

## Hardware variants — V1 vs V2

`src/main_v2.cpp` is shared. At compile time it `#include`s either `src/V1.incl` or `src/V2.incl` based on the `VERSION` macro from `platformio.ini`:

- **V1 = main panel.** 14 channels, asymmetric wiring described by a `SwitchInput[]` table; one of the two muxes must be parked on an "open" port whenever the other is reading.
- **V2 = subpanel.** 16 channels (2 × 8), one mux active at a time via `Disable74HC4051_port[]`. Port 0 is the voltage pickup, last port is GND.

Each `.incl` defines `NUM_ports`, `ControlLines[][]`, `SetPins()`, and `ConnectPort()`. To switch hardware target, build `pio run -e main` vs `pio run -e sub`; do not duplicate the main file.

## Power computation

`loop()` alternates reads between the current port and the voltage pickup, accumulating `Power += V*I`, `Voltage += V`, `Current += I`, `NumSamples++` per port. `samples2string()` returns

```
P = (ΣV·I − ΣV·ΣI / N) / N
```

which is the real power with the DC bias subtracted (each ADC channel has its own offset, so plain ΣV·I would be wrong). After a read the accumulators are reset and `NumScans` advances.

Sampling shares the CPU with Wi-Fi via `WIFI_timeRatio`: in every group of `1+WIFI_timeRatio` 60 Hz periods the firmware samples for one and calls `a->call_in_loop()` (WebServer + OTA housekeeping) for the rest. Sampling is also suppressed while `OTA_IsInProgress`.

## HTTP API

Registered in `setup()`:
- `GET /read` — column of per-port power values, resets accumulators.
- `GET /scan` — same as `/read` but also reports `NumScans`.
- `GET /port?i=N` — raw `analogRead` of port N (debug helper).
- `GET /` and OTA endpoints are added by `avp::StaticWebServer` itself (usage page, log viewer, firmware update).

`main_v2.cpp` uses `avp::StaticWebServer` (via `#include "C_ESP/StaticWebServer.hpp"`) and aliases it locally as `WebSrv` because `Server` collides with ESP8266 core's global type name.

## Push to bsd (since v4.00)

In addition to serving `/read` for pull, the device PUSHes a CSV row every 5 s to `http://bsd:8000/`. The filename matches the historical `readout_*.py` output naming so files appear in the same place with the same rotation:

```
PowerMonitor.v<CONF_VERSION>.<MM>.<YY>.<main|sub>.csv
```

- `CONF_VERSION` is `0` for main, `2` for sub (from `-DCONF_VERSION=...` in the env).
- `<MM>.<YY>` is the current local month/year — the filename rolls over monthly automatically. Requires NTP, which is started in `setup()` via `configTime(NTP_TZ, "pool.ntp.org")`; default `NTP_TZ` is `EST5EDT,M3.2.0/2,M11.1.0/2`. Push skips a tick if the clock hasn't synced yet.
- `<main|sub>` is derived from `VERSION` (1 → "main", else "sub").
- The push uses `avp::RemoteLog::postf(filename, ...)` from C_ESP/PLUG, which calls `HTTP_POST_puts` (lives in `C_ESP/client.cpp`, already pulled in by `build_src_filter = +<*>`).
- bsd's `http_server.py` prepends a server-side timestamp and appends to `/mnt/T/<filename>` (open mode `'a'`, so restarts don't clobber).
- `/read` and `/scan` still work and still reset accumulators; whichever consumer (push or a pulled `/read`) fires first gets the full interval, the other gets a near-empty one. After the legacy `power_monitor` service is retired on bsd, push owns the data path cleanly.

## Submodule libraries (`src/C_*`)

Three Git submodules vendored under `src/` (URLs in the parent repo's `.gitmodules`):

| Path | Repo | Purpose |
| --- | --- | --- |
| `src/C_General/` | `avpanasyuk/C_General` | Platform-agnostic C++ utilities: `Macros.h`, `MyMath.hpp`, `MyTime.hpp` (`TimePeriod1`, etc.), `Error.hpp`, `CircBuffer.hpp`, `CommandParser.hpp`, `Vector.hpp`, … |
| `src/C_ESP/` | `avpanasyuk/C_ESP` | ESP8266/ESP32 Wi-Fi+Web stack: `StaticWiFi_Conn`, `StaticWebServer`, `HTML_Log`, OTA wrappers, `fast_gpio`, `hw_timer`. |
| `src/C_ARDUINO/` | `avpanasyuk/C_ARDUINO` | Arduino-only helpers (`avp::Print`, urlencode/decode, printf-to-String). |

Everything is in the `avp::` namespace. Includes use the directory prefix (`#include "C_General/MyMath.hpp"`, `#include "C_ESP/StaticWebServer.hpp"`), so `src/` is on the include path. The `.clang-format` at the repo root is a symlink into `src/C_General/` — formatting rules come from the library.

When working in these directories you are inside another Git repo (submodule). Don't commit firmware-only changes there; if a fix belongs in the library, push it upstream and then bump the submodule pointer from this repo.

## Companion off-device tooling

- `src/C_ESP/csv_logging_server.py` — minimal Python HTTP server (`POST` to `:8000`) that the firmware can push CSV log lines to; writes per-name files under `/tmp/logs/`.
- `../MATLAB/show1.m`, `../conf_sub_v2.m`, etc. — MATLAB analysis of collected data. Helpful as the ground truth for what each port number means physically.
- `out_of_build/main_v2_phase.cpp` — historical variant that also estimated phase. Kept for reference; not compiled (excluded by directory). The author's commit log notes phase error was <2 % so it was abandoned.

## Project conventions

- LED on `D8` is the status indicator (`avp::WebServer::BlinkerFunc<LED_PIN>`); don't repurpose it.
- `debug_puts()` is the single sink for log output — it forwards to the web log buffer and (only when `DEBUG` is defined) to `Serial`. Prefer it over `Serial.print` so messages survive in the `/log` page after a Wi-Fi-only session.
- Anything touched from an ISR or hot loop should respect `AVP_RAM_ATTR` (`= IRAM_ATTR` here) — flash-resident code can't run during SPI flash cache misses.
- Bump `Opts.Version` in `setup()` when you ship a behavior change; it appears on the device's root page and is the easiest way to confirm an OTA actually took.
