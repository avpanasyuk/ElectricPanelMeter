# ElectricPanelMeter

A whole-panel electric power meter: custom measurement hardware plus ESP8266
firmware that reports voltage / current / power over the LAN and logs it, with
MATLAB and Python tooling for readout and analysis.

## Layout

- **Hardware** — `Altium/` (PCB), `CAD/`, `Drawings/`, `LTspace/` (LTspice
  schematics) for the measurement boards.
- **`FirmwareV2/`** — current [PlatformIO](https://platformio.org) ESP8266
  firmware (`board = nodemcu`, 4 MB flash, LittleFS). One source builds two
  variants for the main and sub units:
  - `pio run -e main` → device `EPM_main` (`-DVERSION=1`)
  - `pio run -e sub`  → device `EPM_sub`  (`-DVERSION=2`)
  It vendors the shared `C_General` / `C_ESP` / `C_ARDUINO` submodules and uses
  `avp::StaticWebServer` (web UI / config / OTA) plus `avp::RemoteLog`, which
  POSTs `<filename>,<csv>` rows to the home server's `http_server.py` log sink.
  See `FirmwareV2/CLAUDE.md` for build and agent notes.
- **`Firmware/`** — the original (V1) firmware, kept for reference.
- **`MATLAB/`** — analysis code; vendors `github.com/avpanasyuk/MATLAB` at
  `MATLAB/AVP_LIB`. Logging is push-based: since firmware v4.00 the device POSTs
  its CSV rows straight to the home server's `http_server.py`, so no host-side
  polling script is needed.
- **`conf_*.m`, `*.csv`, `*.xls`** — configuration and recorded data.

## Building the firmware

```
cd FirmwareV2
pio run -e main        # main unit  (EPM_main)
pio run -e sub         # sub unit   (EPM_sub)
pio run -e <env> -t upload
```

The firmware libraries are git submodules — clone the repo with `--recursive`
(or run `git submodule update --init --recursive` after a plain clone).
