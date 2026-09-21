# AGENTS.md

yoRadio fork for the **ESP32-A1S** (Ai-Thinker audio kit) with the **ES8388 codec**. Internet radio firmware built on PlatformIO (Arduino framework), forked from `e2002/yoradio`. ESP8266 support is removed; ES8388 + Vorbis/Opus decoding were added. Upstream README/Wiki apply, but this repo's build flow is PlatformIO, **not** the Arduino IDE.

## Build & flash

All commands run from `yoRadio/` (the PlatformIO project root, where `platformio.ini` lives).

```bash
pio run                          # build default env (Yoradio_RELEASE_serialPort)
pio run -e Yoradio_RELEASE_serialPort -t upload     # flash firmware via serial (/dev/ttyUSB0)
pio run -e Yoradio_RELEASE_serialPort -t uploadfs   # upload data/ as SPIFFS image
pio run -e Yoradio_RELEASE_OTA -t upload        # flash via OTA (espota to 10.11.12.13)
pio device monitor               # serial monitor (115200, esp32_exception_decoder filter)
```

- Envs: `Yoradio_RELEASE_serialPort` (default), `Yoradio_RELEASE_OTA`, `Yoradio_JLINK_debug` (adds `-D FREE_JTAG_PINS`, reserves pins 12-15).
- Board is defined in `yoRadio/boards/esp32-a1s.json` (custom: `BOARD_HAS_PSRAM`, `-mfix-esp32-psram-cache-issue`). Partition table is `partition_1.5Mapp_OTA_0.9Mfs.csv` (1.5MB app OTA x2 + 0.9MB SPIFFS). Filesystem is SPIFFS.
- Build artifacts go to `yoRadio/_BUILD/` and `yoRadio/_LIBDEPS/`. **There is no root `.gitignore`** — these are untracked but not ignored; never `git add` them (100MB+ of artifacts).

## Source layout

- `yoRadio/src/main.cpp` — real entrypoint (`yoRadio.ino` is a stub with just a logo).
- `yoRadio/src/core/` — application: `player`, `display`, `network`, `netserver`, `config`, `controls`, `mqtt`, `telnet`, `sdmanager`, `touchscreen`, `rtcsupport`. `config.cpp` is the settings store; `common.h` holds shared request/event structs used by plugins.
- `yoRadio/src/audioI2S/` — vendored ESP32-audioI2S (incl. aac/flac/mp3/opus/vorbis decoders, PSRAM-aware); `audioES8388/` — ES8388 codec driver (fork-specific); `audioVS1053/` — VS1053 support (optional).
- `yoRadio/src/displays/` — one `displayXXX.cpp/.h` per display model, selected by `DSP_MODEL`. Widget layout configs live in `displays/conf/display_XXX_conf.h`; rendering in `displays/widgets/`; helpers (`utf8RusGFX.h`, `l10n.h`) in `displays/tools/`.
- `yoRadio/src/AsyncWebServer/`, `async-mqtt-client/`, `OneButton/`, `IRremoteESP8266/` — vendored libraries (do not install via Library Manager; they're in-tree).
- Plugins: new system in `src/pluginsManager/` (see its `README.md` for the `Plugin` hook API) + user plugins in `src/plugins/<MyPlugin>/`. `examples/plugins/` are samples (legacy `.ino` ones are deprecated).

## Configuration — critical rules

- **Never edit `yoRadio/src/core/options.h`** — it says so itself; it's overwritten on updates. It contains all `#ifndef` defaults.
- User config lives in `yoRadio/myoptions.h`, auto-included via `__has_include("../../myoptions.h")` from `options.h`. This repo's committed `myoptions.h` is already configured for the A1S/ES8388 (I2S_DOUT 26, BCLK 27, LRC 25, MCLK 0, ES8388 SCL 32 / SDA 33, MUTE_PIN 21).
- `examples/myoptions.h` is the annotated master template for every option; `examples/mytheme.h` for colors; `examples/mqttoptions.h` (copy to `yoRadio/mqttoptions.h` to enable MQTT).
- Select audio path by pin assignments: I2S DAC vs VS1053 — one must be `255`/disabled.
- Version string is `YOVERSION` in `options.h` (currently `"0.9.434_A1S"`). Drop a `test.h` next to `yoRadio.ino` to override it.

## Web UI (SPIFFS data) gotchas

- Served from `yoRadio/data/www/`. **JS/CSS are committed pre-gzipped** (`script.js.gz`, `style.css.gz`, …) — there are no plain `.js`/`.css` sources in the repo. AsyncWebServer transparently serves `path.gz` with `Content-Encoding: gzip` when the plain file is absent, so to edit JS/CSS: create/edit the plain file, `gzip` it, and commit the `.gz` (keep the name without `.gz` in the HTML references).
- `index.html` etc. use a `%VERSION%` query placeholder that the server replaces with `YOVERSION` (forces cache busting). `serveStatic` sends `Cache-Control: max-age=31536000` — after firmware+fs updates, browsers need a hard refresh (Ctrl+F5), as the README stresses.
- `yoRadio/data/data/` contains runtime user data (`wifi.csv`, `playlist.csv`, `index.dat`). It has a self-referential `.gitignore` (`*` + `!.gitignore`) so user data is never committed — leave that dir alone; don't add sample CSVs there.
- The device's own OTA/update flow: web UI at `/update`, or `pio run -t uploadfsota` / espota. README "Update" section documents the manual SPIFFS+firmware update; use it when changing `data/`.

## Repo-specific conventions & traps

- Localization: `yoRadio/locale/displayL10n_{en,ru}.h` selected by `L10N_LANGUAGE`.
- `yoRadio/fonts/glcdfont.c` replaces Adafruit_GFX's `glcdfont.c` (custom icons). When adding glyphs/icon assets, keep both font files consistent.
- `broken204/` holds fixed copies of Adafruit_SSD1327/SH110X for the old esp32 2.0.4 core — historical, not part of the current build; don't "clean it up" without checking.
- `HA/custom_components/yoradio/` is the Home Assistant integration (async, MQTT-based); `nextion/` is the Nextion HMI project.
- No test suite, no CI, no linter config in this repo. Verification = `pio run` compiling cleanly + flash to hardware. The `CORE_DEBUG_LEVEL=4` build flag produces verbose serial logging (telnet `##…` prefixed messages), which is the main debugging surface.