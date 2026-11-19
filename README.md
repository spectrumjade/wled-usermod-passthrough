# WLED Passthrough Usermod

Lets a WLED controller sit inline behind another LED controller. In passthrough mode WLED reads the upstream
controller's WS2812-style data on an input GPIO, decodes it and re-sends it through WLED's LED output, re-timed and
at full signal level. When passthrough is off, the input is ignored and WLED runs its own effects.

**Input**: WS281x-style RGB data only (WS2812/WS2812B, WS2811, WS2813/WS2815, SK6812 RGB): 3 bytes per pixel,
one data line, idle low. RGBW input (4 bytes per pixel, e.g. SK6812 RGBW), 16-bit chips (e.g. UCS8903), inverted
signals (e.g. TM1829) and clocked protocols (APA102, WS2801) are not supported. The LED output can be any type WLED
drives; WLED converts the RGB data as usual (e.g. auto-white for RGBW strips).

**Compatibility**: classic ESP32 only, WLED 17.0 dev builds (ESP-IDF 5). On other chips the usermod compiles to a
stub that shows "unsupported" in the Info panel.

> This usermod was written with AI assistance.

## How it works

- An I2S peripheral is used as a 1-bit sampler: RX master mode, internal clock, no clock pins, data-in routed to
  the input GPIO. The line is sampled at 8 or 16 MHz into a continuous DMA ring.
- A capture task (core 0) decodes the samples: every high pulse is measured (short = 0, long = 1), bits are
  grouped into bytes, and a low period of 50 µs or more ends a frame. The decoder is a plain C++ class
  (`ws2812_decoder.h`) with host-side tests (`test/`).
- `loop()` takes the newest complete frame and writes it straight to the LED outputs while holding WLED's realtime
  lock (like E1.31/DDP), so WLED's own effects pause. Stalls in the main loop only delay output; they don't lose
  capture data.

## Build

This is a community usermod in its own repository. Add it to a classic ESP32 environment in WLED's
`platformio_override.ini` via `custom_usermods` (the default usermods are kept via `${env:esp32dev.custom_usermods}`):

```ini
[env:esp32dev_passthrough]
extends = env:esp32dev
custom_usermods = ${env:esp32dev.custom_usermods}
  https://github.com/spectrumjade/wled-usermod-passthrough.git#v1.0.0
```

For local development, replace the git URL line with `symlink:///path/to/wled-usermod-passthrough`. The URL or
symlink entry must be on its own (indented) line: WLED only recognizes it at the start of a line.
See `platformio_override.ini.sample`, which also lists the compile-time options (`PASSTHROUGH_MAX_PIXELS`,
`PASSTHROUGH_DMA_BUFFERS`).

## Wiring

```
 Upstream controller           ESP32 board
 DATA (5 V) ──[ R1 ]──┬──────── input GPIO (default 33)
                      │
                    [ R2 ]
                      │
 PSU GND ─────────────┴──────── GND
```

- **Resistor divider** from 5 V to ≤ 3.3 V, e.g. R1 = 1 kΩ, R2 = 2 kΩ. Keep it low-impedance (a few kΩ total)
  so the edges stay sharp at 800 kHz.
- **Ground**: take the common ground from the power supply side, not from the upstream controller's output terminal.
- **Input pin**: strapping pins (GPIO0, 2, 5, 12, 15) are rejected, because the upstream signal could change the
  boot mode. GPIO34–39 are input-only and fine.

## Settings (Usermods settings page, "Passthrough")

Passthrough is **off by default** and does nothing until activated: it doesn't claim the input pin, start capture
or take over the LEDs. Try it with the Info panel button first (not saved); once it works, check "enabled" so it
starts at every boot.

| Setting | Default | Description |
|---|---|---|
| enabled | off | Passthrough active after boot. The Info panel button / JSON toggle is not saved. |
| pin | 33 | Input GPIO. Allocated only while passthrough is running. |
| i2s | I2S1 | I2S peripheral used for capture, see "Peripheral conflicts". |
| sampleRate | 8 MHz | 8 or 16 MHz, see below. |
| thresholdNs | 600 | High pulses at least this long decode as 1, shorter as 0. |
| gapUs | 50 | A low period this long ends a frame. |
| inputOrder | GRB | Color order of the incoming data. Set it equal to the LED output's color order. |
| noSignalBlack | off | Go black when no frames arrive (otherwise the last frame is held). |
| noSignalMs | 1000 | Time without frames before going black. |
| applyBri | off | Scale passthrough data with WLED's brightness. Off = byte-identical output. |

**Color order**: the decoder keeps the wire bytes. If inputOrder equals the LED output's color order, the output
bytes equal the input bytes, whatever their meaning.

**Sample rate**: at 8 MHz a pulse is measured in 125 ns steps. That separates 0 and 1 bits well for typical
controllers (e.g. 0 ≈ 250–375 ns, 1 ≈ 875–1000 ns), needs less CPU time and gives the capture task twice the
buffer time. A sender running at the edges of the WS2812 tolerance (0 up to 550 ns, 1 from 650 ns) needs 16 MHz.
The "PT bit widths" row shows your controller's actual timing.

## Behavior while active

- **Faithful output**: the input fills LED output 1, then output 2, and so on, in the order listed in LED
  settings (their start indexes are ignored).
  Segments, 2D/matrix layout, LED maps, gamma, white balance and the brightness slider/multiplier have no effect
  (unless applyBri is on); they apply again when passthrough is turned off. Extra input pixels are ignored,
  missing ones are black. The Info panel's live preview doesn't show passthrough data.
- **Still applied**: the current limiter (ABL), and reversed/skipped LEDs on an output.
- **Relay**: WLED's strip brightness is kept at full, so the relay stays closed even when WLED is "off". The
  upstream controller is the real on/off switch.
- **Live override** (UI): WLED's effects are shown at WLED's brightness; passthrough keeps decoding and resumes
  when override ends. In 17.0 dev builds, overridden effects look washed out while "Disable realtime gamma
  correction" (Sync settings) is on; turning it off doesn't affect passthrough output.
- Other realtime sources (E1.31, DDP, Art-Net, …) compete for the realtime lock; don't use them at the same time.

## JSON API

```json
{"passthrough": {"on": true}}        // on/off; "t" toggles
{"passthrough": {"reset": true}}     // reset the diagnostics counters
```

## Diagnostics (Info panel)

| Item | Meaning |
|---|---|
| Passthrough | On/off toggle, and the reason if capture couldn't start. |
| PT input | Decoded frames per second and pixels in the last frame. |
| PT bit widths | Measured width range of 0 and 1 pulses. Both ranges must stay clear of the threshold. |
| PT stats | Dropped frames (malformed pulse or partial byte) and overruns (capture fell behind, e.g. during flash writes; it resynchronizes at the next frame gap). |

Frames with malformed pulses or a partial byte are dropped; the last good frame stays on the LEDs.

## Peripheral conflicts

| Port | Also used by | Detection |
|---|---|---|
| I2S0 | AudioReactive | Capture won't start if AudioReactive is compiled in. |
| I2S1 | Parallel I2S LED driver; HUB75 output | Checked at start and once per second; capture stops with an error if the LED configuration uses I2S1. |

With LED outputs on RMT (the default for a few outputs), I2S1 is free. WLED's parallel I2S driver programs the
I2S1 registers directly, so the usermod checks the LED configuration instead of relying on the ESP-IDF driver.

## Memory

While capturing: about 32 KB DMA buffers, 3 × 3 KB frame buffers, a 4 KB read buffer and a 4 KB task stack.
Nothing is allocated while passthrough is off.

## Unit tests (host)

```sh
c++ -std=c++17 -O2 -Wall -Wextra -o /tmp/ws2812_decoder_test test/decoder_test.cpp && /tmp/ws2812_decoder_test
```

The tests synthesize WS2812 waveforms with timing jitter, random gaps, mid-frame starts, noise, truncated and
oversize frames, split into random chunk sizes, and check that the recovered frames match byte for byte.

## License

EUPL-1.2, see `LICENSE`.
