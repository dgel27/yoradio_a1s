# ES8388 EQ coefficient lab (throwaway)

Reverse-engineering harness for the ES8388 **DEQ** (digital equalizer) — the
2-band parametric EQ in registers **30..37**. Not part of the radio firmware.

## Why

The chip has a shelving-filter EQ but the coefficient encoding is **not
documented**:

- Datasheet (rev 12.0, Nov 2023) only publishes the flat default
  `{5'h0f, 5'h1f, 5'h0f, 5'h1f, 5'h0f, 5'h1f}` for the 30-bit `Shelving_a` /
  `Shelving_b` words.
- User guide §10.3: *"only 2 band equalizer… it can do bass **or** treble
  operation, but can't do bass and treble at the same time"* and *"Everest
  Semiconductor will provide equalizer calculator"* — which is not in either
  PDF and not published anywhere.
- A web search turns up no reference implementation.

So we discover it **by ear**. There is a real risk the coefficients turn out to
need a specific fixed-point encoding that a blind field sweep won't reveal. If
~1 hour of sweeping produces nothing, the fallback is to request the
calculator from Everest (Shenzhen) — see "Bail-out" below.

## Register packing

From the datasheet bit assignments, the 30-bit word is split as:

| Bits | Register | Width |
|------|----------|-------|
| `a[29:24]` | reg 30, bits 5:0 | 6 |
| `a[23:16]` | reg 31 | 8 |
| `a[15:8]`  | reg 32 | 8 |
| `a[7:0]`   | reg 33 | 8 |

6+8+8+8 = 30. Filter **B** is identical in regs 34..37. Flat default for both
is `0x1F 0xF7 0xFD 0xFF`.

A "field" in the console is one of the six 5-bit groups from the datasheet
default `{0f,1f,0f,1f,0f,1f}`, numbered 0 (most significant) to 5.

## How it works

Plays a **logarithmic sine sweep 20 Hz → 20 kHz over 12 s** and, with A/B on,
alternates **candidate → 0.4 s silence → flat → 0.4 s silence**. Any change is
directly comparable within one 24 s cycle, and you localise it to a frequency
band by ear — which is what distinguishes a low shelf from a high shelf.

Suggested order: hold flat, then vary **one field at a time** across its range,
so any audible change is unambiguously attributable.

## Console (115200)

| Command | Effect |
|---|---|
| `f` | restore both filters to flat default |
| `fa <8 hex>` | set filter A raw, e.g. `fa 1FF7FDFF` |
| `fb <8 hex>` | set filter B raw |
| `s <0..5> <v>` | set one 5-bit field of filter A (`v` 0..31) |
| `g <0..5> <v>` | same for filter B |
| `d` | dump registers 29..37 |
| `A` | toggle A/B sweep on/off |
| `?` | help |

## Build / flash

This project is **outside `yoRadio/`** and shares nothing with the radio build
except the ES8388 driver, which is **symlinked** from
`../yoRadio/src/audioES8388/` so the baseline init is byte-identical to
production.

```bash
cd es8388_eq_test
pio run                 # build
pio run -t upload       # flash to /dev/ttyUSB0
pio device monitor      # 115200
```

**Flashing this overwrites the radio firmware.** When you're done, put yoRadio
back:

```bash
cd ../yoRadio && pio run -t upload && pio run -t uploadfs
```

## Bail-out

If sweeping the fields produces no repeatable, audible change, the encoding is
probably not a plain per-field gain and the blind approach has hit its limit.
The alternatives, in order of preference:

1. Contact Everest Semiconductor for the DEQ calculator.
2. Sweep two fields jointly (each has 32 values, so 1024 combos per pair) and
   listen for any tilt — slower but still feasible by ear.
3. Drop hardware EQ and keep the (now correct) software tone control.

Do **not** ship guessed coefficients into the radio firmware. Leave the EQ
plumbing out of `yoRadio/` until real coefficients are known.
