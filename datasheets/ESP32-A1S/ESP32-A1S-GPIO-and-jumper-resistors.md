# ESP32-A1S: GPIO map and the resistors that gate them

Which pins the ESP32-A1S exposes, which ones this project uses, and which
resistors on the carrier board have to come off before a pin can be reused for
something else.

## Sources

| Used for | File |
|---|---|
| Module pinout (authoritative, 38 pins) | `esp32-a1s_v2.3_specification.pdf` §4 "PIN definition" |
| Carrier schematic, resistor designators | `esp32-audio-kit_v2.2_sch.pdf` |
| Extracted netlist / page images | `kicad/` |

The module and the carrier are different parts and this matters below:

- **ESP32-A1S module** — ESP32-WROVER + PSRAM + an **onboard ES8388 codec**.
- **Carrier board** — the AudioKit V2.2 PCB the module plugs into. This is where
  the 0 Ω resistors, keys, SD socket and headphone jack live.

**Board-revision warning.** The carrier schematic in this folder is
`esp32-audio-kit_v2.2_sch.pdf`. The A1S module spec is V2.3. Designators on a
V2.3 carrier are *probably* the same but are not proven by anything here.
Confirm against the schematic for the board in your hand before soldering.

---

## 1. The pins you cannot move

**GPIO 25, 26, 27, 32, 33, 35 and 0 are not free.** They are not absent from
the module by accident — the module wires them internally to its own ES8388:

| GPIO | Function | Wired inside the module to |
|---|---|---|
| 0 | I2S MCLK | ES8388 |
| 25 | I2S LRC/WS | ES8388 |
| 26 | I2S DOUT | ES8388 |
| 27 | I2S BCLK | ES8388 |
| 35 | I2S DIN | ES8388 |
| 32 | I2C SCL | ES8388 |
| 33 | I2C SDA | ES8388 |

That is why the module pin table below has no entry for any of them, and why
`yoRadio/myoptions.h` sets them explicitly. **They are fixed; there is no jumper
that releases them.** The same applies to the module's dedicated audio pins
(`HPOUTR/HPOUTL`, `SPOLP/SPOLN`, `SPORP/SPORN`, `LINEINL/LINEINR`,
`MIC1P/MIC1N`, `MIC2P/MIC2N`, `HBIAS`, `MBIAS`), which are the codec's analog
lines rather than GPIOs.

GPIO 0 and GPIO 2 are also **strapping pins** — they are sampled at reset. GPIO 0
is tied to the BOOT button, and Ai-Thinker note it must be left alone ("must be
hanging when using internal codec").

---

## 2. Module pinout

Left column is the pin function of the ESP32 inside the module. "Free GPIO" is
whether it can be used as a general-purpose pin.

| Pin | Name | GPIO | Free GPIO | Used by this project |
|---:|---|---|---|---|
| 1 | GND | – | – | – |
| 2 | 3V3 | – | – | – |
| 3 | SENSOR_VN | 39 | yes | headphone detect (`HP_Detect`) |
| 4 | SENSOR_VP | 36 | yes (input only) | key ladder `KEY_AD` |
| 5 | IO34 | 34 | yes (input only) | SD CLK / detect |
| 6 | IO0 | 0 | **strapping** | **internal ES8388 MCLK** |
| 7 | IO14 | 14 | yes | encoder DT |
| 8 | IO12 | 12 | yes | encoder CLK |
| 9 | IO13 | 13 | yes | – |
| 10 | IO15 | 15 | yes | encoder SW |
| 11 | IO2 | 2 | **strapping** | SD DATA0 |
| 12 | IO4 | 4 | yes | – |
| 13 | HBIAS | – | no (bias) | – |
| 14 | MIC2N | – | no (analog) | – |
| 15 | MIC1N | – | no (analog) | – |
| 16 | MBIAS | – | no (bias) | – |
| 17 | MIC1P | – | no (analog) | – |
| 18 | MIC2P | – | no (analog) | – |
| 19 | GND | – | – | – |
| 20 | GND | – | – | – |
| 21 | LINEINR | – | no (analog) | – |
| 22 | LINEINL | – | no (analog) | – |
| 23 | SPORN | – | no (analog) | – |
| 24 | NC | – | suspended | – |
| 25 | SPOLP | – | no (analog) | – |
| 26 | NC | – | suspended | – |
| 27 | HPOUTL | – | no (analog) | – |
| 28 | HPOUTR | – | no (analog) | – |
| 29 | IO5 | 5 | yes | – |
| 30 | IO18 | 18 | yes | – |
| 31 | IO23 | 23 | yes | – |
| 32 | IO19 | 19 | yes | – |
| 33 | IO22 | 22 | yes | – |
| 34 | IO21 | 21 | yes | **speaker amp mute** (`MUTE_PIN`) |
| 35 | EN | – | – | – |
| 36 | TXD0 | 1 | yes | – |
| 37 | RXD0 | 3 | yes | – |
| 38 | GND | – | – | – |

Note GPIO 34 and 36 are **input-only** on the ESP32 — they have no output driver
and no internal pull-up. Pull-ups on those nets come from the carrier board.

The V2.2 carrier schematic draws the module symbol with 39 pins (three GND at
the bottom); the V2.3 spec lists 38. The extra one is a GND, so no function is
lost either way.

---

## 3. Resistors that gate each GPIO

These sit **between** a GPIO and the onboard circuit it normally drives. Remove
the resistor (or set a DIP switch) and the pin is free for your own use. Fit
them and the onboard circuit keeps working.

Confidence column:

- **spec** — confirmed by Ai-Thinker's own documentation
- **schematic** — read from `esp32-audio-kit_v2.2_sch.pdf`; verify before soldering
- **community** — from a third-party porting write-up, corroborated by the schematic

| GPIO | Resistor | Value | Gates | To free the pin | Confidence |
|---|---|---|---|---|---|
| 5 | **R70** | **0 Ω** | KEY6 button + debounce cap | remove R70 | community + schematic |
| 13 | **R66** | **0 Ω** | KEY2 button + debounce cap | remove R66 | community + schematic |
| 18 | **R69** | **0 Ω** | KEY5 button + debounce cap | remove R69 | community + schematic |
| 19 | **R67** | **0 Ω** | KEY3 button + debounce cap (also LED5) | remove R67 | community + schematic |
| 23 | **R68** | **0 Ω** | KEY4 button + debounce cap | remove R68 | community + schematic |
| 36 | **R53** | not printed | `KEY_AD` resistor ladder (KEY1–KEY6 as one analog ADC channel) | remove R53 | schematic |
| 21 | **R46** | not printed | speaker-amp `CTRL` / ShutDown — this project's `MUTE_PIN` | remove R46 | schematic |
| 39 | **R37** | not printed | SD `DATA2` | remove R37 | schematic |
| 39 | **R36** | not printed | headphone `HP_Detect` | remove R36 | schematic |
| 22 | **R14** | not printed | LED4 indicator | remove R14 | schematic |
| 19 | **R76** | not printed | LED5 indicator | remove R76 | schematic |
| 34 | **R29** | not printed | SD `CLK` | remove R29 | schematic |
| 34 | **R18** | not printed | 3V3 pull-up on IO34 | remove R18 | schematic |
| 14 | **R26** | not printed | SD `CLK` (shared net with R29) | remove R26 | schematic |
| 2 | **R27** | not printed | SD `DATA0` | remove R27 | schematic |
| 4 | **R28** | not printed | SD `DATA1` | remove R28 | schematic |
| 12 | **R23** | not printed | SD `DATA2` net / pull-down | remove R23 | schematic |
| 15 | **R25** | not printed | SD `CMD` | remove R25 | schematic |
| 13 | **R58** | not printed | SD `DATA3` | remove R58 | schematic |

R67–R70 and R66 are explicitly called out in a community porting note as *"all
0 Ohms … unsoldered R66, 67, 68, 69, 70 to free these GPIOs from the
capacitors"*, which matches what the schematic shows. That note also warns that
every key carries a capacitor for debounce, so removing the series resistor is
what actually disconnects the pin from the capacitor — this is the whole reason
a plain 0 Ω is there.

### Keys and the ADC ladder

Ai-Thinker's key table (spec, authoritative):

| Key | GPIO | Notes |
|---|---|---|
| KEY1 | 36 | no external pull-up |
| KEY2 | 13 | no external pull-up |
| KEY3 | 19 | no external pull-up |
| KEY4 | 23 | no external pull-up |
| KEY5 | 18 | no external pull-up |
| KEY6 | 5 | no external pull-up |

All six share one ADC input, `KEY_AD`, through a resistor ladder
(R55–R59 with R61–R64), so only one of them is high at a time and the ESP32 tells
them apart by voltage. **Consequence: you cannot use two of these keys
independently.** `KEY_AD` is on GPIO 36, which is input-only.

If you remove R53 you lose all six keys but free GPIO 36 — except it still has
no output driver, so it is only useful as an ADC input.

### Key circuit component values

> **The carrier schematic prints no component values at all.** Every designator
> is present, every value is absent — the Altium PDF export carries reference
> designators without the Comment/Value fields. So the table below says
> `not printed` wherever the value genuinely cannot be established from the
> documentation in this folder. Do not read a blank as "0 Ω" or "100 k".

| Ref | Function | Value | Source |
|---|---|---|---|
| **R66** | 0 Ω link, IO13 ↔ KEY2 | **0 Ω** | porting write-up: "all 0 Ohms" |
| **R67** | 0 Ω link, IO19 ↔ KEY3 | **0 Ω** | as above |
| **R68** | 0 Ω link, IO23 ↔ KEY4 | **0 Ω** | as above |
| **R69** | 0 Ω link, IO18 ↔ KEY5 | **0 Ω** | as above |
| **R70** | 0 Ω link, IO5 ↔ KEY6 | **0 Ω** | as above |
| **R53** | IO36 → `KEY_AD` | not printed | schematic |
| **R52**, **R54** | `KEY_AD` bias network with C41 | not printed | schematic |
| **R55–R59** | ladder, VDD3V3 side | not printed | schematic |
| **R60–R64** | ladder, key side | not printed | schematic |
| **R36** | HP-detect pull-up to VDD3V3 | not printed | schematic |

Ladder topology, from the schematic: `VDD3V3` biases `KEY_AD` through R52/R54
with C41 filtering; R55–R59 form the divider chain down to GND; each key switch
pulls `KEY_AD` to a distinct tap through R60–R64, so the six keys produce six
different ADC voltages on GPIO 36. Removing R66–R70 disconnects the GPIO from
its key **and its debounce capacitor** — that is the entire reason a 0 Ω link is
there at all.

The only resistance value printed anywhere on this schematic
(`Vo=(Ra/Rb+1)*0.6V=(510k/110k+1)*0.6V=3.38V`) belongs to **R7 beside JP1**,
the USB-serial 5 V→3.3 V level divider. It is not part of the key circuit and is
not a key-ladder value.

#### If you rebuild the ladder yourself

The tap voltages are set by R61–R64, so their values are yours to choose rather
than copy:

- Keep the ladder in the tens of kΩ range. Too large and the ADC input
  impedance of input-only GPIO 36 distorts the reading; too small and you burn
  current for as long as a key is held.
- Work each divider backwards from the voltage you want: for a tap with `Ra` to
  VDD3V3 and `Rb` to GND, `V = VDD3V3 × Rb / (Ra + Rb)`. Leave **≥ ~0.15 V**
  between adjacent taps so ADC error and noise cannot make two keys ambiguous.
- KEY1 needs no divider — it reads as a direct low, which is why it still works
  with R56/R61, R57/R62, R58/R63 and R59/R64 unpopulated.

**The fitted values are not documented anywhere in this repository.** Measure
them in circuit, or pick your own with the formula above.

### ⚠ yoRadio cannot read these keys

Being blunt about this: the change is **hardware-only**. `src/core/controls.cpp`
reads buttons as discrete digital pins through OneButton, and there is no
`analogRead` of `KEY_AD` anywhere in the tree — the only `analogRead` calls are
the backlight and a commented-out seed line.

So removing R66–R70 buys you five free GPIOs **and five dead keys**. Making the
ladder work in firmware means adding ADC key scanning: sample GPIO 36, compare
against thresholds, map to button ids. That is a feature to write, not a flag to
set — no option in `options.h` enables it.

Remove R66–R70 if you want the pins for an encoder, an I2C bus or SPI, and
accept the keys going dead. Keep them fitted if you would rather have working
keys and find the pins elsewhere.

---

## 4. DIP switches on GPIO 13 and 15

Two pins are shared three ways and selected by a **DIP switch** rather than a
resistor. Per Ai-Thinker's spec:

| Switch | Off / On routing |
|---|---|
| 1 | KEY2 ↔ SD DATA3 ↔ JTAG MTCK |
| 2 | SD CMD ↔ JTAG MTDO |

So GPIO 13 and 15 are each one of three functions depending on switch position.
This project puts its encoder push-button on **GPIO 15**, which the carrier also
wants for SD CMD or JTAG.

---

## 5. What this means for this build

`yoRadio/myoptions.h` currently uses:

```c
ENC_BTNR  12   // encoder CLK
ENC_BTNL  14   // encoder DT
ENC_BTNB  15   // encoder SW
MUTE_PIN  21   // speaker amp mute
```

**GPIO 12, 14 and 15 are SD-card lines on the carrier** (`DATA2`, `CLK`, `CMD`).
Using them for the encoder means:

- the SD socket cannot be used, which is why `SDC_CS` stays `255` and `USE_SD` is
  never defined — this is not an oversight, it follows from the pin choices;
- GPIO 15 should be in the **JTAG MTDO** position on the DIP switch, otherwise
  the carrier may still be driving SD CMD;
- the encoder needs no resistors removed, because none of 12/14/15 feeds an
  onboard button — but they do share the SD nets, so a fitted SD card would
  contend.

**GPIO 21 as `MUTE_PIN`** goes through **R46** to the amplifier `CTRL` pin. That
is why the value is active-low (`MUTE_VAL LOW`) and why `myoptions.h` insists on
a 10 kΩ pull-down: R46 is a 0 Ω link, so at reset the amp-enable pin is
high-impedance and the amplifier floats, which is the loud burst on reboot.
Leaving the pull-down off means a loud pop every restart.

If you want to free GPIO 21 for something else, remove **R46** — but you also
give up the clean-reboot mute, and you should then fit a real amplifier
disable path or accept the noise burst.

### Freeing the five key GPIOs (remove R66–R70)

Removing those five 0 Ω links hands you **GPIO 5, 13, 18, 19 and 23**, which is
the whole VSPI group plus KEY2's pin:

| GPIO | Module pin | Free for |
|---|---|---|
| 5 | 29 | VSPI SS |
| 18 | 30 | VSPI SCK |
| 19 | 32 | VSPI MISO |
| 23 | 31 | VSPI MOSI |
| 13 | 9 | SD DATA3 / JTAG MTCK / KEY2 — still shared, see §4 |

Two consequences:

- **The keys stop working.** They are only reachable through the `KEY_AD`
  ladder on GPIO 36, and yoRadio has no ADC key support at all — see the note
  above. This is not reversible in software.
- **GPIO 13 stays shared.** It is a three-way DIP (KEY2 / SD DATA3 / JTAG MTCK),
  so removing R66 frees it from the key but the switch still decides its other
  two functions.

With those five free, the recommended peripheral I2C pair changes: **22 + 13**
needs R14 *and* R66, whereas **22 + 23** now needs only R14. The VSPI display
option stays open either way since 5/18/19/23 are now all yours.

Note that a second encoder is also possible on this board — `ENC_BTNL`/`ENC_BTNR`
support a second one — and 5/18/19/23 are exactly the pins it would want.

---

## 6. Using the free GPIOs for a second I2C bus

The module's ES8388 is on **I2C0** (`Wire`, hard-wired — the Arduino core cannot
re-point it). The ESP32 has a second controller, **I2C1**, which is free for a
display, sensors or an RTC.

Set the bus in `myoptions.h`; no code change is needed:

```c
#define I2C2_SDA 22
#define I2C2_SCL 23
```

Left at `255`/`255` (the default) no second bus is opened and peripherals fall
back to the ES8388 bus, which is the previous behaviour.

**These are build-time only.** The Arduino core lets a bus's pins be set
exactly once: a second `begin()` on a running bus returns `true` and silently
keeps the old pins, and `setPins()` on a running bus returns `false`. So the
pins cannot be decided at runtime — pick them, then rebuild.

### Candidate pairs

Every one needs at least one 0 Ω removed, since each of these pins currently
drives an onboard circuit.

| SDA + SCL | Remove | Leaves free | Note |
|---|---|---|---|
| **22 + 23** | R14, R68 | 5, 18, 19 | best default — see below |
| 22 + 13 | R14, R66 | 5, 18, 19, 23 | 13 is the most shared pin (DIP switch) |
| 5 + 18 | R70, R69 | 19, 23, 22 | |
| 18 + 19 | R69, R67 | 5, 23, 22 | |
| 23 + 19 | R68, R67 | 5, 18, 22 | |
| 4 + 22 | R28, R14 | 5, 18, 19, 23 | 4 has the least on-board attachment |

**22 + 23 is the best default.** GPIO22 is the Arduino-default SCL and has the
lightest on-board load, and GPIO23 is one of the four VSPI pins — so the pair
leaves **5, 18, 19** intact, which is exactly the VSPI group
(SCK=18, MISO=19, MOSI=23, SS=5) for an SPI display if you ever want one.

### Pins that cannot be used for I2C

| GPIO | Why |
|---|---|
| 0, 25, 26, 27, 32, 33, 35 | wired inside the module to its own ES8388 |
| 34, 36, 39 | input only, no output driver — invalid as SDA **or** SCL |
| 1, 3 | the USB serial console used for `pio run -t upload` and the monitor |
| 2 | strapping pin, sampled at reset |
| 12, 14, 15 | the encoder in this build (and the JTAG pins) |
| 21 | `MUTE_PIN`, the amplifier enable |

### Electrical notes

- **Fit external pull-ups** (2.2 k–10 k to 3V3) on SDA and SCL. The ESP32's
  internal ones are always enabled but are far too weak (~45 kΩ) for 400 kHz,
  and there is no way to switch them off.
- The bus is initialised at 100 kHz, the core's default when no frequency is
  given. Raise it only if your peripheral and wiring allow.
- Only one device per address per bus. The ES8388 is at `0x10`; typical
  peripherals are elsewhere (OLED `0x3C`/`0x3D`, DS3231/DS1307 `0x68`, GT911
  `0x5D`/`0x14`), so there is no conflict on the primary bus if you choose to
  share it.

### What already uses this

`src/core/i2cbuses.h` owns the bus. `i2cPeripheralBus()` returns I2C1 when
configured, otherwise `Wire`. It is the **only** place a peripheral bus is ever
begun, which is what keeps the set-once rule from being broken by a driver that
happens to initialise first.

Wired to it: the SSD1306, SH1106, SSD1305 and SSD1327 display drivers, and
`rtcsupport.cpp`. Not yet converted: `LiquidCrystal_I2C` and the GT911 touch
driver, both of which use the global `Wire` directly and would need a
`TwoWire*` member added.

---

## 7. Verify before you solder

Designators in the *schematic* rows come from reading the V2.2 carrier PDF. Two
known caveats:

1. The carrier schematic is **V2.2**; a V2.3 carrier is not covered by anything
   in this folder.
2. The auto-extraction in `kicad/extracted.json` is **not reliable** for this
   purpose — it resolves nets by nearest text label, so many resistors come out
   with the same net on both pins, and some labels (`PIR2`–`PIR5`) are
   misparsed. Every designator above was taken from the rendered schematic, and
   the key→GPIO assignments were cross-checked against Ai-Thinker's own tables
   and an independent porting write-up.

Open `esp32-audio-kit_v2.2_sch.pdf` and confirm the designator before removing
anything.