#ifndef myoptions_h
#define myoptions_h

/* ---------------------------------------------------------------------------
 * yoRadio - ESP32-A1S (Ai-Thinker ESP32-Audio-Kit) with ES8388 codec
 * ---------------------------------------------------------------------------
 *
 * Drop-in replacement for yoRadio/myoptions.h on the ESP32-A1S / ESP32-Audio-Kit
 * V2.2 and the V2.3 revision of it. Everything needed to get audio out of the
 * board is already set below; the optional sections (display, encoder, SD) are
 * left commented out because the bare board has none of them.
 *
 * Usage:
 *   cp examples/myoptions-a1s.h yoRadio/myoptions.h
 *
 * Then build with the A1S board definition, which this fork provides:
 *   platformio.ini            -> board = esp32-a1s
 *   boards/esp32-a1s.json     -> PSRAM + the esp32-psram-cache bug workarounds
 *   partition_1.75Mapp_OTA_0.375Mfs.csv
 *
 * The upstream generator at
 * https://e2002.github.io/docs/myoptions-generator.html
 * still works for other boards; use this file as the starting point for the
 * A1S so you inherit the verified codec settings below.
 *
 * All ES8388 register values below are annotated with the register they come
 * from, taken from "ES8388 DS.pdf" rev 12.0 and the ES8388 user guide (Sep 2018),
 * both in datasheets/ES8388/.
 * -------------------------------------------------------------------------- */


/* ======================= ES8388 CODEC ==================================== */

/* NOTE: the value must be spelled out. Upstream examples use a bare
 * "#define ES8388_ENABLE" with no value, which does NOT compile in this fork:
 * netserver.cpp uses the macro in an expression, e.g.
 *   if (ES8388_ENABLE || dbgact) act += F("\"group_es8388\",");
 * so a bare #define fails with "expected primary-expression before '||' token".
 * The #ifdef guards elsewhere would accept it; that one line will not. */
#define ES8388_ENABLE true

/* I2S pins to the codec's DAC.
   I2S_DOUT 26 = BCKL (bit clock into the codec)
   I2S_BCLK 27 = DIN   (data into the codec)   <- the names are swapped relative
   I2S_LRC  25 = LRCLK (left/right clock)          to their usual sense
   I2S_DSIN 35 = data from the codec's ADC (unused for playback)
   I2S_MCLK  0 = MCLK  */
#define I2S_DOUT      26
#define I2S_BCLK      27
#define I2S_LRC       25
#define I2S_DSIN      35
#define I2S_MCLK      0

/* I2C control bus for the codec. The codec itself is at address 0x10.
   These two names are NOT the ES_IIC_CLK / ES_IIC_DATA pair used by the
   upstream templates - those two are not referenced anywhere in this firmware.
   src/core/player.cpp reads exactly these two. */
#define ES8388_SCL    32
#define ES8388_SDA    33

/* Amplifier enable. On this board MUTE_PIN is the inverted GPIO_PA_EN, and
   the amplifier is active-high, so writing MUTE_VAL means "shut the speaker
   amp up". MUTE_VAL = LOW mutes, HIGH unmutes.
   OUT1 of the codec feeds the headphone amp, OUT2 feeds the on-board speaker
   amp, and the same MUTE_PIN gates both on this board. */
#define MUTE_PIN        21   /* = GPIO_PA_EN, amplifier enable */
#define MUTE_VAL        LOW  /* write this to MUTE_PIN when stopped (i.e. mute) */

/* HARDWARE: fit a pull-down resistor (10k is fine) between MUTE_PIN and GND.
   A restart takes every GPIO to input/high-Z, so with no external pull the
   amplifier enable pin floats while the ESP32 resets and runs its bootloader,
   and the amp amplifies whatever the codec output stage happens to be doing -
   a loud burst that is unrelated to the volume setting.
   The firmware drives the mute level as the first thing in setup() and calls
   Player::prepareForRestart() before every ESP.restart(), so it covers the
   long pre-restart and post-boot windows. Nothing executes between
   esp_restart() and setup(), so only the resistor covers that gap. */


/* ======================= ES8388 CODEC SETTINGS ============================ */

/* Digital volume, register 26/27 (LDACVOL/RDACVOL), attenuates BOTH analog
   outputs. 0..192, where 192 = 0dB (unity) and 0 = -96dB, 0.5dB per step.
   192 (0dB) is safe because the software chain can never exceed the decoder's
   own output: Audio halves each sample and the biquad tone gains are clamped to
   +6dB, so the worst case nets to 0dB. For ES8388 builds this register is
   driven by the main-page slider instead (index.html "volume" ->
   config.store.volume -> Player::applyEs8388Volume); this define only seeds a
   field nothing reads, kept so sizeof(config_t) and the EEPROM layout hold. */
#define ES8388_MAIN_VOLUME     192     // 0dB, unity

/* Analog output volume, registers 46/47 (LOUT1/ROUT1) and 48/49 (LOUT2/ROUT2).
   0..33, where 30 = 0dB and 33 = +4.5dB, 1.5dB per step, 0 = -45dB. The field
   is 6 bits wide, so values above 33 are clamped rather than bleeding into the
   reserved bits. On this board OUT1 is the on-board speaker amp and OUT2 the
   headphone amp. */
#define ES8388_OUT1_VOLUME     30      // 0dB into the headphone amp
#define ES8388_OUT2_VOLUME     30      // 0dB into the speaker amp

#define ES8388_MAIN_MUTE       false   // digital mute
#define ES8388_OUT1_MUTE       false   // headphone amp not muted
#define ES8388_OUT2_MUTE       false   // speaker amp not muted

/* DAC Control 7, register 29 (0x1d).
   SE is bits 4:2, MONO is bit 5, Vpp_scale is bits 1:0.
   SE is a stereo WIDENING effect, not a tone EQ - the tone controls on the
   main page are software biquads. */
#define ES8388_STEREO_EFF      4       // 0..7, 0 = off, 7 = strongest widening
#define ES8388_MONO            false   // false = stereo, true = (L+R)/2
#define ES8388_VPP_SCALE       2       // 0: 3.5V, 1: 4.0V, 2: 3.0V, 3: 2.5V

/* DAC Control 3, register 25 (0x19): soft volume ramp. Keep this on - it is
   what removes the click on every volume change. Rate n = 0.5dB per N*4 LRCK. */
#define ES8388_SOFT_RAMP       true
#define ES8388_RAMP_RATE       0       // 0..3 -> 4 / 32 / 64 / 128 LRCK per 0.5dB

/* DAC Control 6, register 28 (0x1c). Deemphasis is bits 7:6, the two phase
   inversions are bits 5 and 4, ClickFree is bit 3. */
#define ES8388_CLICK_FREE      true    // click-free power up/down
#define ES8388_DEEMPHASIS      0       // 0 off, 1 = 32k, 2 = 44.1k, 3 = 48k
#define ES8388_INVERT_L        false
#define ES8388_INVERT_R        false

/* DAC Control 23, register 45 (0x2d): analog output impedance reference. */
#define ES8388_VROI            false   // false = 1.5k (default), true = 40k

/* Line-in contribution to the output mixers. false keeps LIN1/LIN2 out of the
   outputs, which is the safe default with nothing plugged into those pins. */
#define ES8388_LINEIN_MIX      false
#define ES8388_LINEIN_GAIN     -12     // -15..+6 dB, 3dB steps

/* Microphone / ADC. The ADC stays powered down for playback; these take
   effect if you switch the ADC on from the web UI. */
#define ES8388_MIC_PGA         3       // 0..8 -> 0..+24dB in 3dB steps
#define ES8388_MIC_INPUT       0       // 0 = LIN1&RIN1, 1 = LIN2&RIN2, 2 = differential
#define ES8388_MIC_BIAS        false   // electret mics usually need true

/* Park the codec in standby whenever the player stops. */
#define ES8388_STANDBY_ON_STOP false

/* All of the above are the FACTORY DEFAULTS. From the first boot they can be
 * changed at runtime from Settings -> es8388 in the web UI, or over telnet
 * (esvol, esvol1, esv2, esstereo, esreset, esdump, ...), and the values persist.
 * Use the group's reset button, or the telnet "esreset", to come back to the
 * values in this file. */


/* ======================= DISPLAY (OPTIONAL) ============================== */
/*
   The bare A1S has no display, so leave this out. The default (DSP_DUMMY)
 * builds and runs fine with no screen; you get the web UI only.
   Uncomment ONE line and set I2C_SDA/I2C_SCL as noted below.

#define DSP_MODEL  DSP_SSD1306      // 128x64  0.96"
//#define DSP_MODEL  DSP_SH1106     // 128x64  1.3"
//#define DSP_MODEL  DSP_SSD1306x32 // 128x32  0.91"
*/
//#define INITR_BLACKTAB  // only for the ST7735 / ST7789 SPI TFTs

/* IMPORTANT if you add an I2C display: the defaults are I2C_SDA 21 and
   I2C_SCL 22, and 21 is already MUTE_PIN here - the display and the amplifier
   mute would fight over one pin. Move the display onto the codec's bus instead,
   which is free: the codec is at 0x10, so a display at its own address on the
   same bus is fine. */
#define I2C_SDA 33
#define I2C_SCL 32
//#define I2C_RST -1   // set to a GPIO only if your display needs a reset line

/* For anything not listed above, use the upstream generator:
 * https://e2002.github.io/docs/myoptions-generator.html */


/* ======================= ENCODER (OPTIONAL) ============================== */
/*
   The A1S has no rotary encoder; this is for a board you have fitted one to.
   ALL THREE PINS MUST BE GIVEN together - defining only some of them leaves the
   rest at their defaults and can leave the encoder half-configured.

   Also check these against your build environment before using them: the
   Yoradio_JLINK_debug environment builds with -D FREE_JTAG_PINS, which reserves
   pins 12, 13, 14 and 15. An encoder on any of those will misbehave in that
   environment, so pick different pins there.

#define ENC_BTNR    16   // CLK
#define ENC_BTNL    17   // DT
#define ENC_BTNB    18   // SW
//#define ENC_INTERNALPULLUP  true
//#define ENC_HALFQUARD      true
*/


/* ======================= SD CARD (OPTIONAL) ============================== */
/*
   No SD slot on the bare A1S. For an external SD module, set SDC_CS to its chip
   select. Leaving it at 255 compiles the SD support out, which also stops
   store.lastSdStation / store.sdsnuffle from occupying EEPROM.

//#define SDC_CS 5
//#define SD_HSPI  false  // true for the HSPI pins (miso=12, mosi=13, clk=14)
*/


/* ======================= IR REMOTE (OPTIONAL) ============================ */
/*
//#define IR_PIN 12
//#define IR_TIMEOUT 80
//#define IRTOL 35
*/


/* ======================= BUTTONS (OPTIONAL) ============================== */
/*
   The A1S has no buttons either. All the pins must be set together, or the
   button group is left partly on defaults.

//#define BTN_LEFT   36
//#define BTN_CENTER 19
//#define BTN_RIGHT   5
*/


/* ======================= NOT SET HERE ==================================== */
/*
   Deliberately absent, because they are per-installation rather than per-board
   and you should not inherit anyone else's:

     - the Wi-Fi network: that lives in data/data/wifi.csv on the device, and
       examples/wifi.csv is a template to copy.
     - the station list: data/data/playlist.csv, see examples/playlist.csv.
     - MQTT: yoRadio/mqttoptions.h, which must exist for the build to work but
       ships with an empty host, i.e. MQTT off until you set one in
       Settings -> mqtt.
     - the theme: mytheme.h, copied from examples/mytheme.h.
*/

#endif
