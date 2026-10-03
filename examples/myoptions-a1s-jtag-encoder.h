#ifndef myoptions_h
#define myoptions_h

/* ===========================================================================
 * yoRadio options -- MODIFIED ESP32-A1S, JTAG header repopulated as encoder
 * ===========================================================================
 *
 * For a board where the JTAG header pins have been rewired to carry a rotary
 * encoder instead of JTAG. The three pins used are the ones the ESP32-A1S
 * module already brought out as MTDI / MTMS / MTDO, so no trace has to be cut:
 *
 *     JTAG pin   module function   GPIO
 *     MTDI       MTDI              12    encoder CLK
 *     MTMS       MTMS              14    encoder DT
 *     MTDO       MTDO              15    encoder SW
 *
 * Every other setting is identical to the stock board; only the encoder block
 * and these notes differ. See examples/myoptions-a1s-stock.h for the
 * unmodified variant.
 *
 * ---------------------------------------------------------------------------
 * WHY THESE PINS ARE THE RIGHT ONES FOR A MODIFIED BOARD
 * ---------------------------------------------------------------------------
 * The stock board has no spare GPIO that is not already doing something, which
 * is why the stock file has to depopulate three resistors. The JTAG header is
 * the clean way in: those four pins (12/13/14/15) are routed to the header
 * specifically as an alternate path, so the encoder can hang off the header
 * while the carrier keeps its own use of the same GPIOs.
 *
 * The cost is that GPIO12, 14 and 15 are ALSO the carrier's SD nets
 * (DATA2, CLK, CMD). Two consequences:
 *
 *   1. DIP SWITCH POSITION MATTERS.
 *      The carrier has a 2-position DIP that reroutes GPIO15 between SD CMD and
 *      JTAG MTDO. Set it to the JTAG/MTDO position, otherwise the card socket
 *      keeps driving CMD and will fight the encoder push-button.
 *      GPIO12 and GPIO14 have no DIP option -- they are routed to the header
 *      as-is.
 *
 *   2. THE SD SOCKET MUST NOT CONTEND. There is no CS pin on this socket
 *      (see the stock file for why SD is unusable here anyway), but if the
 *      socket is populated the SD CLK and DATA2 nets are still driven from the
 *      carrier. If the encoder misbehaves -- ghost rotation, detents that do not
 *      register, a push-button that bounces -- the first thing to check is
 *      whether anything is loading GPIO12/14.
 *
 *      Whether that needs a resistor removed depends on what the socket side
 *      looks like on your board, and the carrier schematic available in this
 *      repo (esp32-audio-kit_v2.2_sch.pdf) is NOT reliable enough to tell you:
 *      the resistors around the SD block (R23 on GPIO12, R26/R29 on GPIO14, R25
 *      on GPIO15) are ambiguous in that render between series links and
 *      pull-downs. The kicad/ netlist extraction in this repo is worse -- it
 *      resolves nets by nearest text label and gets several of these wrong.
 *
 *      So: measure before you solder. With the board powered off, check
 *      continuity from the MTDI / MTMS / MTDO header pins to the SD socket pads.
 *      If they are continuous, either lift the series resistor or the socket's
 *      side of the net, or accept that the socket is loading those lines.
 *      A fitted 10k pull-down is survivable next to an ESP32 output; a driven
 *      socket line is not.
 *
 * ---------------------------------------------------------------------------
 * ENCODER WIRING
 * ---------------------------------------------------------------------------
 *   MTDI (GPIO12) -> encoder CLK
 *   MTMS (GPIO14) -> encoder DT
 *   MTDO (GPIO15) -> encoder SW
 *   GND           -> encoder GND
 *
 * Most rotary encoders need a debounce capacitor between CLK and DT (~100nF).
 * Add it.
 *
 * Because these are also SDMMC pins, if you ever want the SD card back you must
 * move the encoder -- there is no pin-multiplexing that shares CLK/CMD between
 * an encoder and SDMMC.
 */

#define ENC_BTNR      12  /* encoder CLK -- JTAG MTDI */
#define ENC_BTNL      14  /* encoder DT  -- JTAG MTMS */
#define ENC_BTNB      15  /* encoder SW  -- JTAG MTDO, set DIP to JTAG position */
/* Both default to true/false in options.h, so these are spelled out only to
   make it visible. Leave ENC_INTERNALPULLUP at true unless your encoder
   already has external pull-ups - fitting both fights each other. */
#define ENC_INTERNALPULLUP  true
#define ENC_HALFQUARD      false

/* Discreet buttons, if wired to other free GPIOs on the modified board.
   Left at 255 = not fitted. GPIO18/19/23 are free the same way as on the stock
   board (remove R69/R67/R68) if you want them. */
#define BTN_LEFT     255
#define BTN_RIGHT    255
#define BTN_OK       255
#define BTN_UP       255
#define BTN_DOWN     255
#define BTN_MODE     255
#define BTN_INTERNALPULLUP  true

/* ===========================================================================
 * AUDIO -- identical on both board variants
 * =========================================================================== */

#define ES8388_ENABLE true

/* ======================= AUDIO DECODERS ================================ */
/* Set one to false and that codec is left out of the build entirely. Keep MP3
   and AAC: internet radio is almost entirely one of those two.

   Measured flash cost of each on this board (1.75MB app partition):
       MP3       ~31 KB      AAC ~43 KB      FLAC ~11 KB
       OPUS      ~75 KB      VORBIS ~68 KB

   Turning OPUS and VORBIS off frees about 144 KB, which is more than every
   other optional setting here combined. Turning off all five leaves 231 KB
   free, but the radio will then only play WAV and will reject most stations.

   A decoder you switch off is not detected, so a station using it fails
   instead of playing. All are true here: nothing is lost. */
#define DECODER_MP3      true
#define DECODER_AAC      true
#define DECODER_FLAC     true
#define DECODER_OPUS     true
#define DECODER_VORBIS   true

/* I2S and I2C pins are FIXED on the ESP32-A1S module: it integrates its own
   ES8388, so these GPIOs are wired to the codec inside the module and are not
   brought out. There is no jumper that releases them and no alternative
   assignment. They are listed here only to make that explicit. */
#define I2S_DOUT      26
#define I2S_BCLK      27
#define I2S_LRC       25
#define I2S_DSIN      35
#define I2S_MCLK      0
#define ES8388_SCL    32
#define ES8388_SDA    33

/* The MUTE_PIN is an inverted GPIO_PA_EN, handled by yoRadio.
 *
 * HARDWARE NOTE: fit a pull-down resistor (10k is fine) between MUTE_PIN and
 * GND. A restart takes every GPIO to input/high-Z, so without an external pull
 * the amplifier enable pin floats for the whole time the ESP32 is resetting and
 * running its bootloader - the amplifier then amplifies whatever the codec
 * output stage is doing, which is loud and unrelated to the volume setting.
 *
 * The firmware cannot cover that gap: it drives the mute level as the first
 * thing in setup() (closing the window from there on) and calls
 * Player::prepareForRestart() before every ESP.restart(), but nothing executes
 * between esp_restart() and setup(). Only the resistor covers that. */
#define MUTE_PIN        21
#define MUTE_VAL        LOW

/* Optional inputs. */
#define SD_DETECT        255
#define HP_DETECT        39

/*****************************************************************************/
/*                                                                           */
/*                     ES8388 AUDIO CODEC Settings                           */
/*                                                                           */
/* Register reference: ES8388 DS.pdf rev 12.0 / ES8388 user guide (Sep 2018)  */
/*                                                                           */
/*****************************************************************************/

/* Digital volume, attenuates BOTH analog outputs. Register 26/27.
   0..192 where 192 = 0dB (unity) and 0 = -96dB, 0.5dB per step.
   192 (0dB) is safe here: the software chain can never exceed the decoder's
   own output. Audio halves each sample (Audio.cpp, "half Vin so we can boost up
   to 6dB in filters") and the biquad gains are clamped to +6dB max, so the
   worst case nets to 0dB. For ES8388 builds this register is driven by the
   main-page slider, not by this value: the slider is the DAC master register
   (index.html "volume" -> config.store.volume -> Player::applyEs8388Volume).
   This define only seeds config.store.es8388.es_master_vol, a field nothing
   reads, kept solely so sizeof(config_t) and the EEPROM layout stay stable. */
#define ES8388_MAIN_VOLUME     192     /* 0dB, unity */
/* Analog output volume. Registers 46/47 (LOUT1/ROUT1) and 48/49 (LOUT2/ROUT2).
   0..33 where 30 = 0dB and 33 = +4.5dB, 1.5dB steps, 0 = -45dB.
   On the ESP32-A1S: OUT1 = speaker (SPOLP/N -> U4/U5 -> J3/J4), OUT2 =
   headphone (HPOUTL/R -> J2). Only the speaker path has a hardware enable
   (GPIO21 -> R46 -> the amps' CTRL pin); the headphone jack has none, so it
   can only be muted here. Values above 33 are clamped. */
#define ES8388_OUT1_VOLUME     30      /* 0dB into the speaker amp */
#define ES8388_OUT2_VOLUME     30      /* 0dB into the headphone amp */

#define ES8388_MAIN_MUTE       false   /* digital mute */
#define ES8388_OUT1_MUTE       false   /* speaker amp not muted */
#define ES8388_OUT2_MUTE       false   /* headphone amp not muted */

/* DAC Control 7 (0x1d) */
#define ES8388_STEREO_EFF      4       /* 0..7, 0 = off, 7 = strongest widening */
#define ES8388_MONO            false   /* false = stereo, true = (L+R)/2 */
#define ES8388_VPP_SCALE       2       /* 0: 3.5V, 1: 4.0V, 2: 3.0V, 3 = 2.5V */

/* DAC Control 3 (0x19): soft volume ramp. Keep enabled, it removes the
   click/pop on every volume change. Rate n = 0.5dB per N*4 LRCK (n=0..3). */
#define ES8388_SOFT_RAMP       true
#define ES8388_RAMP_RATE       0       /* 0..3 -> 4 / 32 / 64 / 128 LRCK per 0.5dB */

/* DAC Control 6 (0x1c) */
#define ES8388_CLICK_FREE      true    /* click-free power up/down */
#define ES8388_DEEMPHASIS      0       /* 0 off, 1 = 32k, 2 = 44.1k, 3 = 48k */
#define ES8388_INVERT_L        false
#define ES8388_INVERT_R        false

/* DAC Control 23 (0x2d): analog output impedance reference */
#define ES8388_VROI            false   /* false = 1.5k (default), true = 40k */

/* Output mixer line-in contribution. false keeps LIN1/LIN2 out of the outputs
   (only the DAC is heard), which is the safe default with nothing plugged
   into the line-in pins. */
#define ES8388_LINEIN_MIX      false
#define ES8388_LINEIN_GAIN     -12     /* -15..+6 dB, 3dB steps */

/* Mic (ADC). The ADC stays powered down for playback; these apply when
   adc_power(true) is called. */
#define ES8388_MIC_PGA         3       /* 0..8 -> 0..+24dB in 3dB steps */
#define ES8388_MIC_INPUT       0       /* 0 = LIN1&RIN1, 1 = LIN2&RIN2, 2 = differential */
#define ES8388_MIC_BIAS        false   /* electret mics usually need true */

/* Power the codec down into standby when the player is stopped. */
#define ES8388_STANDBY_ON_STOP false

#endif