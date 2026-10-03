#ifndef myoptions_h
#define myoptions_h

/* ===========================================================================
 * yoRadio options -- STOCK Ai-Thinker ESP32-A1S / ESP32-AudioKit
 * ===========================================================================
 *
 * For an unmodified board. The encoder below sits on three GPIOs that the
 * carrier board otherwise gives to the KEY3/KEY4/KEY5 push buttons, so to use it
 * you must remove three 0 ohm resistors first. Nothing else on the board needs
 * touching.
 *
 * If your board has the JTAG header repopulated as the encoder instead, use
 * examples/myoptions-a1s-jtag-encoder.h -- the two files differ in this block
 * and in the notes beside it, nothing else.
 *
 * ---------------------------------------------------------------------------
 * BEFORE FIRST BOOT: remove these three resistors
 * ---------------------------------------------------------------------------
 *   R69 (0R)  frees GPIO18 from KEY5   -> use as encoder CLK
 *   R67 (0R)  frees GPIO19 from KEY3   -> use as encoder DT
 *   R68 (0R)  frees GPIO23 from KEY4   -> use as encoder SW
 *
 * Every key on this board carries a debounce capacitor, and the 0R is the only
 * thing separating the GPIO from it. Depopulating the resistor is what actually
 * disconnects the pin -- desoldering the switch is not enough. With the
 * resistor fitted the key capacitor fights your encoder and you will see ghost
 * rotation and missed detents.
 *
 * Two pins stay available the same way if you need them:
 *   R70 (0R) frees GPIO5  from KEY6
 *   R66 (0R) frees GPIO13 from KEY2
 *
 * Keys KEY1 (GPIO36) and KEY2 (GPIO13) are unaffected by the above, but note
 * that yoRadio reads discrete button pins rather than the KEY_AD resistor
 * ladder, so it cannot use the keys through the shared ADC input. Set the
 * BTN_* pins below if you have wired real switches to free GPIOs.
 *
 * ---------------------------------------------------------------------------
 * ENCODER WIRING
 * ---------------------------------------------------------------------------
 *   CLK -> GPIO18      DT -> GPIO19      SW -> GPIO23     GND -> GND
 *
 * Most rotary encoders need a debounce capacitor between CLK and DT (~100nF).
 * Add it.
 *
 * ---------------------------------------------------------------------------
 * SD CARD IS NOT USABLE ON THIS BOARD
 * ---------------------------------------------------------------------------
 * The carrier's SD socket is wired to the ESP32's SDMMC pins -- CLK GPIO14, CMD
 * GPIO15, DATA0 GPIO2, DATA1 GPIO4, DATA2 GPIO12, DATA3 GPIO13 -- and the socket
 * has no chip-select pin at all. yoRadio's SD support expects an SPI-mode card
 * with a CS line, so SDC_CS stays 255 and USE_SD is never defined. That is not
 * an oversight to be fixed: there is no CS to give it. It also means the
 * GPIO12/14/15 "SD" labels on those nets are theoretical for this socket.
 */

#define ENC_BTNR      18  /* encoder CLK -- remove R69 */
#define ENC_BTNL      19  /* encoder DT  -- remove R67 */
#define ENC_BTNB      23  /* encoder SW  -- remove R68 */
/* Leave ENC_INTERNALPULLUP at true unless your encoder already has external
   pull-ups - fitting both fights each other. */
#define ENC_INTERNALPULLUP  true
#define ENC_HALFQUARD      false

/* Discreet buttons. Left at 255 = not fitted. GPIO5 is free here if you add
   one (remove R70). */
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

/* Optional inputs. Leave SD_DETECT / HP_DETECT undefined unless you have wired
   them; both are behind resistors (R18/R29 and R36) on the carrier. */
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
#define ES8388_VPP_SCALE       2       /* 0: 3.5V, 1: 4.0V, 2: 3.0V, 3: 2.5V */

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

/* Line-in routing, the analogue input on the J1 jack. 0 = off (radio only),
   1 = mixed in with the radio, 2 = line-in only (the radio's digital path is
   muted, which line-in survives because it joins after the DAC).

   false/0 is the safe default with nothing plugged into J1: line-in left on with
   an empty jack just amplifies noise. */
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