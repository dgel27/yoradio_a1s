#ifndef myoptions_h
#define myoptions_h

/* ===========================================================================
 * yoRadio options -- MODIFIED ESP32-A1S, keys on the GPIO36 resistor ladder
 * ===========================================================================
 *
 * For a board where the six front keys have been moved off their individual
 * GPIOs and onto the KEY_AD resistor ladder, which the ESP32 reads as one analog
 * input. That frees five GPIOs (13, 18, 19, 23, 5) for an encoder, a second
 * I2C bus or SPI.
 *
 * The encoder below is the same JTAG-header one as
 * examples/myoptions-a1s-jtag-encoder.h -- these two files differ only in this
 * header block and in the KEYS_ADC_* settings further down. Every other setting
 * is identical.
 *
 * ---------------------------------------------------------------------------
 * BEFORE FIRST BOOT: remove these five resistors
 * ---------------------------------------------------------------------------
 *   R66 (0R)  frees GPIO13 from KEY2   \
 *   R67 (0R)  frees GPIO19 from KEY3    |  these five are the whole point of
 *   R68 (0R)  frees GPIO23 from KEY4    |  the mod: the keys stop being digital
 *   R69 (0R)  frees GPIO18 from KEY5    |  inputs and become voltages on GPIO36
 *   R70 (0R)  frees GPIO5  from KEY6   /
 *
 *   R53 stays fitted. It is 0 ohm and joins KEY_AD to GPIO36.
 *
 * ---------------------------------------------------------------------------
 * AND FIT THE LADDER (R55-R64)
 * ---------------------------------------------------------------------------
 * The ladder resistors are not fitted on a stock board. Fit them to the values
 * below, which pair with the voltage table the firmware expects:
 *
 *   R55 1k0   R60 1k0    KEY2 ->  550 mV
 *   R56 1k6   R61 1k0    KEY3 -> 1040 mV
 *   R57 3k3   R62 1k0    KEY4 -> 1554 mV
 *   R58 6k8   R63 1k0    KEY5 -> 2064 mV
 *   R59 16k   R64 1k0    KEY6 -> 2545 mV
 *   KEY1               ->    0 mV  (K3 ties KEY_AD straight to GND)
 *   nothing pressed     -> 3300 mV  (R52 10k pull-up)
 *
 * R52 must stay at 10k and R54 must stay unpopulated -- they set the reference
 * the whole ladder is calculated against. Do not fit anything in R53.
 *
 * The full derivation, the measured R52/R53/R54 values and the reasoning are in
 * datasheets/ESP32-A1S/ESP32-A1S-GPIO-and-jumper-resistors.md.
 *
 * ---------------------------------------------------------------------------
 * WHAT EACH KEY DOES
 * ---------------------------------------------------------------------------
 * Defaults to the stock key meanings, and the encoder's own push-button keeps
 * play/pause as well:
 *
 *   KEY1  EVT_BTNLEFT    previous station / volume down
 *   KEY2  EVT_BTNCENTER  play / pause
 *   KEY3  EVT_BTNRIGHT   next station / volume up
 *   KEY4  EVT_BTNUP      station up
 *   KEY5  EVT_BTNDOWN    station down
 *   KEY6  EVT_BTNMODE    mode
 *
 * To change what a key does, edit adcKeyEvt[] in src/core/controls.cpp -- it is
 * a six-entry table and the only place the mapping exists.
 *
 * Note that with both the ladder and the encoder active there are two ways to
 * reach play/pause: KEY2 and the encoder's SW. That is deliberate, not a clash.
 *
 * ---------------------------------------------------------------------------
 * LIMITATIONS WORTH KNOWING
 * ---------------------------------------------------------------------------
 *   1. ONE KEY AT A TIME. All six share one analog node. Two keys pressed
 *      together produce a single ambiguous voltage, so chords do not work.
 *   2. THE VOLTAGES ARE FIXED AT BUILD TIME. adcKeyMv[] in controls.cpp holds
 *      the expected millivolt levels for the ladder above. If you fit a
 *      different pull-up or different ladder values, that table has to change
 *      with them -- there is no runtime self-calibration.
 *   3. GPIO36 IS INPUT-ONLY and is ADC1 channel 0, which is the accurate,
 *      WiFi-independent one. That is the right pin for this and the only one
 *      the ladder is wired to.
 */

/* ===========================================================================
 * ENCODER -- JTAG header, same as myoptions-a1s-jtag-encoder.h
 * ===========================================================================
 * MTDI (GPIO12) -> encoder CLK
 * MTMS (GPIO14) -> encoder DT
 * MTDO (GPIO15) -> encoder SW
 *
 * Set the carrier's 2-position DIP to the JTAG/MTDO position, otherwise the SD
 * socket keeps driving CMD on GPIO15 and will fight the push-button.
 *
 * GPIO13 and GPIO18 stay available as ordinary GPIOs here: the ladder does not
 * use them, and removing R66/R69 freed them. GPIO19/23/5 likewise. If you would
 * rather run the second I2C bus, see section 6 of the A1S GPIO document --
 * 22 + 23 is the recommended pair. */

#define ENC_BTNR      12  /* encoder CLK -- JTAG MTDI */
#define ENC_BTNL      14  /* encoder DT  -- JTAG MTMS */
#define ENC_BTNB      15  /* encoder SW  -- JTAG MTDO, set DIP to JTAG position */
#define ENC_INTERNALPULLUP  true
#define ENC_HALFQUARD      false

/* ===========================================================================
 * KEYS ON THE ADC LADDER
 * ===========================================================================
 * KEYS_ADC_PIN is the whole switch. At 255 (the default in options.h) none of
 * this code is compiled at all and the binary is unchanged, so leaving it alone
 * costs nothing.
 *
 * 36 is the only pin the ladder is wired to. Do not point it anywhere else
 * expecting it to work -- there is no other ADC input on this circuit. */

#define KEYS_ADC_PIN        36    /* KEY_AD on GPIO36; 255 = disabled */
#define KEYS_ADC_DEADBAND   60    /* mV half-window; see controls.cpp */
#define KEYS_ADC_SAMPLE_MS  10    /* sample period; do not raise casually */

/* No discrete buttons -- the ladder covers all six. Leaving these at 255 is
   what stops the digital OneButton path from duplicating the keys. */
#define BTN_LEFT     255
#define BTN_RIGHT    255
#define BTN_OK       255
#define BTN_UP       255
#define BTN_DOWN     255
#define BTN_MODE     255
#define BTN_INTERNALPULLUP  true

/* ===========================================================================
 * AUDIO -- identical on all three board variants
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
#define ES8388_MIC_BIAS        false   /* electret mics usually use true */

/* Power the codec down into standby when the player is stopped. */
#define ES8388_STANDBY_ON_STOP false

#endif