#ifndef myoptions_h
#define myoptions_h

/* Rotary encoder: DISABLED while the on-board SD card is in use.
 *
 * SD needs IO14 (SCK), IO15 (MOSI), IO2 (MISO) and IO13 (CS). IO14 and IO15 are
 * what the encoder sits on here, and neither has a DIP escape, so the two cannot
 * coexist. Set any of these back to a pin once you are done with SD - the
 * alternatives that clash with nothing are 18/19/23, which the key ladder frees.
 */
#define ENC_BTNR			255  //CLK  -- 12, taken by SD
#define ENC_BTNL			255  //DT   -- 14, taken by SD
#define ENC_BTNB			255  //SW   -- 15, taken by SD
//#define ENC_INTERNALPULLUP	false
//#define ENC_HALFQUARD		true
//#define LED_BUILTIN			2


/* ESP32-A1S with ES8388 DAC SETTINGS */
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


#define I2S_DOUT      26
#define I2S_BCLK      27
#define I2S_LRC       25
#define I2S_DSIN      35
#define I2S_MCLK      0

/* I2C GPIOs for control ES8388 */
#define ES8388_SCL    32
#define ES8388_SDA    33

/* On-board microSD socket, SPI mode.
 *
 * The card's pin 2 is CD/DAT3 in SD mode and CS in SPI mode, and the carrier
 * routes that net to IO13 - so CS exists after all; it just carries the SD-mode
 * name on the schematic. All four SPI signals are already on the carrier:
 *
 *     card pin 2   CD/DAT3  -> IO13   CS      (via R24, behind DIP switch 1)
 *     card pin 3   CMD      -> IO15   MOSI    (via R25, behind DIP switch 2)
 *     card pin 5   CLK      -> IO14   SCK     (via R26)
 *     card pin 7   DATA0    -> IO2    MISO    (via R27)
 *
 * IO12 (DATA2) and IO34 (card detect) are not needed in SPI mode.
 *
 * REQUIRED: DIP switch 1 must be in the SD position (KEY2 / SD DATA3 / JTAG
 * MTCK). Left on KEY2, pressing KEY2 shorts CS to ground and the card will not
 * initialise. The keys themselves are on the GPIO36 ladder now, so nothing else
 * is lost by moving the switch.
 *
 * SPI, not SDMMC: the Arduino ESP32 SD library this builds against is SPI-only
 * (FatFs over SPIClass), so there is no faster interface available without
 * replacing the filesystem layer. At 20MHz that is roughly 120x what a 128kbps
 * MP3 stream needs, so it is not worth doing.
 */
#define SDC_CS        13     // card pin 2, CD/DAT3 = CS in SPI mode
#define SD_SPIPINS    14, 2, 15   // SCK=IO14, MISO=IO2, MOSI=IO15
#define SD_DETECT     34     // card-detect switch, via R29
#define SD_DETECT_ACTIVE LOW  // level that means a card is inserted

/* Amplifier enable PIN - eternal AMP on ESP32-A1S Kit */
//#define GPIO_PA_EN       21   /* Amplifier GPIO */
//#define GPIO_PA_LEVEL    HIGH /* Amplifier enable level */

/* Headphone jack detect, GPIO39 via R36 on the carrier.
 *
 * Silences the HEADPHONE amplifier only while nothing is plugged in, so the amp
 * is not driving an empty output. The speaker is left alone on purpose.
 * Your own headphone-mute setting is still stored while unplugged and comes back
 * when you plug in.
 *
 * HP_DETECT_ACTIVE is the pin level that means "plugged in". LOW, measured on this
 * board: with nothing plugged in the pin reads HIGH, so the empty jack pulls the
 * net UP and inserting a plug pulls it down. That is the opposite of what the
 * schematic implies (R36 looks like a pull-up to VDD3V3 with a normally-closed
 * contact, which would make attached = HIGH), so trust the measurement over the
 * drawing. Confirmed with `hp` over telnet across five reads, all stable at 1.
 *
 * If you swap the jack or the carrier revision, run `hp` again and flip this back
 * if `attached` no longer follows the plug. Nothing else needs changing.
 *
 * Note GPIO39 is input-only and, like all of GPIO34-39, has no internal pull-up or
 * pull-down - R36 is the only bias on this net.
 */
#define HP_DETECT              39    // jack sense pin; 255 = disabled
#define HP_DETECT_ACTIVE       LOW   // level meaning "a headphone is plugged in"
#define HP_DETECT_SAMPLE_MS    50
#define HP_DETECT_DEBOUNCE_MS  250
#define HP_AUTOMUTE            true  // silence the headphone amp while unplugged
/* The MUTE_PIN is inversed GPIO_PA_EN and implemented in YORADIO.
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
#define MUTE_PIN        21   /*  MUTE Pin */
#define MUTE_VAL        LOW  /*  Write this to MUTE_PIN when player is stopped */

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
#define ES8388_MAIN_VOLUME     192     // 0dB, unity
/* Analog output volume. Registers 46/47 (LOUT1/ROUT1) and 48/49 (LOUT2/ROUT2).
   0..33 where 30 = 0dB and 33 = +4.5dB, 1.5dB steps, 0 = -45dB.
   On the ESP32-A1S: OUT1 = speaker (SPOLP/N -> U4/U5 -> J3/J4), OUT2 =
   headphone (HPOUTL/R -> J2). Only the speaker path has a hardware enable
   (GPIO21 -> R46 -> the amps' CTRL pin); the headphone jack has none, so it
   can only be muted here. Values above 33 are clamped. */
#define ES8388_OUT1_VOLUME     30      // 0dB into the speaker amps
#define ES8388_OUT2_VOLUME     30      // 0dB into the headphone jack

#define ES8388_MAIN_MUTE       false   // digital mute
#define ES8388_OUT1_MUTE       false   // speaker amp not muted
#define ES8388_OUT2_MUTE       false   // headphone amp not muted

/* DAC Control 7 (0x1d) */
#define ES8388_STEREO_EFF      4       // 0..7, 0 = off, 7 = strongest widening
#define ES8388_MONO            false   // false = stereo, true = (L+R)/2
#define ES8388_VPP_SCALE       2       // 0: 3.5V, 1: 4.0V, 2: 3.0V, 3: 2.5V

/* DAC Control 3 (0x19): soft volume ramp. Keep enabled, it removes the
   click/pop on every volume change. Rate n = 0.5dB per N*4 LRCK (n=0..3). */
#define ES8388_SOFT_RAMP       true
#define ES8388_RAMP_RATE       0       // 0..3 -> 4 / 32 / 64 / 128 LRCK per 0.5dB

/* DAC Control 6 (0x1c) */
#define ES8388_CLICK_FREE      true    // click-free power up/down
#define ES8388_DEEMPHASIS      0       // 0 off, 1 = 32k, 2 = 44.1k, 3 = 48k
#define ES8388_INVERT_L        false
#define ES8388_INVERT_R        false

/* DAC Control 23 (0x2d): analog output impedance reference */
#define ES8388_VROI            false   // false = 1.5k (default), true = 40k

/* Line-in routing, the analogue input on the J1 jack. 0 = off (radio only),
   1 = mixed in with the radio, 2 = line-in only (the radio's digital path is
   muted, which line-in survives because it joins after the DAC).

   false/0 is the safe default with nothing plugged into J1: line-in left on with
   an empty jack just amplifies noise. */
#define ES8388_LINEIN_MIX      false
#define ES8388_LINEIN_GAIN     -12     // -15..+6 dB, 3dB steps

/* Mic (ADC). The ADC stays powered down for playback; these apply when
   adc_power(true) is called. See the deferred mic work. */
#define ES8388_MIC_PGA         3       // 0..8 -> 0..+24dB in 3dB steps
#define ES8388_MIC_INPUT       0       // 0 = LIN1&RIN1, 1 = LIN2&RIN2, 2 = differential
#define ES8388_MIC_BIAS        false   // electret mics usually need true

/* Power the codec down into standby when the player is stopped. */
#define ES8388_STANDBY_ON_STOP false

#endif
