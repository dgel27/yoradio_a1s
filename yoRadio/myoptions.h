#ifndef myoptions_h
#define myoptions_h

#define ENC_BTNR			12  //CLK
#define ENC_BTNL			14  //DT 
#define ENC_BTNB			15  //SW 
//#define ENC_INTERNALPULLUP	false
//#define ENC_HALFQUARD		true
//#define LED_BUILTIN			2


/* ESP32-A1S with ES8388 DAC SETTINGS */
#define ES8388_ENABLE true

#define I2S_DOUT      26
#define I2S_BCLK      27
#define I2S_LRC       25
#define I2S_DSIN      35
#define I2S_MCLK      0

/* I2C GPIOs for control ES8388 */
#define ES8388_SCL    32
#define ES8388_SDA    33

/* Amplifier enable PIN - eternal AMP on ESP32-A1S Kit */
//#define GPIO_PA_EN       21   /* Amplifier GPIO */
//#define GPIO_PA_LEVEL    HIGH /* Amplifier enable level */
//#define SD_DETECT        34 // ?
//#define HP_DETECT        39 // ?
/* The MUTE_PIN is inversed GPIO_PA_EN and implemented in YORADIO */
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
   0..192 where 192 = 0dB (loudest) and 0 = -96dB, 0.5dB per step. */
#define ES8388_MAIN_VOLUME     75      // ~-58dB digital attenuation
/* Analog output volume. Registers 46/47 (LOUT1/ROUT1) and 48/49 (LOUT2/ROUT2).
   0..33 where 30 = 0dB and 33 = +4.5dB, 1.5dB per step, 0 = -45dB.
   On the ESP32-A1S: OUT1 = headphone amp (HPOUTL/R), OUT2 = on-board
   speaker amp (SPOLP/N). Values above 33 are clamped. */
#define ES8388_OUT1_VOLUME     25      // ~-16dB into the external amp
#define ES8388_OUT2_VOLUME     25      // ~-16dB into the speaker amp

#define ES8388_MAIN_MUTE       false   // digital mute
#define ES8388_OUT1_MUTE       false   // headphone amp not muted
#define ES8388_OUT2_MUTE       false   // speaker amp not muted

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

/* Output mixer line-in contribution. false keeps LIN1/LIN2 out of the outputs
   (only the DAC is heard), which is the safe default with nothing plugged
   into the line-in pins. */
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