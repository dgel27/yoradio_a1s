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
/*****************************************************************************/

#define ES8388_MAIN_VOLUME     75      // 0-191 in dBm (from 0 to -96dBm in 0.5dBm steps) 
#define ES8388_OUT1_VOLUME     25      // 0-33 in dBm (from -45 to 4.5 dBm in 1.5dBm steps.0dBm=30 ) works for amp
#define ES8388_OUT2_VOLUME     25      // 0-33 in dBm (from -45 to 4.5 dBm in 1.5dBm steps.0dBm=30 ) HeadPhones?
#define ES8388_MAIN_MUTE       false   // This works on Audio Kit
#define ES8388_OUT1_MUTE       false   // not muted
#define ES8388_OUT2_MUTE       false   // not muted Amplifier on Audio Kit

#endif