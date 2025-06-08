#ifndef myoptions_h
#define myoptions_h

/*  SD HSPI PINS. SD SCK must be connected to pin 14
                  SD MISO must be connected to pin 12
                  SD MOSI must be connected to pin 13  */

//#define SDC_CS  15    /* SDCARD CS pin */
//#define SD_HSPI true	/* use HSPI for SD (miso=12, mosi=13, clk=14) instead of VSPI (by default) */

#define BTN_LEFT			36 //KEY1 /VOL-/PREV
#define BTN_CENTER			19 //KEY3 /STOP/PLAY
#define BTN_RIGHT			5 //KEY6 /VOL+/NEXT

//#define BTN_UP  			19

//#define BTN_DOWN			5 // VOL- - KEY2
//#define BTN_MODE			36

// IO19 - KEY3
// IO13 - KEY2
// IO18 - ?
// IO05 - KEY6
// IO23 - ?
// IO36 - KEY1



/* ESP32-A1S with ES8388 DAC SETTINGS */
#define ES8388_ENABLE

#define I2S_DOUT      26
#define I2S_BCLK      27
#define I2S_LRC       25
#define I2S_DSIN      35
#define I2S_MCLK      0

/* GPIOs for SPI ES8388 - need check if used */
//#define ES_SPI_MOSI      23
//#define ES_SPI_MISO      19
//#define ES_SPI_SCK       18
//#define ES_SPI_CS        5 

/* I2C GPIOs for control ES8388 */
#define ES8388_SCL    32
#define ES8388_SDA    33

/* Amplifier enable PIN - eternal AMP on ESP32-A1S Kit */
//#define GPIO_PA_EN       21   /* Amplifier GPIO */
//#define GPIO_PA_LEVEL    HIGH /* Amplifier enable level */
//#define SD_DETECT        34 // ?
//#define HEP_DETECT       39 // ?
/* The MUTE_PIN is inversed GPIO_PA_EN and implemented in YORADIO */
#define MUTE_PIN        21   /*  MUTE Pin */
#define MUTE_VAL        LOW  /*  Write this to MUTE_PIN when player is stopped */

/*****************************************************************************/
/*                                                                           */
/*                     ES8388 AUDIO CODEC Settings                           */
/*                                                                           */
/*****************************************************************************/

#define ES8388_MAIN_VOLUME 75 // 0-191 in dBm (from 0 to -96dBm in 0.5dBm steps) 
#define ES8388_OUT1_VOLUME 25 // 0-33 in dBm (from -45 to 4.5 dBm in 1.5dBm steps.0dBm=30 ) works for amp
#define ES8388_OUT2_VOLUME 25 // 0-33 in dBm (from -45 to 4.5 dBm in 1.5dBm steps.0dBm=30 ) HeadPhones?
#define ES8388_MAIN_MUTE   false  // This works on Audio Kit
#define ES8388_OUT1_MUTE   false // not muted
#define ES8388_OUT2_MUTE   false // not muted Amplifier on Audio Kit





//#define ENC2_BTNR			22  //CLK
//#define ENC2_BTNL			14  //DT 
//#define ENC2_BTNB			13  //SW 
//#define ENC2_INTERNALPULLUP			true

//#define IR_PIN	13


#endif