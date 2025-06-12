#include <Arduino.h>
#include "ES8388.h"
#include <Wire.h>


bool ES8388::write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data)
{
    Wire.beginTransmission(slave_add);
    Wire.write(reg_add);
    Wire.write(data);
    return Wire.endTransmission() == 0;
}

bool ES8388::read_reg(uint8_t slave_add, uint8_t reg_add, uint8_t &data)
{
    bool retval = false;
    Wire.beginTransmission(slave_add);
    Wire.write(reg_add);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)slave_add, (uint8_t)1, true);
    if (Wire.available() >= 1)
    {
        data = Wire.read();
        retval = true;
    }
    return retval;
}

bool ES8388::begin(int sda, int scl, uint32_t frequency)
{
    bool res = identify(sda, scl, frequency);

    if (res == true)
    {

        /* mute DAC during setup, power up all systems, slave mode */
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04); //ok
        res &= write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50); //ok
        res &= write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00); //ok

        // Disable the internal DLL to improve 8K sample rate
        res &= write_reg(ES8388_ADDR, 0x35, 0xA0);
        res &= write_reg(ES8388_ADDR, 0x37, 0xD0);
        res &= write_reg(ES8388_ADDR, 0x39, 0xD0);

        res &= write_reg(ES8388_ADDR, ES8388_MASTERMODE, 0x00); //check

/************************************************************************************************


    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);  // 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp
    // Chip Control and Power Management 
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
    res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);  // normal all and power up all

    // Disable the internal DLL to improve 8K sample rate
    res |= es_write_reg(ES8388_ADDR, 0x35, 0xA0);
    res |= es_write_reg(ES8388_ADDR, 0x37, 0xD0);
    res |= es_write_reg(ES8388_ADDR, 0x39, 0xD0);

    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, cfg->i2s_iface.mode);  // CODEC IN I2S SLAVE MODE

    // dac
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xC0);  // disable DAC and disable Lout/Rout/1/2
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);  // Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
    // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0);  //LPVrefBuf=0,Pdn_ana=0
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);   // 1a 0x18:16bit iis , 0x00:24
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);   // DACFsMode,SINGLE SPEED; DACFsRatio,256
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00);  // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90);  // only left DAC to left mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90);  // only right DAC to right mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);  // set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);  // vroi=0

    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1E);  // Set L1 R1 L2 R2 volume. 0x00: -30dB, 0x1E: 0dB, 0x21: 3dB
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1E);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL26, 0);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL27, 0);
    // res |= es8388_set_adc_dac_volume(ES_MODULE_DAC, 0, 0);       // 0db
    int tmp = 0;
    if (AUDIO_HAL_DAC_OUTPUT_LINE2 == cfg->dac_output) {
        tmp = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_ROUT1;
    } else if (AUDIO_HAL_DAC_OUTPUT_LINE1 == cfg->dac_output) {
        tmp = DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT2;
    } else {
        tmp = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
    }
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, tmp);  // 0x3c Enable DAC and Enable Lout/Rout/1/2
    // adc 
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0xbb);  // MIC Left and Right channel PGA gain
    tmp = 0;
    if (AUDIO_HAL_ADC_INPUT_LINE1 == cfg->adc_input) {
        tmp = ADC_INPUT_LINPUT1_RINPUT1;
    } else if (AUDIO_HAL_ADC_INPUT_LINE2 == cfg->adc_input) {
        tmp = ADC_INPUT_LINPUT2_RINPUT2;
    } else {
        tmp = ADC_INPUT_DIFFERENCE;
    }
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, tmp);  // 0x00 LINSEL & RINSEL, LIN1/RIN1 as ADC Input; DSSEL,use one DS Reg11; DSR, LINPUT1-RINPUT1
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x02);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0c);  // 16 Bits length and I2S serial audio data format
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);  // ADCFsMode,singel SPEED,RATIO=256
    // ALC for Microphone
    res |= es8388_set_adc_dac_volume(ES_MODULE_ADC, 0, 0);    // 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x09);  // Power on ADC, enable LIN&RIN, power off MICBIAS, and set int1lp to low power mode


*************************************************************************************************/



        /* power up DAC and enable LOUT1+2 / ROUT1+2, ADC sample rate = DAC sample rate */
        res &= write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xc0); // 3c or fc
        //res &= write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xC0);  // disable DAC and disable Lout/Rout/1/2
        res &= write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12); //ok

        /* DAC I2S setup: 16 bit word length, I2S format; MCLK / Fs = 256*/
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18); // OK
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02); // OK

        /* DAC to output route mixer configuration: ADC MIX TO OUTPUT */
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x1B); // ?
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90); // OK
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90); // OK

        /* DAC and ADC use same LRCK, enable MCLK input; output resistance setup */
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); //OK
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00); //OK

        /* set LOUT1 / ROUT1 volume: 0dB (unattenuated) */
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1e);
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1e);

        /* set LOUT2 / ROUT2 volume: 0dB (unattenuated) */
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL26, 0x1e);
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL27, 0x1e);

        /* power up DAC and enable LOUT1+2 / ROUT1+2, ADC sample rate = DAC sample rate */
        res &= write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c); // 3c or fc

        /* DAC volume control: 0dB (maximum, unattenuated)  */
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL5, 0x00); // ?
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL4, 0x00); // ?

        /* Disable ADC for both channels */
        res &= write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xff); // ?

        /* power up and enable DAC; power up ADC (no MIC bias) */
//        res &= write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c); // OK
//        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x00);
//        res &= write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);
        
        /* set up MCLK) */
		PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
        WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
    }

    return res;
}

/**
 * @brief (un)mute one of the two outputs or main dac output of the ES8388 by switching of the output register bits. Does not really mute the selected output, causes an attenuation. 
 * hence should be used in conjunction with appropriate volume setting. Main dac output mute does mute both outputs
 * 
 * @param out
 * @param muted
 */
void ES8388::mute(const ES8388_OUT out, const bool muted)
{
    uint8_t reg_addr;
    uint8_t mask_mute;
    uint8_t mask_val;

    switch (out)
    {
    case ES_OUT1:
        reg_addr = ES8388_DACPOWER;
        mask_mute = (3 << 4);
        mask_val = muted ? 0 : mask_mute;
        break;
    case ES_OUT2:
        reg_addr = ES8388_DACPOWER;
        mask_mute = (3 << 2);
        mask_val = muted ? 0 : mask_mute;
        break;
    case ES_MAIN:
    default:
        reg_addr = ES8388_DACCONTROL3;
        mask_mute = 1 << 2;
        mask_val = muted ? mask_mute : 0;
        break;
    }
//    mask_val = muted ? mask_mute : 0;


    uint8_t reg;
    if (read_reg(ES8388_ADDR, reg_addr, reg))
    {
        reg = (reg & ~mask_mute) | (mask_val & mask_mute);
        write_reg(ES8388_ADDR, reg_addr, reg);
    }
}

/**
 * @brief Set volume gain for the main dac, or for one of the two output channels. Final gain = main gain + out channel gain 
 * 
 * @param out which gain setting to control
 * @param vol 0-100 (100 is max)
 */
void ES8388::volume(const ES8388_OUT out, const uint8_t vol)
{
    const uint32_t max_vol = 100; // max input volume value

    const int32_t max_vol_val = out == ES8388_OUT::ES_MAIN ? 96 : 0x21; // max register value for ES8388 out volume

    uint8_t vol_val = vol > max_vol ? max_vol_val : (max_vol_val * vol) / max_vol;

    // main dac volume control is reverse scale (lowest value is loudest)
    // hence we reverse the calculated value
    if (out == ES_MAIN)
    {
        vol_val = max_vol_val - vol_val;
    }
    else
    {
        vol_val = vol;
    }

    uint8_t lreg = 0, rreg = 0;

    switch (out)
    {
    case ES_MAIN:
        lreg = ES8388_DACCONTROL4;
        rreg = ES8388_DACCONTROL5;
        break;
    case ES_OUT1:
        lreg = ES8388_DACCONTROL24;
        rreg = ES8388_DACCONTROL25;
//        vol_val = vol;
        break;
    case ES_OUT2:
        lreg = ES8388_DACCONTROL26;
        rreg = ES8388_DACCONTROL27;
//        vol_val = vol;
        break;
    }


    write_reg(ES8388_ADDR, lreg, vol_val);
    write_reg(ES8388_ADDR, rreg, vol_val);
}

/**
 * @brief Test if device with I2C address for ES8388 is connected to the I2C bus 
 * 
 * @param sda which pin to use for I2C SDA
 * @param scl which pin to use for I2C SCL
 * @param frequency which frequency to use as I2C bus frequency
 * @return true device was found
 * @return false device was not found
 */
bool ES8388::identify(int sda, int scl, uint32_t frequency)
{
    Wire.begin(sda, scl, frequency);
    Wire.beginTransmission(ES8388_ADDR);
    return Wire.endTransmission() == 0;
}
