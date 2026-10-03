#pragma once
#include <stdint.h>

#define ES8388_ADDR 0x10

/* Chip control and power management */
#define ES8388_CONTROL1      0x00
#define ES8388_CONTROL2      0x01
#define ES8388_CHIPPOWER     0x02
#define ES8388_ADCPOWER      0x03
#define ES8388_DACPOWER      0x04
#define ES8388_CHIPLOPOW1    0x05
#define ES8388_CHIPLOPOW2    0x06
#define ES8388_ANAVOLMANAG   0x07
#define ES8388_MASTERMODE    0x08

/* ADC */
#define ES8388_ADCCONTROL1   0x09
#define ES8388_ADCCONTROL2   0x0a
#define ES8388_ADCCONTROL3   0x0b
#define ES8388_ADCCONTROL4   0x0c
#define ES8388_ADCCONTROL5   0x0d
#define ES8388_ADCCONTROL6   0x0e
#define ES8388_ADCCONTROL7   0x0f
#define ES8388_ADCCONTROL8   0x10
#define ES8388_ADCCONTROL9   0x11
#define ES8388_ADCCONTROL10  0x12
#define ES8388_ADCCONTROL11  0x13
#define ES8388_ADCCONTROL12  0x14
#define ES8388_ADCCONTROL13  0x15
#define ES8388_ADCCONTROL14  0x16

/* DAC */
#define ES8388_DACCONTROL1   0x17
#define ES8388_DACCONTROL2   0x18
#define ES8388_DACCONTROL3   0x19
#define ES8388_DACCONTROL4   0x1a
#define ES8388_DACCONTROL5   0x1b
#define ES8388_DACCONTROL6   0x1c
#define ES8388_DACCONTROL7   0x1d
#define ES8388_DACCONTROL8   0x1e
#define ES8388_DACCONTROL9   0x1f
#define ES8388_DACCONTROL10  0x20
#define ES8388_DACCONTROL11  0x21
#define ES8388_DACCONTROL12  0x22
#define ES8388_DACCONTROL13  0x23
#define ES8388_DACCONTROL14  0x24
#define ES8388_DACCONTROL15  0x25
#define ES8388_DACCONTROL16  0x26
#define ES8388_DACCONTROL17  0x27
#define ES8388_DACCONTROL18  0x28
#define ES8388_DACCONTROL19  0x29
#define ES8388_DACCONTROL20  0x2a
#define ES8388_DACCONTROL21  0x2b
#define ES8388_DACCONTROL22  0x2c
#define ES8388_DACCONTROL23  0x2d
#define ES8388_DACCONTROL24  0x2e
#define ES8388_DACCONTROL25  0x2f
#define ES8388_DACCONTROL26  0x30
#define ES8388_DACCONTROL27  0x31
#define ES8388_DACCONTROL28  0x32
#define ES8388_DACCONTROL29  0x33
#define ES8388_DACCONTROL30  0x34

class ES8388
{
    bool identify(int sda, int scl, uint32_t frequency);

public:

    enum ES8388_OUT
    {
        ES_MAIN, // DAC digital volume, attenuates both outputs
        ES_OUT1, // LOUT1/ROUT1 (regs 46/47) = on-board speaker amp on the A1S
        ES_OUT2  // LOUT2/ROUT2 (regs 48/49) = headphone amp
    };

    /* What the analog line input contributes to the outputs. The line jack is
       summed inside the codec at regs 39/42, which sit DOWNSTREAM of the DAC
       digital volume (regs 26/27) - so the three modes differ only in whether
       the line-in bit is set and whether the DAC path is muted:

         OFF   line-in absent. Radio only.
         MIX   line-in summed with the radio, ratio set by gain_db.
         ONLY  line-in present and the DAC digital path muted, which silences
               the radio while line-in keeps playing. Line-in joins after the
               DAC, so muting the digital path cannot touch it.

       Note the main-page volume does not reach line-in for the same reason:
       it is a digital attenuator and line-in enters downstream of it. */
    enum ES8388_LINEIN
    {
        LINEIN_OFF  = 0,
        LINEIN_MIX  = 1,
        LINEIN_ONLY = 2
    };

    bool begin(int sda = -1, int scl = -1, uint32_t frequency = 400000U);

    /* Digital volume: 0..192, where 192 is 0dB and 0 is -96dB (0.5dB steps) */
    void volume(const ES8388_OUT out, const uint8_t vol);
    /* Analog output volume: 0..33, where 30 is 0dB and 33 is +4.5dB (1.5dB steps) */
    void volume_l(const ES8388_OUT out, const uint8_t vol);
    void volume_r(const ES8388_OUT out, const uint8_t vol);

    void mute(const ES8388_OUT out, const bool muted);

    /* DAC Control 7 (0x1d) field accessors */
    void stereo_eff(const uint8_t eff); // 0..7, 0 = off, 7 = strongest
    void mono(const bool on);
    void vpp_scale(const uint8_t scale); // 0..3, see datasheet 6.3.7

    /* Chip Control 2 (0x1c): de-emphasis, click-free, channel inversion */
    void deemphasis(const uint8_t mode);   // 0 off, 1=32k, 2=44.1k, 3=48k
    void click_free(const bool on);
    void channel_invert(const bool invertL, const bool invertR);

    /* DAC soft volume ramp: 0.5dB per N LRCK, n = 0..3 -> 4/32/64/128 */
    void volume_ramp(const uint8_t n);

    /* Soft ramp enable (bit 5) together with its rate. volume_ramp() only sets
       the rate, so use this to actually switch the ramp on/off. */
    void soft_ramp(const bool on, const uint8_t rate);

    /* Analog output impedance: false = 1.5k (default), true = 40k */
    void output_impedance(const bool high);

    /* Line input (LIN1/LIN2) contribution to the output mixers */
    void line_in_mix(const bool on, const int8_t gain_db);
    /* As above, with the three-way off/mix/line-only routing. */
    void line_in_mix_mode(const uint8_t mode, const int8_t gain_db);

    /* Mic PGA, 0..8 -> 0..+24dB in 3dB steps */
    void mic_gain(const uint8_t gain);
    /* Mic input select: 0 = LIN1&RIN1, 1 = LIN2&RIN2, 2 = differential */
    void mic_input(const uint8_t sel);
    /* Mic bias (MBIAS) output */
    void mic_bias(const bool on);
    /* Power up the ADC and analog inputs (playback only leaves this off) */
    void adc_power(const bool on);
    /* Power the analog INPUT BUFFERS only (register 3 bits 7:6, PdnAINL/R).
       Separate from adc_power() because the two are independent: line-in runs
       through these buffers into the output mixers and never touches the ADCs.
       Leaving them powered down is what silently makes line-in produce nothing.
       Register 3's default has both bits SET, i.e. powered down. */
    void analog_input_power(const bool on);
    /* Which line-in pair feeds the output mixers (register 38).
       0 = LIN1/RIN1, 1 = LIN2/RIN2. On the ESP32-A1S the module's
       LINEINL/LINEINR pins reach LIN2/RIN2 - measured, since the carrier
       schematic treats the module as a black box. */
    void line_input_select(const uint8_t sel);

    /* Low power / standby, per user guide 11.5 and 11.6 */
    void standby();
    void wake();

    bool write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data);
    bool read_reg(uint8_t slave_add, uint8_t reg_add, uint8_t &data);
};
