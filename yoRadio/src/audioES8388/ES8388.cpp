#include <Arduino.h>
#include "ES8388.h"
#include <Wire.h>

/* Register bit helpers for read-modify-write access */
static inline uint8_t rmw_get(uint8_t reg, uint8_t mask) { return reg & mask; }
#define BIT_SET(reg, mask)   ((reg) |= (mask))
#define BIT_CLR(reg, mask)   ((reg) &= ~(mask))

bool ES8388::write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data)
{
    Wire.beginTransmission(slave_add);
    Wire.write(reg_add);
    Wire.write(data);
    return Wire.endTransmission() == 0;
}

bool ES8388::read_reg(uint8_t slave_add, uint8_t reg_add, uint8_t &data)
{
    Wire.beginTransmission(slave_add);
    Wire.write(reg_add);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)slave_add, (uint8_t)1, true);
    if (Wire.available() >= 1)
    {
        data = Wire.read();
        return true;
    }
    data = 0;
    return false;
}

/* Read-modify-write helper: fetch, clear mask, set/clear value bits, store. */
static bool rmw(ES8388 &e, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t v;
    if (!e.read_reg(ES8388_ADDR, reg, v)) return false;
    v = (uint8_t)((v & ~mask) | (value & mask));
    return e.write_reg(ES8388_ADDR, reg, v);
}

bool ES8388::begin(int sda, int scl, uint32_t frequency)
{
    if (identify(sda, scl, frequency) == false) return false;

    bool res = true;

    /* --- analog power up, per ES8388 user guide 11.3 (ES8388_DAC) --- */
    res &= write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x58); // power down whole chip analog
    res &= write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50); // power up whole chip analog
    res &= write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF3); // stop STM and DLL, power down DAC&ADC vref
    res &= write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF0); // power up DAC&ADC vref
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); // ADC and DAC share LRCK, DAC LRCK is the chip LRCK

    /* SameFs=1, DACMCLK is the chip master clock, EnRef=1 (internal reference on).
       The old code used 0x12 here, which disabled the reference and pointed the
       master clock at the powered-down ADC domain. */
    res &= write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x36);

    res &= write_reg(ES8388_ADDR, ES8388_MASTERMODE, 0x00); // I2S slave: BCLK/LRCK come from the ESP32

    res &= write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x00); // power up DAC, outputs off for now
    res &= write_reg(ES8388_ADDR, ES8388_CHIPLOPOW1, 0x00); // low power setting
    res &= write_reg(ES8388_ADDR, ES8388_CHIPLOPOW2, 0xC3); // low power setting

    /* DAC serial port: 16-bit I2S, MCLK/Fs = 256. We keep 16-bit because the
       yoRadio I2S stream is 16-bit; the guide's 0x00 (24-bit) is for 24-bit streams. */
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);

    /* DAC Control 3: soft volume ramp on, ramp rate 0.5dB per 4 LRCK. The old
       code left soft ramp disabled which pops on every volume change. */
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x22);

    /* Digital volume 0dB on both channels (reg 26/27 are reversed: 0 = loudest) */
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL4, 0x00);
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL5, 0x00);

    /* Output mixers: LDAC/RDAC to LOUT/ROUT, line-in present but not mixed in.
       LMIXSEL/RMIXSEL select LIN1/RIN1. The old code used 0x90 (line-in at 0dB)
       and an accidental 0x1B for the source select.
       0xB8 = LD2LO=1, LI2LO=0 (off), LI2LOVOL=111 which is -15dB, not the -12dB
       an earlier comment here claimed. It is transient either way:
       Player::applyEs8388Settings() rewrites both mixers immediately afterwards
       with the stored mode and gain. */
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00); // LMIXSEL=LIN1, RMIXSEL=RIN1
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0xB8); // LD2LO=1, line-in off at -15dB
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0xB8); // RD2RO=1, line-in off at -15dB

    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00); // VROI=0: 1.5k output resistance

    /* Startup FSM and DLL. This powers the ADC digital path down for playback
       and is what the old code (0x00) was missing. */
    res &= write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xAA);
    delay(500);

    /* Analog output volume. On the ESP32-A1S carrier LOUT1/ROUT1 feed the
       speaker amps (SPOLP/N -> U4/U5 -> J3/J4, hardware-enabled by GPIO21) and
       LOUT2/ROUT2 feed the headphone jack (HPOUTL/R -> J2, no hardware enable).
       30 == 0dB. */
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1E); // LOUT1
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1E); // ROUT1
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL26, 0x1E); // LOUT2
    res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL27, 0x1E); // ROUT2

    res &= write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3C); // enable LOUT1&2, ROUT1&2
    res &= write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);  // power down ADC (playback only)

    /* MCLK comes from the ESP32 CLK_OUT1 on GPIO0 */
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    WRITE_PERI_REG(PIN_CTRL, 0xFFF0);

    return res;
}

/**
 * @brief Set the digital volume, which attenuates both outputs equally.
 *        0..192, 192 = 0dB, 0 = -96dB in 0.5dB steps. The register is reversed
 *        (lowest value is loudest).
 */
void ES8388::volume(const ES8388_OUT out, const uint8_t vol)
{
    if (out != ES_MAIN) return; // only ES_MAIN uses the digital volume regs

    uint16_t v = vol > 192 ? 192 : vol;
    uint8_t regval = (uint8_t)(192 - v);
    write_reg(ES8388_ADDR, ES8388_DACCONTROL4, regval);
    write_reg(ES8388_ADDR, ES8388_DACCONTROL5, regval);
}

/**
 * @brief Set the analog output volume of LOUT1/ROUT1 or LOUT2/ROUT2.
 *        0..33, 30 = 0dB, 33 = +4.5dB in 1.5dB steps, 0 = -45dB.
 *        The field is only 6 bits wide, so values above 33 are clamped instead
 *        of bleeding into the reserved bits (the old code wrote 0..100 raw).
 */
void ES8388::volume_l(const ES8388_OUT out, const uint8_t vol)
{
    uint8_t regaddr;
    switch (out) {
    case ES_OUT1: regaddr = ES8388_DACCONTROL24; break;
    case ES_OUT2: regaddr = ES8388_DACCONTROL26; break;
    default: return;
    }
    uint8_t v = vol > 33 ? 33 : vol;
    write_reg(ES8388_ADDR, regaddr, (uint8_t)(v & 0x3F));
}

void ES8388::volume_r(const ES8388_OUT out, const uint8_t vol)
{
    uint8_t regaddr;
    switch (out) {
    case ES_OUT1: regaddr = ES8388_DACCONTROL25; break;
    case ES_OUT2: regaddr = ES8388_DACCONTROL27; break;
    default: return;
    }
    uint8_t v = vol > 33 ? 33 : vol;
    write_reg(ES8388_ADDR, regaddr, (uint8_t)(v & 0x3F));
}

/**
 * @brief (Un)mute one of the two analog outputs, or the main DAC digital path.
 *        For the analog outputs this is really an output-enable, not an
 *        attenuation; ES_MAIN uses the DAC mute bit.
 */
void ES8388::mute(const ES8388_OUT out, const bool muted)
{
    uint8_t reg_addr;
    uint8_t mask;
    uint8_t val;

    switch (out) {
    case ES_OUT1:
        reg_addr = ES8388_DACPOWER;
        mask = (3 << 4); // LOUT1, ROUT1 enable
        val = muted ? 0 : mask;
        break;
    case ES_OUT2:
        reg_addr = ES8388_DACPOWER;
        mask = (3 << 2); // LOUT2, ROUT2 enable
        val = muted ? 0 : mask;
        break;
    case ES_MAIN:
    default:
        reg_addr = ES8388_DACCONTROL3;
        mask = 1 << 2; // DACMute
        val = muted ? mask : 0;
        break;
    }
    rmw(*this, reg_addr, mask, val);
}

/* DAC Control 7 (0x1d): ZeroL(7) ZeroR(6) Mono(5) SE(4:2) Vpp_scale(1:0) */

void ES8388::stereo_eff(const uint8_t eff)
{
    uint8_t e = eff > 7 ? 7 : eff; // clamp: an 8 would spill into the Mono bit
    rmw(*this, ES8388_DACCONTROL7, (7 << 2), (uint8_t)(e << 2));
}

void ES8388::mono(const bool on)
{
    rmw(*this, ES8388_DACCONTROL7, 1 << 5, on ? (1 << 5) : 0);
}

void ES8388::vpp_scale(const uint8_t scale)
{
    uint8_t s = scale > 3 ? 3 : scale;
    rmw(*this, ES8388_DACCONTROL7, 0x03, s);
}

/* DAC Control 6 (0x1c): DeemphasisMode(7:6) DAC_invL(5) DAC_invR(4) ClickFree(3) */

void ES8388::deemphasis(const uint8_t mode)
{
    uint8_t m = mode > 3 ? 3 : mode;
    rmw(*this, ES8388_DACCONTROL6, (3 << 6), (uint8_t)(m << 6));
}

void ES8388::click_free(const bool on)
{
    rmw(*this, ES8388_DACCONTROL6, 1 << 3, on ? (1 << 3) : 0);
}

void ES8388::channel_invert(const bool invertL, const bool invertR)
{
    uint8_t v = 0;
    if (invertL) v |= (1 << 5);
    if (invertR) v |= (1 << 4);
    rmw(*this, ES8388_DACCONTROL6, (3 << 4), v);
}

/* DAC Control 3 (0x19): DACRampRate(7:6) DACSoftRamp(5) DACLeR(3) DACMute(2) */

void ES8388::volume_ramp(const uint8_t n)
{
    uint8_t r = n > 3 ? 3 : n;
    rmw(*this, ES8388_DACCONTROL3, (3 << 6), (uint8_t)(r << 6));
}

/* Soft ramp is a separate enable bit from the rate, so the rate setter alone can
   never switch it off. The two had to be combined or the "soft volume ramp"
   setting was cosmetic: it changed the rate while bit 5 stayed set by the init
   sequence, so the ramp was on regardless. */
void ES8388::soft_ramp(const bool on, const uint8_t rate)
{
    uint8_t r = rate > 3 ? 3 : rate;
    uint8_t v = (uint8_t)((r << 6) | (on ? (1 << 5) : 0));
    rmw(*this, ES8388_DACCONTROL3, (3 << 6) | (1 << 5), v);
}

/* DAC Control 23 (0x2d): VROI(4) - output impedance reference */

void ES8388::output_impedance(const bool high)
{
    rmw(*this, ES8388_DACCONTROL23, 1 << 4, high ? (1 << 4) : 0);
}

/**
 * @brief Mix the line input (LIN1/LIN2) into the output mixers.
 * @param on   enable the line-in path in both output mixers
 * @param gain_db -15..+6 in 3dB steps, mapped to LI2LOVOL/RI2ROVOL
 */
void ES8388::line_in_mix(const bool on, const int8_t gain_db)
{
    line_in_mix_mode(on ? LINEIN_MIX : LINEIN_OFF, gain_db);
}

/**
 * @brief Route the line input as off, mixed with the DAC, or on its own.
 *
 * Reg 39 (DACCONTROL17) and reg 42 (DACCONTROL20) each carry
 *   LD2LO/RD2RO (7) LI2LO/RI2RO (6) LI2LOVOL/RI2ROVOL (5:3)
 * LD2LO stays 1 so the DAC keeps its route; LI2LO is what adds line-in on top.
 * That is why this can never be "line-in instead of the radio" on its own -
 * see line_in_mix_mode()'s enum comment for how LINEIN_ONLY is achieved.
 *
 * The exclusive case is not handled here: muting the DAC digital path is the
 * caller's job, because ES_MAIN is a shared control and Player owns it.
 */
void ES8388::line_in_mix_mode(const uint8_t mode, const int8_t gain_db)
{
    // LI2LOVOL / RI2LOVOL: 000=+6dB .. 111=-15dB
    int8_t g = gain_db;
    if (g > 6) g = 6;
    if (g < -15) g = -15;
    uint8_t code = (uint8_t)((6 - g) / 3); // 6dB->0 ... -15dB->7
    uint8_t bits = (uint8_t)(code << 3);
    bool on = mode != LINEIN_OFF;

    // reg 39: LD2LO(7) LI2LO(6) LI2LOVOL(5:3). Keep LD2LO=1 (DAC always routed).
    uint8_t l = 0;
    l |= (1 << 7);
    if (on) l |= (1 << 6);
    l |= bits;
    write_reg(ES8388_ADDR, ES8388_DACCONTROL17, l);

    // reg 42: RD2RO(7) RI2RO(6) RI2ROVOL(5:3)
    uint8_t r = 0;
    r |= (1 << 7);
    if (on) r |= (1 << 6);
    r |= bits;
    write_reg(ES8388_ADDR, ES8388_DACCONTROL20, r);
}

/* ADC Control 1 (0x09): MicAmpL(7:4) MicAmpR(3:0), 0..8 = 0..+24dB in 3dB steps */
void ES8388::mic_gain(const uint8_t gain)
{
    uint8_t g = gain > 8 ? 8 : gain;
    write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, (uint8_t)((g << 4) | g));
}

/* ADC Control 2 (0x0a): LINSEL(7:4) RINSEL(3:0) */
void ES8388::mic_input(const uint8_t sel)
{
    uint8_t v;
    switch (sel) {
    case 1:  v = 0x50; break; // LIN2 & RIN2
    case 2:  v = 0xF0; break; // differential (LIN1-RIN1), needs ADCCONTROL3 = 0x02
    default: v = 0x00; break; // LIN1 & RIN1
    }
    write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, v);
}

/* ADC Power Management (0x03): PdnAINL(7) PdnAINR(6) PdnADCL(5) PdnADCR(4) PdnMICB(3) ... */
void ES8388::mic_bias(const bool on)
{
    rmw(*this, ES8388_ADCPOWER, 1 << 3, on ? 0 : (1 << 3));
}

void ES8388::adc_power(const bool on)
{
    // on: enable analog inputs and both ADCs, no mic bias, int1 in low power
    // off: everything powered down (playback only)
    write_reg(ES8388_ADDR, ES8388_ADCPOWER, on ? 0x09 : 0xFF);
}

/**
 * @brief Low power / standby, per user guide 11.5. Call when stopped to cut the
 *        codec's 7mW+ idle draw. Pair with wake().
 */
void ES8388::standby()
{
    write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0xE6);
    write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xFC);
    write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);
    write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xC0);
    write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x90);
}

/**
 * @brief Resume from standby, per user guide 11.6.
 */
void ES8388::wake()
{
    write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);
    write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);
    write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);
    write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3C);
    write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0xE2);
}

/**
 * @brief Test if a device with the ES8388 I2C address is on the bus.
 */
bool ES8388::identify(int sda, int scl, uint32_t frequency)
{
    Wire.begin(sda, scl, frequency);
    Wire.beginTransmission(ES8388_ADDR);
    return Wire.endTransmission() == 0;
}
