/*
 * ES8388 DEQ (digital equalizer) coefficient reverse-engineering harness.
 *
 * WHY THIS EXISTS
 * ---------------
 * The ES8388 has a 2-band parametric EQ in registers 30..37 (Shelving_a and
 * Shelving_b, 30 bits each). The coefficient encoding is NOT documented: the
 * datasheet only publishes the flat default
 *     {5'h0f, 5'h1f, 5'h0f, 5'h1f, 5'h0f, 5'h1f}
 * and the user guide says "Everest Semiconductor will provide equalizer
 * calculator" - which is not in either PDF. We also know from the user guide
 * that the 2 bands are bass OR treble, not both at once.
 *
 * So we discover the encoding by ear. This firmware plays a logarithmic sine
 * sweep and alternates between a candidate coefficient set and the flat
 * default, separated by a short silence, so any change is directly audible
 * and can be attributed to a frequency band.
 *
 * REGISTER PACKING (from the datasheet bit assignments)
 *   Shelving_a[29:24] -> reg 30, bits 5:0   (6 bits, not 5)
 *   Shelving_a[23:16] -> reg 31, bits 7:0
 *   Shelving_a[15:8]  -> reg 32, bits 7:0
 *   Shelving_a[7:0]   -> reg 33, bits 7:0
 *   6 + 8 + 8 + 8 = 30 bits.
 *   Filter B is the same layout in regs 34..37.
 *   Flat default for both filters: 0x1F, 0xF7, 0xFD, 0xFF
 *
 * SERIAL CONSOLE (115200)
 *   f            restore both filters to the flat default
 *   fa <8 hex>   set filter A raw, e.g. "fa 1FF7FDFF"
 *   fb <8 hex>   set filter B raw
 *   s <0..5> <v> set one 5-bit field of filter A (fields 0..5, value 0..31)
 *   g <0..3> <v> same for filter B
 *   d            dump registers 29..37
 *   A            toggle A/B sweep on/off
 *   ?            help
 *
 * A "field" here is one of the six 5-bit groups in the datasheet default
 * {0f,1f,0f,1f,0f,1f}, numbered 0..5 from the most significant.
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#include "ES8388.h"

static const int PIN_BCLK = 27;
static const int PIN_LRC  = 25;
static const int PIN_DOUT = 26;
static const int PIN_MCLK = 0;
static const int PIN_SDA  = 33;
static const int PIN_SCL  = 32;

static const uint32_t SAMPLE_RATE = 48000;
static const int      SWEEP_SECONDS = 12;
static const float    F_START = 20.0f;
static const float    F_END   = 20000.0f;

static const uint8_t FLAT[4] = { 0x1F, 0xF7, 0xFD, 0xFF };

// 30-bit words, MSB first: the six 5-bit groups from the datasheet default.
static uint32_t g_a = 0x1FF7FDFFUL & 0x3FFFFFFFUL;
static uint32_t g_b = 0x1FF7FDFFUL & 0x3FFFFFFFUL;

static i2s_port_t   g_port = I2S_NUM_0;
static bool         g_ab = false;        // A/B sweep enabled
static uint32_t     g_phase = 0;         // Q16 fixed point phase
static uint32_t     g_sample = 0;
static volatile bool g_running = false;
static ES8388       g_es;                 // one shared codec object

// A/B cycle state: we play the candidate, then flat, separated by silence.
static bool         g_playingFlat = false;
static uint32_t     g_silenceLeft = 0;    // samples of silence remaining
static const uint32_t SWEEP_SAMPLES = (uint32_t)SAMPLE_RATE * SWEEP_SECONDS;
static const uint32_t GAP_SAMPLES   = (uint32_t)(SAMPLE_RATE * 0.4f);

static uint8_t g_buf[1024];

static float sweep_freq(uint32_t sample)
{
    float t = (float)sample / ((float)SAMPLE_RATE * (float)SWEEP_SECONDS);
    if (t > 1.0f) t = 1.0f;
    return F_START * powf(F_END / F_START, t);
}

// Phase increment for the next sample, Q16.
static uint32_t next_inc()
{
    float f = sweep_freq(g_sample);
    float inc = 6.2831853f * f / (float)SAMPLE_RATE;
    g_phase += (uint32_t)(inc * 65536.0f);
    g_sample++;
    return g_phase;
}

static void fill_tone(uint8_t *buf, size_t bytes, bool silence)
{
    size_t n = bytes / 4; // 16-bit stereo
    int16_t *p = (int16_t *)buf;
    for (size_t i = 0; i < n; i++) {
        int16_t v = 0;
        if (!silence) {
            uint32_t inc = next_inc();
            int32_t s = (int32_t)(((int64_t)(inc >> 15) * 16000) >> 16);
            if (s > 32767) s = 32767;
            if (s < -32768) s = -32768;
            v = (int16_t)s;
        }
        p[i * 2] = v;
        p[i * 2 + 1] = v;
    }
}

static void write_filter(ES8388 &es, bool which, uint32_t w)
{
    uint8_t b0 = (uint8_t)((w >> 24) & 0x3F); // reg30: 6 bits
    uint8_t b1 = (uint8_t)((w >> 16) & 0xFF);
    uint8_t b2 = (uint8_t)((w >> 8) & 0xFF);
    uint8_t b3 = (uint8_t)(w & 0xFF);
    uint8_t base = which ? 34 : 30;
    es.write_reg(ES8388_ADDR, base + 0, b0);
    es.write_reg(ES8388_ADDR, base + 1, b1);
    es.write_reg(ES8388_ADDR, base + 2, b2);
    es.write_reg(ES8388_ADDR, base + 3, b3);
}

static void apply_flat(ES8388 &es)
{
    write_filter(es, false, 0x1FF7FDFFUL);
    write_filter(es, true,  0x1FF7FDFFUL);
}

// Set one of the six 5-bit groups. idx 0 = bits 29..25, idx 5 = bits 4..0.
static uint32_t set_field(uint32_t w, int idx, uint32_t v)
{
    v &= 0x1F;
    int shift = 25 - idx * 5;
    uint32_t mask = 0x1FUL << shift;
    w &= ~mask;
    w |= (v << shift);
    return w & 0x3FFFFFFFUL;
}

static void dump(ES8388 &es)
{
    Serial.println("reg 29 (SE/mono/vpp) / 30..37 (EQ coefficients):");
    for (int r = 29; r <= 37; r++) {
        uint8_t v = 0;
        es.read_reg(ES8388_ADDR, r, v);
        Serial.printf("  0x%02X = 0x%02X (%3u)  %s\n", r, v, v,
                      (r >= 30 && r <= 33) ? "Shelving_a" :
                      (r >= 34 && r <= 37) ? "Shelving_b" : "");
    }
    Serial.printf("filter A word = 0x%06lX\n", (unsigned long)g_a);
    Serial.printf("filter B word = 0x%06lX\n", (unsigned long)g_b);
}

static void help()
{
    Serial.println();
    Serial.println("ES8388 EQ coefficient lab");
    Serial.println("  f            restore flat default on both filters");
    Serial.println("  fa <8hex>    set filter A raw, e.g. 'fa 1FF7FDFF'");
    Serial.println("  fb <8hex>    set filter B raw");
    Serial.println("  s <0..5> <v> set 5-bit field of filter A (v 0..31)");
    Serial.println("  g <0..5> <v> set 5-bit field of filter B (v 0..31)");
    Serial.println("  d            dump registers 29..37");
    Serial.println("  A            toggle A/B sweep (candidate vs flat)");
    Serial.println("  ?            this help");
    Serial.println();
}

// Advance the A/B state machine and set the codec for the next segment.
// Returns true if the next buffer should be silence (a gap marker).
static bool ab_next_segment()
{
    if (!g_ab) return false;
    if (g_silenceLeft > 0) {
        g_silenceLeft--;
        return true;
    }
    if (g_sample >= SWEEP_SAMPLES) {
        g_sample = 0;
        g_phase = 0;
        g_playingFlat = !g_playingFlat;
        // Swap filters: candidate on one pass, flat on the other.
        if (g_playingFlat) {
            apply_flat(g_es);
        } else {
            write_filter(g_es, false, g_a);
            write_filter(g_es, true, g_b);
        }
        g_silenceLeft = GAP_SAMPLES;
        return true; // the gap itself
    }
    return false;
}

static void loop_audio()
{
    if (!g_running) {
        delay(50);
        return;
    }
    size_t bytes = sizeof(g_buf);
    bool silence = ab_next_segment();

    if (!silence) {
        fill_tone(g_buf, bytes, false);
        g_sample++;
    } else {
        memset(g_buf, 0, bytes);
    }
    size_t written = 0;
    i2s_write(g_port, g_buf, bytes, &written, portMAX_DELAY);
}

static void handle_cmd(ES8388 &es, char *line)
{
    while (*line == ' ') line++;
    if (*line == 0) return;

    if (line[0] == '?' || line[0] == 'h') { help(); return; }
    if (line[0] == 'f') {
        apply_flat(es);
        g_a = g_b = 0x1FF7FDFFUL;
        Serial.println("both filters restored to flat default");
        return;
    }
    if (line[0] == 'A') {
        g_ab = !g_ab;
        g_sample = 0;
        Serial.printf("A/B sweep %s\n", g_ab ? "ON (candidate then flat)" : "OFF");
        return;
    }
    if (line[0] == 'd') { dump(es); return; }

    unsigned long w = 0;
    if (line[0] == 'f' && (line[1] == 'a' || line[1] == 'b')) {
        bool which = (line[1] == 'b');
        if (sscanf(line + 2, "%lx", &w) == 1) {
            w &= 0x3FFFFFFFUL;
            if (which) { g_b = w; write_filter(es, true, w); }
            else       { g_a = w; write_filter(es, false, w); }
            Serial.printf("filter %c = 0x%06lX\n", which ? 'B' : 'A', w);
        } else Serial.println("usage: fa/fb <8 hex digits>");
        return;
    }
    int idx, v;
    if ((line[0] == 's' || line[0] == 'g') && sscanf(line + 1, "%d %d", &idx, &v) == 2) {
        bool which = (line[0] == 'g');
        if (idx < 0 || idx > 5) { Serial.println("field index 0..5"); return; }
        if (which) { g_b = set_field(g_b, idx, v); write_filter(es, true, g_b); }
        else       { g_a = set_field(g_a, idx, v); write_filter(es, false, g_a); }
        Serial.printf("filter %c field %d = %d -> 0x%06lX\n",
                      which ? 'B' : 'A', idx, v, which ? (unsigned long)g_b : (unsigned long)g_a);
        return;
    }
    Serial.println("unknown command, ? for help");
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("ES8388 EQ lab");

    // Codec first (it drives MCLK on GPIO0 via CLK_OUT1 like the firmware does).
    Serial.print("ES8388 identify... ");
    while (!g_es.begin(PIN_SDA, PIN_SCL)) { Serial.print("."); delay(500); }
    Serial.println(" OK");
    g_es.volume(ES8388::ES_MAIN, 75);
    g_es.volume_l(ES8388::ES_OUT1, 30); g_es.volume_r(ES8388::ES_OUT1, 30);
    g_es.volume_l(ES8388::ES_OUT2, 30); g_es.volume_r(ES8388::ES_OUT2, 30);
    apply_flat(g_es);

    // I2S TX, master (we drive BCLK/LRC/MCLK), 16-bit stereo.
    i2s_config_t cfg;
    cfg.mode         = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate  = SAMPLE_RATE;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_ALL_RIGHT;
    cfg.communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S_MSB;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = 8;
    cfg.dma_buf_len = 256;
    cfg.use_apll = SS_DISABLE;
    cfg.tx_desc_auto_clear = true;
    cfg.fixed_mclk = 0;

    if (i2s_driver_install(g_port, &cfg, 0, NULL) != ESP_OK) {
        Serial.println("i2s_driver_install FAILED");
        while (1) delay(1000);
    }
    i2s_pin_config_t pins;
    pins.bck_io_num   = PIN_BCLK;
    pins.ws_io_num    = PIN_LRC;
    pins.data_out_num = PIN_DOUT;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;
    pins.mck_io_num   = PIN_MCLK;
    i2s_set_pin(g_port, &pins);

    g_running = true;
    g_sample = 0;
    g_phase = 0;
    Serial.println("sweep running (candidate == flat for now). ? for help");
    help();
}

void loop()
{
    static char line[64];
    static uint8_t idx = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r' || c == '\n') {
            if (idx) { line[idx] = 0; handle_cmd(g_es, line); idx = 0; }
        } else if (idx < sizeof(line) - 1) {
            line[idx++] = c;
        }
    }
    loop_audio();
}
