#include "gb_apu.h"
#include "blip_wrap.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stddef.h>
#if defined(GB_APU_HAS_MATH_H) && GB_APU_HAS_MATH_H
    #include <math.h>
#endif

#define FIFO_CAPACITY 8U /* ensure this is unsigned! */

struct GbApuFrameSequencer
{
    uint8_t index;
    uint8_t _padding[3];
};

struct GbApuLen
{
    uint16_t counter;
};

struct GbApuEnvelope
{
    uint8_t volume;
    uint8_t timer;
    bool disable;
    uint8_t _padding[1];
};

struct GbApuSweep
{
    uint16_t freq_shadow_register;
    uint8_t timer;
    bool enabled;
    bool did_negate;
    uint8_t _padding[3];
};

struct GbApuSquare
{
    uint8_t duty_index;
};

struct GbApuWave
{
    uint8_t sample_buffer;
    uint8_t position_counter;
    bool just_accessed;
    uint8_t _padding[1];
};

struct GbApuNoise
{
    uint16_t lfsr;
};

// implimentation taken from this issue: https://github.com/mgba-emu/mgba/issues/1847
struct GbApuFifo
{
    uint32_t ring_buf[FIFO_CAPACITY];
    uint16_t r_index;
    uint16_t w_index;

    // this is a 32-bit word taken from buf[r_index] on pop()
    // the 8-bit sample output is taken from the lower 8-bits,
    // then, that sample is shifted out.
    // if the fifo and playing buffer are empty, then the current
    // sample is played over, the playing buffer isn't looped as samples
    // are shifted out.
    uint32_t playing_buffer;
    uint16_t playing_buffer_index;

    // the output sample, taken from the lower 8-bits of playing_buffer.
    int8_t current_sample;
    uint8_t _padding[1];
};

struct GbApuChannel
{
    uint32_t clock; /* clock used for blip_buf. */
    uint32_t timestamp; /* timestamp since last tick(). */
    int32_t amp[2]; /* last volume output left/right. */
    int32_t frequency_timer; /* freq that's counted down every tick. */
};

struct GbApu
{
    /* for savestates, back up everything here. */
    struct GbApuChannel channels[6];
    struct GbApuLen len[4]; /* every psg channel has one. */
    struct GbApuEnvelope env[4]; /* all psg channels bar wave. */
    struct GbApuSweep sweep;
    struct GbApuSquare square[2];
    struct GbApuWave wave;
    struct GbApuNoise noise;
    struct GbApuFifo fifo[2];
    struct GbApuFrameSequencer frame_sequencer;
    uint16_t agb_soundcnt;
    uint16_t agb_soundbias;
    uint8_t io[0x50]; /* 0x30 + AGB wave ram (0x20). */
    /* end. */

    blip_wrap_t* blip;
    float channel_volume[6];
#if defined(GB_APU_HAS_MATH_H) && GB_APU_HAS_MATH_H
    int capacitor_charge_factor; /*  */
    int capacitor[2]; /* left and right capacitors */
#endif
    enum GbApuType type;
};

// APU (square1)
#define REG_NR10 apu->io[0x10]
#define REG_NR11 apu->io[0x11]
#define REG_NR12 apu->io[0x12]
#define REG_NR13 apu->io[0x13]
#define REG_NR14 apu->io[0x14]
// APU (square2)
#define REG_NR21 apu->io[0x16]
#define REG_NR22 apu->io[0x17]
#define REG_NR23 apu->io[0x18]
#define REG_NR24 apu->io[0x19]
// APU (wave)
#define REG_NR30 apu->io[0x1A]
#define REG_NR31 apu->io[0x1B]
#define REG_NR32 apu->io[0x1C]
#define REG_NR33 apu->io[0x1D]
#define REG_NR34 apu->io[0x1E]
#define REG_WAVE_TABLE (apu->io + 0x30)
// APU (noise)
#define REG_NR41 apu->io[0x20]
#define REG_NR42 apu->io[0x21]
#define REG_NR43 apu->io[0x22]
#define REG_NR44 apu->io[0x23]
// APU (control)
#define REG_NR50 apu->io[0x24]
#define REG_NR51 apu->io[0x25]
#define REG_NR52 apu->io[0x26]
// APU (agb)
#define REG_SOUNDCNT_H apu->agb_soundcnt
#define REG_SOUNDBIAS apu->agb_soundbias

#define apu_min(x, y) (x) < (y) ? (x) : (y)
#define apu_max(x, y) (x) > (y) ? (x) : (y)
#define apu_clamp(a, x, y) apu_max(apu_min(a, y), x)
#define apu_array_size(a) (sizeof(a) / sizeof(a[0]))

enum { CAPACITOR_SCALE = 15 };

static const double CHARGE_FACTOR[3] = {
    [GbApuFilter_NONE] = 1.0,
    [GbApuFilter_DMG] = 0.999958,
    [GbApuFilter_CGB] = 0.998943,
};

static const uint8_t SQUARE_DUTY_CYCLES[3][4][8] = {
    [GbApuType_DMG] = {
        { 0, 0, 0, 0, 0, 0, 0, 1 }, // 12.5%
        { 1, 0, 0, 0, 0, 0, 0, 1 }, // 25%
        { 1, 0, 0, 0, 0, 1, 1, 1 }, // 50%
        { 0, 1, 1, 1, 1, 1, 1, 0 }, // 75%
    },
    [GbApuType_CGB] = {
        { 0, 0, 0, 0, 0, 0, 0, 1 }, // 12.5%
        { 1, 0, 0, 0, 0, 0, 0, 1 }, // 25%
        { 1, 0, 0, 0, 0, 1, 1, 1 }, // 50%
        { 0, 1, 1, 1, 1, 1, 1, 0 }, // 75%
    },
    [GbApuType_AGB] = {
        { 1, 1, 1, 1, 1, 1, 1, 0, }, // 87.5%
        { 0, 1, 1, 1, 1, 1, 1, 0, }, // 75%
        { 0, 1, 1, 1, 1, 0, 0, 0, }, // 50%
        { 1, 0, 0, 0, 0, 0, 0, 1, }, // 25%
    },
};

// multiply then shift down, eg 75% vol is ((v * 3) / 4)
static const uint8_t WAVE_VOLUME_MULTIPLYER[8] = {
    0, // 0%
    4, // 100%
    2, // 50%
    1, // 25%
    3, // 75%
};

static const uint8_t NOISE_DIVISOR[8] = {
    8, 16, 32, 48, 64, 80, 96, 112
};

static const uint8_t AGB_PSG_SHIFT_TABLE[4] = {
    2, // 25%
    1, // 50%
    0, // 100%
};

enum ChannelType {
    ChannelType_SQUARE0,
    ChannelType_SQUARE1,
    ChannelType_WAVE,
    ChannelType_NOISE,
    ChannelType_FIFOA,
    ChannelType_FIFOB,
};

static const uint8_t SQAURE_DUTY_ADDR[2] = {
    [ChannelType_SQUARE0] = 0x11,
    [ChannelType_SQUARE1] = 0x16,
};

static const uint16_t LEN_RELOAD_VALUE[4] = {
    [ChannelType_SQUARE0] = 64,
    [ChannelType_SQUARE1] = 64,
    [ChannelType_WAVE] = 256,
    [ChannelType_NOISE] = 64,
};

// indexs using the channel number
static const uint8_t LEN_REG_ADDR[4] = {
    [ChannelType_SQUARE0] = 0x14,
    [ChannelType_SQUARE1] = 0x19,
    [ChannelType_WAVE] = 0x1E,
    [ChannelType_NOISE] = 0x23,
};

static const uint8_t ENV_REG_ADDR[4] = {
    [ChannelType_SQUARE0] = 0x12,
    [ChannelType_SQUARE1] = 0x17,
    [ChannelType_NOISE] = 0x21,
};

enum {
    NRx0 = 1 << 2,
    NRx1 = 1 << 3,
    NRx2 = 1 << 4,
    NRx3 = 1 << 5,
    NRx4 = 1 << 6,
};

// converts address to channel number + NRXX
static const uint8_t IO_CHANNEL_NUM[0x40] = {
    [0x10] = NRx0 | ChannelType_SQUARE0,
    [0x11] = NRx1 | ChannelType_SQUARE0,
    [0x12] = NRx2 | ChannelType_SQUARE0,
    [0x13] = NRx3 | ChannelType_SQUARE0,
    [0x14] = NRx4 | ChannelType_SQUARE0,

    [0x16] = NRx1 | ChannelType_SQUARE1,
    [0x17] = NRx2 | ChannelType_SQUARE1,
    [0x18] = NRx3 | ChannelType_SQUARE1,
    [0x19] = NRx4 | ChannelType_SQUARE1,

    [0x1A] = NRx0 | ChannelType_WAVE,
    [0x1B] = NRx1 | ChannelType_WAVE,
    [0x1C] = NRx2 | ChannelType_WAVE,
    [0x1D] = NRx3 | ChannelType_WAVE,
    [0x1E] = NRx4 | ChannelType_WAVE,

    [0x20] = NRx1 | ChannelType_NOISE,
    [0x21] = NRx2 | ChannelType_NOISE,
    [0x22] = NRx3 | ChannelType_NOISE,
    [0x23] = NRx4 | ChannelType_NOISE,
};

// this is used to reduce the size of the table below.
enum { AGB_ADDR_OFFSET = 0x60 };
// this is used for 16 writes.
enum { AGB_UNUSED_ADDR = 0x27 };

// translates agb addr to dmg addr.
static const uint8_t AGB_ADDR_TRANSLATION[64] = {
    [0x60 - AGB_ADDR_OFFSET] = 0x10, // IO_SOUND1CNT_L
    [0x61 - AGB_ADDR_OFFSET] = AGB_UNUSED_ADDR,
    [0x62 - AGB_ADDR_OFFSET] = 0x11, // IO_SOUND1CNT_H
    [0x63 - AGB_ADDR_OFFSET] = 0x12, // IO_SOUND1CNT_H
    [0x64 - AGB_ADDR_OFFSET] = 0x13, // IO_SOUND1CNT_X
    [0x65 - AGB_ADDR_OFFSET] = 0x14, // IO_SOUND1CNT_X

    [0x68 - AGB_ADDR_OFFSET] = 0x16, // IO_SOUND2CNT_L
    [0x69 - AGB_ADDR_OFFSET] = 0x17, // IO_SOUND2CNT_L
    [0x6C - AGB_ADDR_OFFSET] = 0x18, // IO_SOUND2CNT_H
    [0x6D - AGB_ADDR_OFFSET] = 0x19, // IO_SOUND2CNT_H

    [0x70 - AGB_ADDR_OFFSET] = 0x1A, // IO_SOUND3CNT_L
    [0x71 - AGB_ADDR_OFFSET] = AGB_UNUSED_ADDR,
    [0x72 - AGB_ADDR_OFFSET] = 0x1B, // IO_SOUND3CNT_H
    [0x73 - AGB_ADDR_OFFSET] = 0x1C, // IO_SOUND3CNT_H
    [0x74 - AGB_ADDR_OFFSET] = 0x1D, // IO_SOUND3CNT_X
    [0x75 - AGB_ADDR_OFFSET] = 0x1E, // IO_SOUND3CNT_X

    [0x78 - AGB_ADDR_OFFSET] = 0x20, // IO_SOUND4CNT_L
    [0x79 - AGB_ADDR_OFFSET] = 0x21, // IO_SOUND4CNT_L
    [0x7C - AGB_ADDR_OFFSET] = 0x22, // IO_SOUND4CNT_H
    [0x7D - AGB_ADDR_OFFSET] = 0x23, // IO_SOUND4CNT_H

    [0x80 - AGB_ADDR_OFFSET] = 0x24, // IO_SOUNDCNT_L
    [0x81 - AGB_ADDR_OFFSET] = 0x25, // IO_SOUNDCNT_L
    [0x84 - AGB_ADDR_OFFSET] = 0x26, // IO_SOUNDCNT_X
    [0x85 - AGB_ADDR_OFFSET] = AGB_UNUSED_ADDR,

    [0x90 - AGB_ADDR_OFFSET] = 0x30, // IO_WAVE_RAM0_L
    [0x91 - AGB_ADDR_OFFSET] = 0x31, // IO_WAVE_RAM0_L
    [0x92 - AGB_ADDR_OFFSET] = 0x32, // IO_WAVE_RAM0_H
    [0x93 - AGB_ADDR_OFFSET] = 0x33, // IO_WAVE_RAM0_H
    [0x94 - AGB_ADDR_OFFSET] = 0x34, // IO_WAVE_RAM1_L
    [0x95 - AGB_ADDR_OFFSET] = 0x35, // IO_WAVE_RAM1_L
    [0x96 - AGB_ADDR_OFFSET] = 0x36, // IO_WAVE_RAM1_H
    [0x97 - AGB_ADDR_OFFSET] = 0x37, // IO_WAVE_RAM1_H
    [0x98 - AGB_ADDR_OFFSET] = 0x38, // IO_WAVE_RAM2_L
    [0x99 - AGB_ADDR_OFFSET] = 0x39, // IO_WAVE_RAM2_L
    [0x9A - AGB_ADDR_OFFSET] = 0x3A, // IO_WAVE_RAM2_H
    [0x9B - AGB_ADDR_OFFSET] = 0x3B, // IO_WAVE_RAM2_H
    [0x9C - AGB_ADDR_OFFSET] = 0x3C, // IO_WAVE_RAM3_L
    [0x9D - AGB_ADDR_OFFSET] = 0x3D, // IO_WAVE_RAM3_L
    [0x9E - AGB_ADDR_OFFSET] = 0x3E, // IO_WAVE_RAM3_H
    [0x9F - AGB_ADDR_OFFSET] = 0x3F, // IO_WAVE_RAM3_H
};

static const uint8_t IO_READ_VALUE_DMG_CGB[0x40] = {
    [0x10] = 0x80, // NR10
    [0x11] = 0x3F, // NR11
    [0x12] = 0x00, // NR12
    [0x13] = 0xFF, // NR13
    [0x14] = 0xBF, // NR14
    [0x15] = 0xFF,
    [0x16] = 0x3F, // NR21
    [0x17] = 0x00, // NR22
    [0x18] = 0xFF, // NR23
    [0x19] = 0xBF, // NR24
    [0x1A] = 0x7F, // NR30
    [0x1B] = 0xFF, // NR31
    [0x1C] = 0x9F, // NR32
    [0x1D] = 0xFF, // NR33
    [0x1E] = 0xBF, // NR34
    [0x1F] = 0xFF,
    [0x20] = 0xFF, // NR41
    [0x21] = 0x00, // NR42
    [0x22] = 0x00, // NR43
    [0x23] = 0xBF, // NR44
    [0x24] = 0x00, // NR50
    [0x25] = 0x00, // NR51
    [0x26] = 0x70, // NR52
    [0x27] = 0xFF,
    [0x28] = 0xFF,
    [0x29] = 0xFF,
    [0x2A] = 0xFF,
    [0x2B] = 0xFF,
    [0x2C] = 0xFF,
    [0x2D] = 0xFF,
    [0x2E] = 0xFF,
    [0x2F] = 0xFF,
};

static const uint8_t IO_READ_VALUE_AGB[0x40] = {
    [0x10] = 0x80, // NR10
    [0x11] = 0x3F, // NR11
    [0x12] = 0x00, // NR12
    [0x13] = 0xFF, // NR13
    [0x14] = 0xBF, // NR14
    [0x16] = 0x3F, // NR21
    [0x17] = 0x00, // NR22
    [0x18] = 0xFF, // NR23
    [0x19] = 0xBF, // NR24
    [0x1A] = 0x1F, // NR30
    [0x1B] = 0xFF, // NR31
    [0x1C] = 0x1F, // NR32
    [0x1D] = 0xFF, // NR33
    [0x1E] = 0xBF, // NR34
    [0x20] = 0xFF, // NR41
    [0x21] = 0x00, // NR42
    [0x22] = 0x00, // NR43
    [0x23] = 0xBF, // NR44
    [0x24] = 0x88, // NR50
    [0x25] = 0x00, // NR51
    [0x26] = 0x70, // NR52
    [AGB_UNUSED_ADDR] = 0xFF, // see: AGB_UNUSED_ADDR
};

// reads return the register or'd with this table (agb is masked).
static const uint8_t* IO_READ_VALUE[3] = {
    [GbApuType_DMG] = IO_READ_VALUE_DMG_CGB,
    [GbApuType_CGB] = IO_READ_VALUE_DMG_CGB,
    [GbApuType_AGB] = IO_READ_VALUE_AGB,
};

// initial values of wave ram when powered on.
static const uint8_t WAVE_RAM_INITIAL[3][0x10] = {
    [GbApuType_DMG] = {
        0x84, 0x40, 0x43, 0xAA, 0x2D, 0x78, 0x92, 0x3C,
        0x60, 0x59, 0x59, 0xB0, 0x34, 0xB8, 0x2E, 0xDA,
    },
    [GbApuType_CGB] = {
        0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
        0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    },
    [GbApuType_AGB] = {
        0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
        0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    },
};

static inline bool apu_is_dmg(const GbApu* apu)
{
    return apu->type == GbApuType_DMG;
}

static inline bool apu_is_cgb(const GbApu* apu)
{
    return apu->type == GbApuType_CGB || apu->type == GbApuType_AGB;
}

static inline bool apu_is_agb(const GbApu* apu)
{
    return apu->type == GbApuType_AGB;
}

static inline bool apu_is_enabled(const GbApu* apu)
{
    return REG_NR52 & 0x80;
}

static inline void channel_enable(GbApu* apu, unsigned num)
{
    REG_NR52 |= 1 << num;
}

static inline void channel_disable(GbApu* apu, unsigned num)
{
    REG_NR52 &= ~(1 << num);
    apu->channels[num].frequency_timer = 0;
}

static inline bool channel_is_enabled(const GbApu* apu, unsigned num)
{
    return REG_NR52 & (1 << num);
}

static inline bool channel_is_dac_enabled(const GbApu* apu, unsigned num)
{
    // wave channel has it's own dac
    if (num == ChannelType_WAVE)
    {
        return REG_NR30 & 0x80;
    }
    // other channels use envelope
    else
    {
        return apu->io[ENV_REG_ADDR[num]] & 0xF8; // starting_volume || mode
    }
}

static unsigned channel_get_frequency(const GbApu* apu, unsigned num)
{
    const unsigned m = apu_is_agb(apu) ? 4 : 1;

    if (num == ChannelType_SQUARE0)
    {
        const unsigned freq = ((REG_NR14 & 7) << 8) | REG_NR13;
        return (2048 - freq) * 4 * m;
    }
    else if (num == ChannelType_SQUARE1)
    {
        const unsigned freq = ((REG_NR24 & 7) << 8) | REG_NR23;
        return (2048 - freq) * 4 * m;
    }
    else if (num == ChannelType_WAVE)
    {
        const unsigned freq = ((REG_NR34 & 7) << 8) | REG_NR33;
        return (2048 - freq) * 2 * m;
    }
    else // if (num == ChannelType_NOISE)
    {
        const unsigned divisor_code = REG_NR43 & 0x7;
        const unsigned clock_shift = REG_NR43 >> 4;
        return (NOISE_DIVISOR[divisor_code] << clock_shift) * m;
    }
}

static inline void add_delta(GbApu* apu, struct GbApuChannel* c, unsigned clock_time, int sample, unsigned lr)
{
    const int delta = sample - c->amp[lr];
    if (delta) // same as (sample != amp)
    {
        blip_wrap_add_delta(apu->blip, clock_time, delta, lr);
        c->amp[lr] += delta; // same as (amp = sample)
    }
}

static inline void add_delta_fast(GbApu* apu, struct GbApuChannel* c, unsigned clock_time, int sample, unsigned lr)
{
    const int delta = sample - c->amp[lr];
    if (delta) // same as (sample != amp)
    {
        blip_wrap_add_delta_fast(apu->blip, clock_time, delta, lr);
        c->amp[lr] += delta; // same as (amp = sample)
    }
}

static void channel_sync_psg(GbApu* apu, unsigned num, unsigned time)
{
    struct GbApuChannel* c = &apu->channels[num];

    // get starting point
    const unsigned base_clock = c->clock;
    // i am not 100% sure how this works, but trust me, it works
    unsigned from = base_clock + c->frequency_timer;
    // get new timestamp
    const unsigned new_timestamp = time;
    // calculate how many cycles have elapsed since last sync
    const int until = new_timestamp - c->timestamp;
    // advance forward
    c->clock += until;
    // save new timestamp
    c->timestamp = new_timestamp;

    // already clocked on this cycle, or bad timestamp
    if (until <= 0)
    {
        return;
    }

    // clip clock range
    if (c->frequency_timer > until)
    {
        from = base_clock + until;
    }

    // needed to pass blarggs dmg 09-wave and 12-wave
    if (num == ChannelType_WAVE)
    {
        apu->wave.just_accessed = false;
    }

    if (!apu_is_enabled(apu) || !channel_is_enabled(apu, num))
    {
        add_delta(apu, c, from, 0, 0);
        add_delta(apu, c, from, 0, 1);
        return;
    }

    const bool is_agb = apu_is_agb(apu);
    const bool left_enabled = REG_NR51 & (1 << (num + 0));
    const bool right_enabled = REG_NR51 & (1 << (num + 4));
    const int left_volume = left_enabled * (1 + ((REG_NR50 >> 0) & 0x7));
    const int right_volume = right_enabled * (1 + ((REG_NR50 >> 4) & 0x7));
    const unsigned psg_shift = is_agb ? AGB_PSG_SHIFT_TABLE[REG_SOUNDCNT_H & 0x3] : 0;

    const unsigned freq = channel_get_frequency(apu, num);
    const float volume = apu->channel_volume[num];

    c->frequency_timer -= until;

    // the below can be further optimised, should you need to.
    if (num == ChannelType_SQUARE0 || num == ChannelType_SQUARE1)
    {
        struct GbApuSquare* square = &apu->square[num];
        const int envelope = apu->env[num].volume;

        const unsigned duty = apu->io[SQAURE_DUTY_ADDR[num]] >> 6;
        unsigned duty_bit = SQUARE_DUTY_CYCLES[apu->type][duty][square->duty_index];
        const int sign_flipflop = duty_bit ? +1 : -1;

        int left = blip_apply_volume_to_sample(apu->blip, envelope * left_volume * sign_flipflop >> psg_shift, volume);
        int right = blip_apply_volume_to_sample(apu->blip, envelope * right_volume * sign_flipflop >> psg_shift, volume);
        add_delta(apu, c, from, left, 0);
        add_delta(apu, c, from, right, 1);

        while (c->frequency_timer <= 0)
        {
            square->duty_index = (square->duty_index + 1) % 8;
            const unsigned new_duty_bit = SQUARE_DUTY_CYCLES[apu->type][duty][square->duty_index];
            if (new_duty_bit != duty_bit)
            {
                duty_bit = new_duty_bit;
                left = -left;
                right = -right;
                add_delta(apu, c, from, left, 0);
                add_delta(apu, c, from, right, 1);
            }

            from += freq;
            c->frequency_timer += freq;
        }
    }
    else if (num == ChannelType_WAVE)
    {
        struct GbApuWave* wave = &apu->wave;

        const int invert = is_agb ? 0xF : 0x0;
        const bool bank_mode = is_agb ? (REG_NR30 & 0x20) : 0;
        const bool bank_select = is_agb ? (REG_NR30 & 0x40) : 1;
        const unsigned bank_mask = bank_mode ? 64 : 32;
        const unsigned bank_offset = (bank_mode == 0 && bank_select) ? 0 : 16;

        const int wave_mult = WAVE_VOLUME_MULTIPLYER[(REG_NR32 >> 5) & (is_agb ? 0x7 : 0x3)];
        int sample = (wave->position_counter & 0x1) ? wave->sample_buffer & 0xF : wave->sample_buffer >> 4;
        sample = (((sample ^ invert) * 2 - 15) * wave_mult) >> 2; // [-15,+15] inverted

        int left = blip_apply_volume_to_sample(apu->blip, sample * left_volume, volume);
        int right = blip_apply_volume_to_sample(apu->blip, sample * left_volume, volume);
        add_delta_fast(apu, c, from, left, 0);
        add_delta_fast(apu, c, from, right, 1);

        const bool will_tick = c->frequency_timer <= 0;
        while (c->frequency_timer <= 0)
        {
            wave->position_counter = (wave->position_counter + 1) % bank_mask;

            // fetch new sample if we are done with this buffer
            if (!(wave->position_counter & 0x1))
            {
                wave->sample_buffer = REG_WAVE_TABLE[bank_offset + (wave->position_counter >> 1)];
            }

            sample = (wave->position_counter & 0x1) ? wave->sample_buffer & 0xF : wave->sample_buffer >> 4;
            sample = (((sample ^ invert) * 2 - 15) * wave_mult) >> 2; // [-15,+15] inverted

            int left = blip_apply_volume_to_sample(apu->blip, sample * left_volume, volume);
            int right = blip_apply_volume_to_sample(apu->blip, sample * left_volume, volume);
            add_delta_fast(apu, c, from, left, 0);
            add_delta_fast(apu, c, from, right, 1);

            from += freq;
            c->frequency_timer += freq;
        }

        // if ticked, and timer==freq, that means it was accessed on this very cycle
        wave->just_accessed = will_tick && (unsigned)c->frequency_timer == freq;
    }
    else if (num == ChannelType_NOISE)
    {
        struct GbApuNoise* noise = &apu->noise;
        const int envelope = apu->env[num].volume;

        unsigned bit0 = noise->lfsr & 0x1;
        const int sign_flipflop = bit0 ? -1 : +1; // inverted

        int left = blip_apply_volume_to_sample(apu->blip, envelope * left_volume * sign_flipflop >> psg_shift, volume);
        int right = blip_apply_volume_to_sample(apu->blip, envelope * right_volume * sign_flipflop >> psg_shift, volume);
        add_delta_fast(apu, c, from, left, 0);
        add_delta_fast(apu, c, from, right, 1);

        // clock shift of 14/15 means noise recieves no clocks!
        const unsigned clock_shift = REG_NR43 >> 4;
        if (noise->lfsr && clock_shift < 14)
        {
            const unsigned bits = (REG_NR43 & 0x8) ? 0x4040 : 0x4000;

            while (c->frequency_timer <= 0)
            {
                const unsigned result = ((noise->lfsr >> 1) ^ noise->lfsr) & 0x1;
                // now we shift the lfsr BEFORE setting the value!
                noise->lfsr >>= 1;
                // unset bit-14 or bit-6 AND bit-14
                noise->lfsr &= ~bits;
                // set bit-14 or bit-6 AND bit-14
                noise->lfsr |= bits * result;

                // get new bit0, see if it changed
                const unsigned new_bit0 = noise->lfsr & 0x1;
                if (new_bit0 != bit0)
                {
                    bit0 = new_bit0;
                    left = -left;
                    right = -right;
                    add_delta_fast(apu, c, from, left, 0);
                    add_delta_fast(apu, c, from, right, 1);
                }

                from += freq;
                c->frequency_timer += freq;
            }
        }
    }
}

static void channel_sync_fifo(GbApu* apu, unsigned num, unsigned time)
{
    struct GbApuChannel* c = &apu->channels[num];

    // get starting point
    const unsigned base_clock = c->clock;
    // i am not 100% sure how this works, but trust me, it works
    unsigned from = base_clock + c->frequency_timer;
    // get new timestamp
    const unsigned new_timestamp = time;
    // calculate how many cycles have elapsed since last sync
    const int until = new_timestamp - c->timestamp;
    // advance forward
    c->clock += until;
    // save new timestamp
    c->timestamp = new_timestamp;

    // always advance the clock above
    if (!apu_is_agb(apu))
    {
        return;
    }

    if (!apu_is_enabled(apu))
    {
        add_delta(apu, c, from, 0, 0);
        add_delta(apu, c, from, 0, 1);
        return;
    }

    const unsigned reg = REG_SOUNDCNT_H;
    const bool volume_code = num == ChannelType_FIFOA ? (reg & 0x4) : (reg & 0x8);
    const bool enable_right = num == ChannelType_FIFOA ? (reg & 0x100) : (reg & 0x1000);
    const bool enable_left = num == ChannelType_FIFOA ? (reg & 0x200) : (reg & 0x2000);

    const struct GbApuFifo* fifo = &apu->fifo[num - ChannelType_FIFOA];

    const int sample = fifo->current_sample * (volume_code ? 4 : 2);
    const int left = blip_apply_volume_to_sample(apu->blip, sample * enable_left, apu->channel_volume[num]);
    const int right = blip_apply_volume_to_sample(apu->blip, sample * enable_right, apu->channel_volume[num]);

    add_delta(apu, c, from, left, 0);
    add_delta(apu, c, from, right, 1);
}

static void channel_sync_psg_all(GbApu* apu, unsigned time)
{
    channel_sync_psg(apu, ChannelType_SQUARE0, time);
    channel_sync_psg(apu, ChannelType_SQUARE1, time);
    channel_sync_psg(apu, ChannelType_WAVE, time);
    channel_sync_psg(apu, ChannelType_NOISE, time);
}

static void channel_sync_fifo_all(GbApu* apu, unsigned time)
{
    channel_sync_fifo(apu, ChannelType_FIFOA, time);
    channel_sync_fifo(apu, ChannelType_FIFOB, time);
}

// this is used when a channel is triggered
static bool is_next_frame_sequencer_step_not_len(const GbApu* apu)
{
    // check if the current counter is the len clock, the next one won't be!
    return apu->frame_sequencer.index & 0x1;
}

// this is used when channels 1,2,4 are triggered
static bool is_next_frame_sequencer_step_vol(const GbApu* apu)
{
    // check if the current counter is the len clock, the next one won't be!
    return apu->frame_sequencer.index == 7;
}

static unsigned sweep_get_new_freq(GbApu* apu)
{
    const unsigned shift = REG_NR10 & 0x7;
    const unsigned negate = (REG_NR10 >> 3) & 0x1;
    const unsigned new_freq = apu->sweep.freq_shadow_register >> shift;

    if (negate)
    {
        apu->sweep.did_negate = true;
        return apu->sweep.freq_shadow_register - new_freq;
    }
    else
    {
        return apu->sweep.freq_shadow_register + new_freq;
    }
}

static void sweep_do_freq_calc(GbApu* apu, bool update_value)
{
    const unsigned new_freq = sweep_get_new_freq(apu);
    const unsigned shift = REG_NR10 & 0x7;

    if (new_freq > 2047)
    {
        channel_disable(apu, ChannelType_SQUARE0);
    }
    else if (shift && update_value)
    {
        apu->sweep.freq_shadow_register = new_freq;
        REG_NR13 = new_freq & 0xFF;
        REG_NR14 = (REG_NR14 & ~0x7) | new_freq >> 8;
    }
}

static void sweep_clock(GbApu* apu, unsigned num, unsigned time)
{
    struct GbApuSweep* sweep = &apu->sweep;

    if (channel_is_enabled(apu, num) && sweep->enabled)
    {
        assert(sweep->timer >= 0 && sweep->timer <= 8);
        sweep->timer = (sweep->timer - 1) & 0x7;

        if (sweep->timer == 0)
        {
            const unsigned period = (REG_NR10 >> 4) & 0x7;
            sweep->timer = period;

            // sweep is only clocked if period is not 0
            if (period != 0)
            {
                channel_sync_psg(apu, num, time);
                // first time updates the value
                sweep_do_freq_calc(apu, true);
                // second time does not, but still checks for overflow
                sweep_do_freq_calc(apu, false);
            }
        }
    }
}

static void sweep_trigger(GbApu* apu)
{
    apu->sweep.did_negate = false;

    // reload sweep timer with period
    const unsigned period = (REG_NR10 >> 4) & 0x7;
    apu->sweep.timer = period;

    // the freq is loaded into the shadow_freq_reg
    apu->sweep.freq_shadow_register = ((REG_NR14 & 7) << 8) | REG_NR13;

    // sweep is enabled flag if period or shift is non zero
    const unsigned shift = REG_NR10 & 0x7;
    apu->sweep.enabled = (period != 0) || (shift != 0);

    // sweep calc is performed, but the value isn't updated
    if (shift)
    {
        sweep_do_freq_calc(apu, false);
    }
}

static bool len_is_enabled(const GbApu* apu, unsigned num)
{
    return apu->io[LEN_REG_ADDR[num]] & 0x40;
}

static void len_clock(GbApu* apu, unsigned num, unsigned time)
{
    struct GbApuLen* len = &apu->len[num];

    // len is still clocked, even with the channel disabled.
    if (len_is_enabled(apu, num) && len->counter > 0)
    {
        len->counter--;
        if (len->counter == 0)
        {
            channel_sync_psg(apu, num, time);
            channel_disable(apu, num);
        }
    }
}

static void len_on_nrx4_edge_case_write(GbApu* apu, unsigned num, unsigned new_value, unsigned old_value)
{
    struct GbApuLen* len = &apu->len[num];

    const unsigned old_enabled = old_value & 0x40;
    const unsigned new_enabled = new_value & 0x40;

    // if next is not len and len is NOW enabled, it is clocked
    if (is_next_frame_sequencer_step_not_len(apu) && len->counter && !old_enabled && new_enabled)
    {
        len->counter--;

        // if this makes the result 0, and trigger is clear, disable channel
        if (!len->counter && !(new_value & 0x80))
        {
            channel_disable(apu, num);
        }
    }
}

static void len_trigger(GbApu* apu, unsigned num)
{
    struct GbApuLen* len = &apu->len[num];

    if (len->counter == 0)
    {
        len->counter = LEN_RELOAD_VALUE[num];
        if (len_is_enabled(apu, num) && is_next_frame_sequencer_step_not_len(apu))
        {
            len->counter--; // will this disable the channel?
        }
    }
}

static void env_clock(GbApu* apu, unsigned num, unsigned time)
{
    struct GbApuEnvelope* env = &apu->env[num];

    if (channel_is_enabled(apu, num) && !env->disable)
    {
        env->timer = (env->timer - 1) & 0x7;
        if (env->timer == 0)
        {
            const unsigned reg = apu->io[ENV_REG_ADDR[num]];
            const unsigned period = reg & 0x7;
            env->timer = period;

            if (period != 0)
            {
                const unsigned mode = (reg >> 3) & 0x1;
                const unsigned modifier = mode == 1 ? +1 : -1;
                const unsigned new_volume = env->volume + modifier;

                if (new_volume <= 15)
                {
                    channel_sync_psg(apu, num, time);
                    env->volume = new_volume;
                }
                else
                {
                    env->disable = true;
                }
            }
        }
    }
}

static void env_trigger(GbApu* apu, unsigned num)
{
    struct GbApuEnvelope* env = &apu->env[num];

    const unsigned reg = apu->io[ENV_REG_ADDR[num]];
    const unsigned period = reg & 0x7;
    const unsigned starting_vol = reg >> 4;;

    env->disable = false;
    env->timer = period;
    if (is_next_frame_sequencer_step_vol(apu))
    {
        // todo: does this wrap around as the timer is 3-bits?
        env->timer++;
    }

    // reload the volume
    env->volume = starting_vol;
}

static void env_write(GbApu* apu, unsigned num, unsigned new_value, unsigned old_value)
{
#if defined(GB_APU_ZOMBIE) && GB_APU_ZOMBIE
    // zombie mode works differently on agb, disable it for now.
    if (!apu_is_agb(apu) && channel_is_enabled(apu, num))
    {
        // NOTE: the below "zombie mode" isn't accurate for each revision.
        // This causes ticks in zelda as it triggers zombie 2 repeatedly.
        // However, this fixes the fan favourite, Prehistorik Man.
        struct GbApuEnvelope* env = &apu->env[num];
        const unsigned old_period = old_value & 0x7;
        const unsigned old_mode = old_value & 0x8;
        const unsigned new_mode = new_value & 0x8;

        if (!old_period && !env->disable) // zombie 1
        {
            env->volume += 1;
        }
        else if (!old_mode) // zombie 2
        {
            env->volume += 2;
        }

        if (old_mode != new_mode) // zombie 3
        {
            env->volume = 16 - env->volume;
        }

        env->volume &= 0xF;
    }
#endif

    if (!channel_is_dac_enabled(apu, num))
    {
        channel_disable(apu, num);
    }
}

static void trigger(GbApu* apu, unsigned num, unsigned time)
{
    struct GbApuChannel* c = &apu->channels[num];
    const unsigned new_freq = channel_get_frequency(apu, num);
    const bool was_enabled = channel_is_enabled(apu, num);

    channel_enable(apu, num);
    len_trigger(apu, num);

    if (num == ChannelType_WAVE)
    {
        // wave ram is partially corrupted on dmg if triggered while enabled
        // the apu is ticked at 2mhz, but i tick it at 4mhz.
        // to get around this, check if the next access is within 2 cycles.
        if (apu_is_dmg(apu) && was_enabled && c->frequency_timer <= 2)
        {
            unsigned index = ((apu->wave.position_counter + 1) % 32) >> 1;
            if (index < 4)
            {
                REG_WAVE_TABLE[0] = REG_WAVE_TABLE[index];
            }
            else
            {
                index &= ~0x3;
                memcpy(REG_WAVE_TABLE, REG_WAVE_TABLE + index, 4);
            }
        }

        // https://forums.nesdev.org/viewtopic.php?t=13730
        c->frequency_timer = new_freq + 6 * (apu_is_agb(apu) ? 4 : 1);
        apu->wave.position_counter = 0;
    }
    else
    {
        env_trigger(apu, num);

        if (num == ChannelType_NOISE)
        {
            apu->noise.lfsr = 0x7FFF;
            c->frequency_timer = new_freq;
        }
        else // square0 | square1
        {
            // keep lower 2 bits
            c->frequency_timer = (c->frequency_timer & 0x3) | (new_freq & ~0x3);

            if (num == ChannelType_SQUARE0)
            {
                sweep_trigger(apu);
            }
        }
    }

    if (!channel_is_dac_enabled(apu, num))
    {
        channel_disable(apu, num);
    }

    if (channel_is_enabled(apu, num))
    {
        c->timestamp = time;
    }
}

static void on_nrx0_write(GbApu* apu, unsigned num, unsigned time, unsigned new_value, unsigned old_value)
{
    if (num == ChannelType_SQUARE0)
    {
        const unsigned old_sweep_negate = (old_value >> 3) & 0x1;
        const unsigned new_sweep_negate = (new_value >> 3) & 0x1;

        // if at least 1 sweep negate has happened since last trigger,
        // and negate is now cleared, then disable ch1.
        if (old_sweep_negate && !new_sweep_negate && apu->sweep.did_negate)
        {
            channel_disable(apu, num);
        }
    }
    else if (num == ChannelType_WAVE)
    {
        if (!channel_is_dac_enabled(apu, num))
        {
            channel_disable(apu, num);
        }
    }
}

static void on_nrx1_write(GbApu* apu, unsigned num, unsigned time, unsigned new_value, unsigned old_value)
{
    const unsigned reload_value = LEN_RELOAD_VALUE[num];
    const unsigned mask = reload_value - 1;
    apu->len[num].counter = reload_value - (new_value & mask);
}

static void on_nrx2_write(GbApu* apu, unsigned num, unsigned time, unsigned new_value, unsigned old_value)
{
    if (num != ChannelType_WAVE)
    {
        env_write(apu, num, new_value, old_value);
    }
}

static void on_nrx3_write(GbApu* apu, unsigned num, unsigned time, unsigned new_value, unsigned old_value)
{
    // nothing special happens here...
}

static void on_nrx4_write(GbApu* apu, unsigned num, unsigned time, unsigned new_value, unsigned old_value)
{
    len_on_nrx4_edge_case_write(apu, num, new_value, old_value);

    if (new_value & 0x80)
    {
        trigger(apu, num, time);
    }
}

static void frame_sequencer_clock_len(GbApu* apu, unsigned time)
{
    len_clock(apu, ChannelType_SQUARE0, time);
    len_clock(apu, ChannelType_SQUARE1, time);
    len_clock(apu, ChannelType_WAVE, time);
    len_clock(apu, ChannelType_NOISE, time);
}

static void frame_sequencer_clock_sweep(GbApu* apu, unsigned time)
{
    sweep_clock(apu, ChannelType_SQUARE0, time);
}

static void frame_sequencer_clock_env(GbApu* apu, unsigned time)
{
    env_clock(apu, ChannelType_SQUARE0, time);
    env_clock(apu, ChannelType_SQUARE1, time);
    env_clock(apu, ChannelType_NOISE, time);
}

static unsigned fifo_get_size(const struct GbApuFifo* fifo)
{
    return (fifo->w_index - fifo->r_index) % FIFO_CAPACITY;
}

static void fifo_reset(struct GbApuFifo* fifo)
{
    fifo->r_index = fifo->w_index = 0;
}

static struct GbApuFifo* fifo_from_addr(GbApu* apu, unsigned addr)
{
    return &apu->fifo[(addr & 0x4) == 0x4];
}

#if defined(GB_APU_HAS_MATH_H) && GB_APU_HAS_MATH_H
static inline int high_pass(int charge_factor, int in, int* capacitor)
{
    in <<= CAPACITOR_SCALE;
    const int out = (in - *capacitor) >> CAPACITOR_SCALE;
    *capacitor = (in - out * charge_factor);
    return apu_clamp(out, INT16_MIN, INT16_MAX);
}
#endif

/* ------------------PUBLIC API------------------ */
GbApu* apu_init(double clock_rate, double sample_rate)
{
    GbApu* apu = calloc(1, sizeof(*apu));
    if (!apu)
    {
        goto fail;
    }

    for (unsigned i = 0; i < apu_array_size(apu->channel_volume); i++)
    {
        apu->channel_volume[i] = 1.0;
    }

    if (!(apu->blip = blip_wrap_new(sample_rate))) {
        goto fail;
    }

    if (blip_wrap_set_rates(apu->blip, clock_rate, sample_rate)) {
        goto fail;
    }

    apu_set_master_volume(apu, 0.25);
    apu_set_highpass_filter(apu, GbApuFilter_NONE, clock_rate, sample_rate);

    return apu;

fail:
    apu_quit(apu);
    return NULL;
}

void apu_quit(GbApu* apu)
{
    if (apu)
    {
        if (apu->blip)
        {
            blip_wrap_delete(apu->blip);
            apu->blip = NULL;
        }
        free(apu);
    }
}

void apu_reset(GbApu* apu, enum GbApuType type)
{
    apu_clear_samples(apu);
    apu->type = type;
    memset(&apu->channels, 0, sizeof(apu->channels));
    memset(&apu->sweep, 0, sizeof(apu->sweep));
    memset(&apu->square, 0, sizeof(apu->square));
    memset(&apu->wave, 0, sizeof(apu->wave));
    memset(&apu->noise, 0, sizeof(apu->noise));
    memset(&apu->fifo, 0, sizeof(apu->fifo));
    memset(&apu->frame_sequencer, 0, sizeof(apu->frame_sequencer));
    apu->agb_soundcnt = 0;
    apu->agb_soundbias = 0;
    memset(apu->io + 0x10, 0, 0x17);
    memcpy(REG_WAVE_TABLE, WAVE_RAM_INITIAL[type], sizeof(WAVE_RAM_INITIAL[type]));
    apu->noise.lfsr = 0x7FFF;
}

void apu_update_timestamp(GbApu* apu, int time)
{
    for (unsigned i = 0; i < apu_array_size(apu->channels); i++)
    {
        apu->channels[i].timestamp += time;
    }
}

unsigned apu_read_io(GbApu* apu, unsigned addr, unsigned time)
{
    assert((addr & 0xFF) >= 0x10 && (addr & 0xFF) <= 0x3F);
    addr &= 0x3F;

    if (addr >= 0x30 && addr <= 0x3F)
    {
        const bool bank_mode = REG_NR30 & 0x20;
        if (apu_is_agb(apu) && !bank_mode)
        {
            // writes happen to the opposite bank.
            const bool bank_select = REG_NR30 & 0x40;
            const unsigned offset = (bank_select ^ 1) ? 0 : 16;
            return apu->io[addr + offset];
        }
        else
        {
            if (channel_is_enabled(apu, ChannelType_WAVE))
            {
                channel_sync_psg(apu, ChannelType_WAVE, time);
                if (apu_is_cgb(apu) || apu->wave.just_accessed)
                {
                    return REG_WAVE_TABLE[apu->wave.position_counter >> 1];
                }
                else
                {
                    return 0xFF; // writes to dmg are ignored if wave wasn't just accessed.
                }
            }
        }
    }

    return apu->io[addr] | IO_READ_VALUE[apu->type][addr];
}

void apu_write_io(GbApu* apu, unsigned addr, unsigned value, unsigned time)
{
    assert((addr & 0xFF) >= 0x10 && (addr & 0xFF) <= 0x3F);
    addr &= 0x3F;

    // nr52 is always writeable
    if (addr == 0x26) // nr52
    {
        // check if it's now disabled
        if (apu_is_enabled(apu) && !(value & 0x80))
        {
            channel_sync_psg_all(apu, time);

            // len counters are unaffected on the dmg
            const unsigned nr11 = REG_NR11 & 0x3F;
            const unsigned nr21 = REG_NR21 & 0x3F;
            const unsigned nr31 = REG_NR31;
            const unsigned nr41 = REG_NR41;

            // reset everything aside from wave ram
            memset(apu->io + 0x10, 0, 0x17);
            memset(&apu->sweep, 0, sizeof(apu->sweep));
            memset(&apu->square, 0, sizeof(apu->square));
            memset(&apu->wave, 0, sizeof(apu->wave));
            memset(&apu->noise, 0, sizeof(apu->noise));
            memset(&apu->frame_sequencer, 0, sizeof(apu->frame_sequencer));
            memset(&apu->env, 0, sizeof(apu->env));

            if (apu_is_dmg(apu))
            {
                REG_NR11 = nr11;
                REG_NR21 = nr21;
                REG_NR31 = nr31;
                REG_NR41 = nr41;
            }
            else
            {
                memset(&apu->len, 0, sizeof(apu->len));
            }
        }
        // check if it's now enabled
        else if (!apu_is_enabled(apu) && (value & 0x80))
        {
            REG_NR52 |= 0x80;
            apu->frame_sequencer.index = 0;
        }
    }
    // wave ram is always accessable
    else if (addr >= 0x30 && addr <= 0x3F) // wave ram
    {
        const bool bank_mode = REG_NR30 & 0x20;
        if (apu_is_agb(apu) && !bank_mode)
        {
            // writes happen to the opposite bank.
            const bool bank_select = REG_NR30 & 0x40;
            const unsigned offset = (bank_select ^ 1) ? 0 : 16;
            apu->io[addr + offset] = value;
        }
        else
        {
            // if enabled, writes are ignored on dmg, allowed on cgb.
            if (channel_is_enabled(apu, ChannelType_WAVE))
            {
                channel_sync_psg(apu, ChannelType_WAVE, time);
                if (apu_is_cgb(apu) || apu->wave.just_accessed)
                {
                    REG_WAVE_TABLE[apu->wave.position_counter >> 1] = value;
                }
            }
            else
            {
                apu->io[addr] = value;
            }
        }
    }
    // writes to the apu are ignore whilst disabled
    else if (!apu_is_enabled(apu))
    {
        // len counters are writeable even if apu is off
        if (apu_is_dmg(apu) && (addr == 0x11 || addr == 0x16 || addr == 0x1B || addr == 0x20))
        {
            const unsigned num = IO_CHANNEL_NUM[addr] & 0x3;
            const unsigned old_value = apu->io[addr];
            const unsigned reload_value = LEN_RELOAD_VALUE[num];
            const unsigned mask = reload_value - 1;
            apu->io[addr] = (apu->io[addr] & ~mask) | (value & mask);

            on_nrx1_write(apu, num, time, apu->io[addr], old_value);
        }
    }
    else if (addr == 0x24 || addr == 0x25) // nr50 | nr51
    {
        channel_sync_psg_all(apu, time);
        apu->io[addr] = value;
    }
    else
    {
        const unsigned num = IO_CHANNEL_NUM[addr] & 0x3;
        const unsigned nrxx = IO_CHANNEL_NUM[addr] & ~0x3;
        if (nrxx)
        {
            channel_sync_psg(apu, num, time);

            const unsigned old_value = apu->io[addr];
            apu->io[addr] = value;

            switch (nrxx)
            {
                case NRx0: on_nrx0_write(apu, num, time, value, old_value); break;
                case NRx1: on_nrx1_write(apu, num, time, value, old_value); break;
                case NRx2: on_nrx2_write(apu, num, time, value, old_value); break;
                case NRx3: on_nrx3_write(apu, num, time, value, old_value); break;
                case NRx4: on_nrx4_write(apu, num, time, value, old_value); break;
            }
        }
    }
}

void apu_frame_sequencer_clock(GbApu* apu, unsigned time)
{
    if (!apu_is_enabled(apu))
    {
        return;
    }

    switch (apu->frame_sequencer.index)
    {
        case 0: // len
        case 4:
            frame_sequencer_clock_len(apu, time);
            break;

        case 2: // len, sweep
        case 6:
            frame_sequencer_clock_len(apu, time);
            frame_sequencer_clock_sweep(apu, time);
            break;

        case 7: // vol
            frame_sequencer_clock_env(apu, time);
            break;
    }

    apu->frame_sequencer.index = (apu->frame_sequencer.index + 1) % 8;
}

unsigned apu_cgb_read_pcm12(GbApu* apu, unsigned time)
{
    assert(apu_is_cgb(apu) && "invalid access");
    channel_sync_psg(apu, ChannelType_SQUARE0, time);
    channel_sync_psg(apu, ChannelType_SQUARE1, time);

    const bool square0_enabled = channel_is_enabled(apu, ChannelType_SQUARE0);
    const bool square1_enabled = channel_is_enabled(apu, ChannelType_SQUARE1);

    const bool square0_duty = SQUARE_DUTY_CYCLES[apu->type][apu->io[SQAURE_DUTY_ADDR[0]] >> 6][apu->square[0].duty_index];
    const bool square1_duty = SQUARE_DUTY_CYCLES[apu->type][apu->io[SQAURE_DUTY_ADDR[1]] >> 6][apu->square[1].duty_index];
    const unsigned square0_sample = apu->env[ChannelType_SQUARE0].volume;
    const unsigned square1_sample = apu->env[ChannelType_SQUARE1].volume;

    unsigned value = square0_duty * square0_sample * square0_enabled * apu_is_enabled(apu) << 0;
    value |= square1_duty * square1_sample * square1_enabled * apu_is_enabled(apu) << 4;

    return value;
}

unsigned apu_cgb_read_pcm34(GbApu* apu, unsigned time)
{
    assert(apu_is_cgb(apu) && "invalid access");
    channel_sync_psg(apu, ChannelType_WAVE, time);
    channel_sync_psg(apu, ChannelType_NOISE, time);

    const bool wave_enabled = channel_is_enabled(apu, ChannelType_WAVE);
    const bool noise_enabled = channel_is_enabled(apu, ChannelType_NOISE);
    const unsigned wave_sample = (apu->wave.position_counter & 0x1) ? apu->wave.sample_buffer & 0xF : apu->wave.sample_buffer >> 4;
    const unsigned noise_sample = !(apu->noise.lfsr & 1) * apu->env[ChannelType_NOISE].volume;

    unsigned value = wave_sample * wave_enabled * apu_is_enabled(apu) << 0;
    value |= noise_sample * noise_enabled * apu_is_enabled(apu) << 4;

    return value;
}

unsigned apu_agb_read8_io(GbApu* apu, unsigned addr, unsigned time)
{
    assert(apu_is_agb(apu) && "invalid access");
    addr = AGB_ADDR_TRANSLATION[(addr & 0xFF) - AGB_ADDR_OFFSET];
    return apu_read_io(apu, addr, time) & ~IO_READ_VALUE_AGB[addr];
}

void apu_agb_write8_io(GbApu* apu, unsigned addr, unsigned value, unsigned time)
{
    assert(apu_is_agb(apu) && "invalid access");
    addr = AGB_ADDR_TRANSLATION[(addr & 0xFF) - AGB_ADDR_OFFSET];
    apu_write_io(apu, addr, value, time);
}

unsigned apu_agb_read16_io(GbApu* apu, unsigned addr, unsigned time)
{
    unsigned v = apu_agb_read8_io(apu, addr + 0, time) << 0;
    v |= apu_agb_read8_io(apu, addr + 1, time) << 8;
    return v;
}

void apu_agb_write16_io(GbApu* apu, unsigned addr, unsigned value, unsigned time)
{
    apu_agb_write8_io(apu, addr + 0, value >> 0, time);
    apu_agb_write8_io(apu, addr + 1, value >> 8, time);
}

unsigned apu_agb_soundcnt_read(GbApu* apu, unsigned time)
{
    assert(apu_is_agb(apu) && "invalid access");
    return REG_SOUNDCNT_H & 0x770F;
}

void apu_agb_soundcnt_write(GbApu* apu, unsigned value, unsigned time)
{
    assert(apu_is_agb(apu) && "invalid access");
    channel_sync_psg_all(apu, time);
    channel_sync_fifo_all(apu, time);

    if (value & 0x800)
    {
        fifo_reset(&apu->fifo[0]);
    }
    if (value & 0x8000)
    {
        fifo_reset(&apu->fifo[1]);
    }

    REG_SOUNDCNT_H = value;
}

unsigned apu_agb_soundbias_read(GbApu* apu, unsigned time)
{
    assert(apu_is_agb(apu) && "invalid access");
    return REG_SOUNDBIAS & 0xC3FF;
}

void apu_agb_soundbias_write(GbApu* apu, unsigned value, unsigned time)
{
    assert(apu_is_agb(apu) && "invalid access");
    REG_SOUNDBIAS = value;
}

// 8-bit writes use the previous 3 samples in the buffer
void apu_agb_fifo_write8(GbApu* apu, unsigned addr, unsigned value)
{
    const struct GbApuFifo* fifo = fifo_from_addr(apu, addr);
    const unsigned bit_shift = (addr & 0x3) * 8;
    const unsigned buf_word = fifo->ring_buf[fifo->w_index];
    const unsigned result = (buf_word & ~(0xFF << bit_shift)) | (value << bit_shift);
    apu_agb_fifo_write32(apu, addr, result);
}

// 16-bit writes use the previous 1 sample in the buffer
void apu_agb_fifo_write16(GbApu* apu, unsigned addr, unsigned value)
{
    const struct GbApuFifo* fifo = fifo_from_addr(apu, addr);
    const unsigned bit_shift = (addr & 0x2) * 8;
    const unsigned buf_word = fifo->ring_buf[fifo->w_index];
    const unsigned result = (buf_word & ~(0xFFFF << bit_shift)) | (value << bit_shift);
    apu_agb_fifo_write32(apu, addr, result);
}

void apu_agb_fifo_write32(GbApu* apu, unsigned addr, unsigned value)
{
    assert(apu_is_agb(apu) && "invalid access");
    struct GbApuFifo* fifo = fifo_from_addr(apu, addr);
    fifo->ring_buf[fifo->w_index] = value;
    fifo->w_index = (fifo->w_index + 1) % FIFO_CAPACITY; // advance write pointer
}

void apu_agb_timer_overflow(GbApu* apu, void* user, apu_agb_fifo_dma_request dma_callback, unsigned timer_num, unsigned time)
{
    assert(apu_is_agb(apu) && "invalid access");
    const unsigned reg = REG_SOUNDCNT_H;

    for (unsigned i = 0; i < apu_array_size(apu->fifo); i++)
    {
        struct GbApuFifo* fifo = &apu->fifo[i];
        const bool timer_select = i == 0 ? (reg & 0x400) : (reg & 0x4000);
        if (timer_select == timer_num)
        {
            /* request dma if there are 4 or more empty words. */
            if (FIFO_CAPACITY - fifo_get_size(fifo) > 4)
            {
                dma_callback(user, i, time);
            }

            // if playing buffer is empty and we have a at least a 32-bit word in the fifo, reload it.
            if (!fifo->playing_buffer_index && fifo_get_size(fifo))
            {
                fifo->playing_buffer_index = 4; // 32-bits, 4 samples
                fifo->playing_buffer = fifo->ring_buf[fifo->r_index]; // load 32-bits, 4 samples
                fifo->r_index = (fifo->r_index + 1) % FIFO_CAPACITY; // advance read pointer
            }

            // if playing buffer isn't empty, reload new sample and shift out the old one.
            if (fifo->playing_buffer_index)
            {
                channel_sync_fifo(apu, ChannelType_FIFOA + i, time);
                fifo->current_sample = fifo->playing_buffer; // load 8-bits
                fifo->playing_buffer >>= 8; // shift out sample
                fifo->playing_buffer_index--; // reduce buffer size
            }
        }
    }
}

unsigned apu_read_io_raw(const GbApu* apu, unsigned addr)
{
    assert((addr & 0xFF) >= 0x10 && (addr & 0xFF) <= 0x3F);
    return apu->io[addr & 0x3F];
}

unsigned apu_agb_read_io_raw(const GbApu* apu, unsigned addr)
{
    addr = AGB_ADDR_TRANSLATION[(addr & 0xFF) - AGB_ADDR_OFFSET];
    return apu_read_io_raw(apu, addr);
}

unsigned apu_agb_soundcnt_read_raw(const GbApu* apu)
{
    return REG_SOUNDCNT_H;
}

unsigned apu_agb_soundbias_read_raw(const GbApu* apu)
{
    return REG_SOUNDBIAS;
}

void apu_set_highpass_filter(GbApu* apu, enum GbApuFilter filter, double clock_rate, double sample_rate)
{
    apu_set_highpass_filter_custom(apu, CHARGE_FACTOR[filter], clock_rate, sample_rate);
}

void apu_set_highpass_filter_custom(GbApu* apu, double charge_factor, double clock_rate, double sample_rate)
{
#if defined(GB_APU_HAS_MATH_H) && GB_APU_HAS_MATH_H
    const double capacitor_charge = pow(apu_clamp(charge_factor, 0.0, 1.0), clock_rate / sample_rate);
    const double fixed_point_scale = 1 << CAPACITOR_SCALE;
    apu->capacitor_charge_factor = round(capacitor_charge * fixed_point_scale);
    memset(apu->capacitor, 0, sizeof(apu->capacitor));
#endif
}

void apu_set_channel_volume(GbApu* apu, unsigned channel_num, float volume)
{
    apu->channel_volume[channel_num] = apu_clamp(volume, 0.0F, 1.0F);
}

void apu_set_master_volume(GbApu* apu, float volume)
{
    blip_wrap_set_volume(apu->blip, apu_clamp(volume, 0.0F, 1.0F));
}

void apu_set_bass(GbApu* apu, int frequency)
{
    blip_wrap_set_bass(apu->blip, frequency);
}

void apu_set_treble(GbApu* apu, double treble_db)
{
    blip_wrap_set_treble(apu->blip, treble_db);
}

int apu_clocks_needed(const GbApu* apu, int sample_count)
{
    return blip_wrap_clocks_needed(apu->blip, sample_count);
}

int apu_samples_avaliable(const GbApu* apu)
{
    return blip_wrap_samples_avail(apu->blip);
}

void apu_end_frame(GbApu* apu, unsigned time)
{
    // catchup all the channels to the same point.
    channel_sync_psg_all(apu, time);
    channel_sync_fifo_all(apu, time);

    // clocks of all channels will be the same as they're synced above.
    const unsigned clock_duration = apu->channels[0].clock;

    // reset clocks.
    for (unsigned i = 0; i < apu_array_size(apu->channels); i++)
    {
        assert(clock_duration == apu->channels[i].clock);
        apu->channels[i].clock = 0;
    }

    // make all samples up to this clock point available.
    blip_wrap_end_frame(apu->blip, clock_duration);
}

int apu_read_samples(GbApu* apu, short out[], int count)
{
    count = blip_wrap_read_samples(apu->blip, out, count);

#if defined(GB_APU_HAS_MATH_H) && GB_APU_HAS_MATH_H
    if (apu->capacitor_charge_factor != CAPACITOR_SCALE) /* only apply if enabled. */
    {
        for (int i = 0; i < count; i += 2)
        {
            out[i + 0] = high_pass(apu->capacitor_charge_factor, out[i + 0], &apu->capacitor[0]);
            out[i + 1] = high_pass(apu->capacitor_charge_factor, out[i + 1], &apu->capacitor[1]);
        }
    }
#endif

    return count;
}

void apu_clear_samples(GbApu* apu)
{
    blip_wrap_clear(apu->blip);
}

#if (defined(__cplusplus) && __cplusplus < 201103L) || (!defined(static_assert))
  #if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
    #define static_assert _Static_assert
  #else
    #define static_assert(expr, msg) typedef char static_assertion[(expr) ? 1 : -1]
  #endif
#endif

static_assert(offsetof(GbApu, channels) == 0, "bad channels offset, save states broken!");
static_assert(offsetof(GbApu, len) == 120, "bad len offset, save states broken!");
static_assert(offsetof(GbApu, env) == 128, "bad env offset, save states broken!");
static_assert(offsetof(GbApu, sweep) == 144, "bad sweep offset, save states broken!");
static_assert(offsetof(GbApu, square) == 152, "bad square offset, save states broken!");
static_assert(offsetof(GbApu, wave) == 154, "bad wave offset, save states broken!");
static_assert(offsetof(GbApu, noise) == 158, "bad noise offset, save states broken!");
static_assert(offsetof(GbApu, fifo) == 160, "bad fifo offset, save states broken!");
static_assert(offsetof(GbApu, frame_sequencer) == 248, "bad frame_sequencer offset, save states broken!");
static_assert(offsetof(GbApu, agb_soundcnt) == 252, "bad agb_soundcnt offset, save states broken!");
static_assert(offsetof(GbApu, agb_soundbias) == 254, "bad agb_soundbias offset, save states broken!");
static_assert(offsetof(GbApu, io) == 256, "bad io offset, save states broken!");
static_assert(offsetof(GbApu, blip) == 336, "bad blip offset, save states broken!");

unsigned apu_state_size(void)
{
    return offsetof(GbApu, blip);
}

int apu_save_state(const GbApu* apu, void* data, unsigned size)
{
    if (!data || size < apu_state_size())
    {
        return 1;
    }

    return !memcpy(data, apu, apu_state_size());
}

int apu_load_state(GbApu* apu, const void* data, unsigned size)
{
    if (!data || size < apu_state_size())
    {
        return 1;
    }

    return !memcpy(apu, data, apu_state_size());
}
