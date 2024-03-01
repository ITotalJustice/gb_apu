#include "gb_apu.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

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

enum ChannelType {
    ChannelType_SQUARE0,
    ChannelType_SQUARE1,
    ChannelType_WAVE,
    ChannelType_NOISE,
};

// values of zero are treated as 8.
static const uint8_t PERIOD_TABLE[8] = {
    8, 1, 2, 3, 4, 5, 6, 7
};

// NOTE: this table is inverted on agb, make adjustments as needed.
static const uint8_t SQUARE_DUTY_CYCLES[4][8] = {
    { 0, 0, 0, 0, 0, 0, 0, 1 }, // 12.5%
    { 1, 0, 0, 0, 0, 0, 0, 1 }, // 25%
    { 1, 0, 0, 0, 0, 1, 1, 1 }, // 50%
    { 0, 1, 1, 1, 1, 1, 1, 0 }, // 75%
};

static const uint8_t WAVE_VOLUME_DIVIDER[4] = {
    4, // 0%
    0, // 100%
    1, // 50%
    2, // 25%
};

static const uint8_t NOISE_DIVISOR[8] = {
    8, 16, 32, 48, 64, 80, 96, 112
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

// reads return the register or'd with this table
static const uint8_t IO_READ_VALUE[0x40] = {
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

    // this can be read back as 0xFF or the actual value
    [0x30] = 0x00, // WAVE RAM
    [0x31] = 0x00, // WAVE RAM
    [0x32] = 0x00, // WAVE RAM
    [0x33] = 0x00, // WAVE RAM
    [0x34] = 0x00, // WAVE RAM
    [0x35] = 0x00, // WAVE RAM
    [0x36] = 0x00, // WAVE RAM
    [0x37] = 0x00, // WAVE RAM
    [0x38] = 0x00, // WAVE RAM
    [0x39] = 0x00, // WAVE RAM
    [0x3A] = 0x00, // WAVE RAM
    [0x3B] = 0x00, // WAVE RAM
    [0x3C] = 0x00, // WAVE RAM
    [0x3D] = 0x00, // WAVE RAM
    [0x3E] = 0x00, // WAVE RAM
    [0x3F] = 0x00, // WAVE RAM
};

static inline bool is_apu_enabled(const struct GbApu* apu)
{
    return REG_NR52 & 0x80;
}

static inline void channel_enable(struct GbApu* apu, unsigned num)
{
    REG_NR52 |= 1 << num;
}

static inline void channel_disable(struct GbApu* apu, unsigned num)
{
    REG_NR52 &= ~(1 << num);
    apu->channels[num].frequency_timer = 0;
}

static inline bool channel_is_enabled(const struct GbApu* apu, unsigned num)
{
    return REG_NR52 & (1 << num);
}

static inline bool channel_is_dac_enabled(const struct GbApu* apu, unsigned num)
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

static unsigned channel_get_frequency(const struct GbApu* apu, unsigned num)
{
    if (num == ChannelType_SQUARE0)
    {
        const unsigned freq = ((REG_NR14 & 7) << 8) | REG_NR13;
        return (2048 - freq) * 4;
    }
    else if (num == ChannelType_SQUARE1)
    {
        const unsigned freq = ((REG_NR24 & 7) << 8) | REG_NR23;
        return (2048 - freq) * 4;
    }
    else if (num == ChannelType_WAVE)
    {
        const unsigned freq = ((REG_NR34 & 7) << 8) | REG_NR33;
        return (2048 - freq) * 2;
    }
    else // if (num == ChannelType_NOISE)
    {
        const unsigned divisor_code = REG_NR43 & 0x7;
        const unsigned clock_shift = REG_NR43 >> 4;
        return NOISE_DIVISOR[divisor_code] << clock_shift;
    }
}

static inline void add_delta(struct GbApu* apu, struct GbApuChannel* c, unsigned clock_time, int sample, unsigned lr)
{
    const int delta = sample - c->amp[lr];
    if (delta) // same as (sample != amp)
    {
        blip_wrap_add_delta(apu->blip, clock_time, delta, lr);
        c->amp[lr] += delta; // same as (amp = sample)
    }
}

static inline void add_delta_fast(struct GbApu* apu, struct GbApuChannel* c, unsigned clock_time, int sample, unsigned lr)
{
    const int delta = sample - c->amp[lr];
    if (delta) // same as (sample != amp)
    {
        blip_wrap_add_delta_fast(apu->blip, clock_time, delta, lr);
        c->amp[lr] += delta; // same as (amp = sample)
    }
}

static void channel_sync(struct GbApu* apu, unsigned num, unsigned time, unsigned late)
{
    struct GbApuChannel* c = &apu->channels[num];

    // get starting point
    const unsigned base_clock = c->clock;
    // i am not 100% sure how this works, but trust me, it works
    unsigned from = base_clock + c->frequency_timer;
    // get new timestamp
    const unsigned new_timestamp = time - late;
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

    if (!is_apu_enabled(apu) || !channel_is_enabled(apu, num))
    {
        add_delta(apu, c, from, 0, 0);
        add_delta(apu, c, from, 0, 1);
        return;
    }

    const bool left_enabled = REG_NR51 & (1 << (num + 0));
    const bool right_enabled = REG_NR51 & (1 << (num + 4));
    const int left_volume = left_enabled * (1 + ((REG_NR50 >> 0) & 0x7));
    const int right_volume = right_enabled * (1 + ((REG_NR50 >> 4) & 0x7));

    const unsigned freq = channel_get_frequency(apu, num);
    const float volume = apu->channel_volume[num];

    c->frequency_timer -= until;

    // the below can be further optimised, should you need to.
    if (num == ChannelType_SQUARE0 || num == ChannelType_SQUARE1)
    {
        struct GbApuSquare* square = &apu->square[num];

        const unsigned duty = apu->io[SQAURE_DUTY_ADDR[num]] >> 6;
        unsigned duty_bit = SQUARE_DUTY_CYCLES[duty][square->duty_index];
        const int sign_flipflop = duty_bit ? +1 : -1;

        int left = blip_apply_volume_to_sample(apu->blip, c->env.volume * left_volume * sign_flipflop, volume);
        int right = blip_apply_volume_to_sample(apu->blip, c->env.volume * right_volume * sign_flipflop, volume);
        add_delta(apu, c, from, left, 0);
        add_delta(apu, c, from, right, 1);

        while (c->frequency_timer <= 0)
        {
            square->duty_index = (square->duty_index + 1) % 8;
            const unsigned new_duty_bit = SQUARE_DUTY_CYCLES[duty][square->duty_index];
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

        const unsigned volume_divider = WAVE_VOLUME_DIVIDER[(REG_NR32 >> 5) & 0x3];
        int sample = (wave->position_counter & 0x1) ? wave->sample_buffer & 0xF : wave->sample_buffer >> 4;
        sample = (sample * 2 - 15) >> volume_divider; // [-15,+15]

        int left = blip_apply_volume_to_sample(apu->blip, sample * left_volume, volume);
        int right = blip_apply_volume_to_sample(apu->blip, sample * left_volume, volume);
        add_delta_fast(apu, c, from, left, 0);
        add_delta_fast(apu, c, from, right, 1);

        while (c->frequency_timer <= 0)
        {
            wave->position_counter = (wave->position_counter + 1) % 32;

            // fetch new sample if we are done with this buffer
            if (!(wave->position_counter & 0x1))
            {
                wave->sample_buffer = REG_WAVE_TABLE[wave->position_counter >> 1];
            }

            sample = (wave->position_counter & 0x1) ? wave->sample_buffer & 0xF : wave->sample_buffer >> 4;
            sample = (sample * 2 - 15) >> volume_divider; // [-15,+15]

            int left = blip_apply_volume_to_sample(apu->blip, sample * left_volume, volume);
            int right = blip_apply_volume_to_sample(apu->blip, sample * left_volume, volume);
            add_delta_fast(apu, c, from, left, 0);
            add_delta_fast(apu, c, from, right, 1);

            from += freq;
            c->frequency_timer += freq;
        }

    }
    else if (num == ChannelType_NOISE)
    {
        struct GbApuNoise* noise = &apu->noise;

        unsigned bit0 = noise->lfsr & 0x1;
        const int sign_flipflop = bit0 ? -1 : +1; // inverted

        int left = blip_apply_volume_to_sample(apu->blip, c->env.volume * left_volume * sign_flipflop, volume);
        int right = blip_apply_volume_to_sample(apu->blip, c->env.volume * right_volume * sign_flipflop, volume);
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

static void channel_sync_all(struct GbApu* apu, unsigned time, unsigned late)
{
    channel_sync(apu, ChannelType_SQUARE0, time, late);
    channel_sync(apu, ChannelType_SQUARE1, time, late);
    channel_sync(apu, ChannelType_WAVE, time, late);
    channel_sync(apu, ChannelType_NOISE, time, late);
}

// this is used when a channel is triggered
static bool is_next_frame_sequencer_step_not_len(const struct GbApu* apu)
{
    // check if the current counter is the len clock, the next one won't be!
    return apu->frame_sequencer.index & 0x1;
}

// this is used when channels 1,2,4 are triggered
static bool is_next_frame_sequencer_step_vol(const struct GbApu* apu)
{
    // check if the current counter is the len clock, the next one won't be!
    return apu->frame_sequencer.index == 7;
}

static unsigned sweep_get_new_freq(struct GbApu* apu)
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

static void sweep_do_freq_calc(struct GbApu* apu, bool update_value)
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

static void sweep_clock(struct GbApu* apu, unsigned num, unsigned time, unsigned late)
{
    struct GbApuSweep* sweep = &apu->sweep;

    if (channel_is_enabled(apu, num) && sweep->enabled)
    {
        assert(sweep->timer > 0 && sweep->timer <= 8);
        sweep->timer--;

        if (sweep->timer == 0)
        {
            const unsigned period = (REG_NR10 >> 4) & 0x7;
            sweep->timer = PERIOD_TABLE[period];

            // sweep is only clocked if period is not 0
            if (period != 0)
            {
                channel_sync(apu, num, time, late);
                // first time updates the value
                sweep_do_freq_calc(apu, true);
                // second time does not, but still checks for overflow
                sweep_do_freq_calc(apu, false);
            }
        }
    }
}

static void sweep_trigger(struct GbApu* apu)
{
    apu->sweep.did_negate = false;

    // reload sweep timer with period
    const unsigned period = (REG_NR10 >> 4) & 0x7;
    apu->sweep.timer = PERIOD_TABLE[period];

    // the freq is loaded into the shadow_freq_reg
    apu->sweep.freq_shadow_register = ((REG_NR14 & 7) << 8) | REG_NR13;

    // sweep is enabled flag if period or shift is non zero
    const unsigned shift = REG_NR10 & 0x7;
    apu->sweep.enabled = (period != 0) || (shift != 0);

    // sweep calc is performed, but the value isn't updated
    // this means that it only really checks if the value overflows
    // if it does, then ch1 is disabled.
    if (shift)
    {
        sweep_do_freq_calc(apu, false);
    }
}

static bool len_is_enabled(const struct GbApu* apu, unsigned num)
{
    return apu->io[LEN_REG_ADDR[num]] & 0x40;
}

static void len_clock(struct GbApu* apu, unsigned num, unsigned time, unsigned late)
{
    struct GbApuLen* len = &apu->channels[num].len;

    if (channel_is_enabled(apu, num) && len_is_enabled(apu, num) && len->counter > 0)
    {
        len->counter--;
        if (len->counter == 0)
        {
            channel_sync(apu, num, time, late);
            channel_disable(apu, num);
        }
    }
}

static void len_on_nrx4_edge_case_write(struct GbApu* apu, unsigned num, unsigned new_value, unsigned old_value)
{
    struct GbApuLen* len = &apu->channels[num].len;

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

static void len_trigger(struct GbApu* apu, unsigned num)
{
    struct GbApuLen* len = &apu->channels[num].len;

    if (len->counter == 0)
    {
        len->counter = LEN_RELOAD_VALUE[num];
        if (len_is_enabled(apu, num) && is_next_frame_sequencer_step_not_len(apu))
        {
            len->counter--; // will this disable the channel?
        }
    }
}

static void env_clock(struct GbApu* apu, unsigned num, unsigned time, unsigned late)
{
    struct GbApuEnvelope* env = &apu->channels[num].env;

    if (channel_is_enabled(apu, num) && !env->disable)
    {
        env->timer--;
        if (env->timer == 0)
        {
            const unsigned reg = apu->io[ENV_REG_ADDR[num]];
            const unsigned period = reg & 0x7;
            env->timer = PERIOD_TABLE[period];

            if (period != 0)
            {
                const unsigned mode = (reg >> 3) & 0x1;
                const unsigned modifier = mode == 1 ? +1 : -1;
                const unsigned new_volume = env->volume + modifier;

                if (new_volume <= 15)
                {
                    channel_sync(apu, num, time, late);
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

static void env_trigger(struct GbApu* apu, unsigned num)
{
    struct GbApuEnvelope* env = &apu->channels[num].env;

    const unsigned reg = apu->io[ENV_REG_ADDR[num]];
    const unsigned period = reg & 0x7;
    const unsigned starting_vol = reg >> 4;;

    env->disable = false;
    env->timer = PERIOD_TABLE[period];
    if (is_next_frame_sequencer_step_vol(apu))
    {
        env->timer++;
    }

    // reload the volume
    env->volume = starting_vol;
}

static void env_write(struct GbApu* apu, unsigned num, unsigned new_value, unsigned old_value)
{
    // todo: zombie mode, prehistorik man needed this
    if (!channel_is_dac_enabled(apu, num))
    {
        channel_disable(apu, num);
    }
}

static void trigger(struct GbApu* apu, unsigned num, unsigned time)
{
    channel_enable(apu, num);
    const unsigned new_freq = channel_get_frequency(apu, num);
    struct GbApuChannel* c = &apu->channels[num];

    len_trigger(apu, num);

    if (num == ChannelType_WAVE)
    {
        apu->wave.position_counter = 0;
        // 6 cycle delay until first clock
        c->frequency_timer = new_freq + 6;
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

static void on_nrx0_write(struct GbApu* apu, unsigned num, unsigned time, unsigned new_value, unsigned old_value)
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

static void on_nrx1_write(struct GbApu* apu, unsigned num, unsigned time, unsigned new_value, unsigned old_value)
{
    const unsigned reload_value = LEN_RELOAD_VALUE[num];
    const unsigned mask = reload_value - 1;
    apu->channels[num].len.counter = reload_value - (new_value & mask);
}

static void on_nrx2_write(struct GbApu* apu, unsigned num, unsigned time, unsigned new_value, unsigned old_value)
{
    if (num != ChannelType_WAVE)
    {
        env_write(apu, num, new_value, old_value);
    }
}

static void on_nrx3_write(struct GbApu* apu, unsigned num, unsigned time, unsigned new_value, unsigned old_value)
{
    // nothing special happens here...
}

static void on_nrx4_write(struct GbApu* apu, unsigned num, unsigned time, unsigned new_value, unsigned old_value)
{
    len_on_nrx4_edge_case_write(apu, num, new_value, old_value);

    if (new_value & 0x80)
    {
        trigger(apu, num, time);
    }
}

static void frame_sequencer_clock_len(struct GbApu* apu, unsigned time, unsigned late)
{
    len_clock(apu, ChannelType_SQUARE0, time, late);
    len_clock(apu, ChannelType_SQUARE1, time, late);
    len_clock(apu, ChannelType_WAVE, time, late);
    len_clock(apu, ChannelType_NOISE, time, late);
}

static void frame_sequencer_clock_sweep(struct GbApu* apu, unsigned time, unsigned late)
{
    sweep_clock(apu, ChannelType_SQUARE0, time, late);
}

static void frame_sequencer_clock_env(struct GbApu* apu, unsigned time, unsigned late)
{
    env_clock(apu, ChannelType_SQUARE0, time, late);
    env_clock(apu, ChannelType_SQUARE1, time, late);
    env_clock(apu, ChannelType_NOISE, time, late);
}

/* ------------------PUBLIC API------------------ */
void apu_init(struct GbApu* apu, double clock_rate, double sample_rate)
{
    assert(clock_rate > 0.0 && sample_rate > 0.0);
    memset(apu, 0, sizeof(*apu));

    for (int i = 0; i < 4; i++)
    {
        apu->channel_volume[i] = 1.0;
    }

    apu->blip = blip_wrap_new(sample_rate);
    blip_wrap_set_rates(apu->blip, clock_rate, sample_rate);
    blip_wrap_set_volume(apu->blip, 0.5);
}

void apu_quit(struct GbApu* apu)
{
    if (apu->blip)
    {
        blip_wrap_delete(apu->blip);
        apu->blip = NULL;
    }
}

void apu_reset(struct GbApu* apu, unsigned skip_bios)
{
    apu_clear_samples(apu);

    memset(&apu->channels, 0, sizeof(apu->channels));
    memset(&apu->sweep, 0, sizeof(apu->sweep));
    memset(&apu->square, 0, sizeof(apu->square));
    memset(&apu->wave, 0, sizeof(apu->wave));
    memset(&apu->noise, 0, sizeof(apu->noise));
    memset(&apu->frame_sequencer, 0, sizeof(apu->frame_sequencer));
    memset(apu->io + 0x10, 0, 0x17);

    apu->noise.lfsr = 0x7FFF;

    // for (int i = 0; i < 4; i++)
    // {
    //     apu->channels[i].env.disable = true;
    // }

    // todo: confirm the below
    if (skip_bios)
    {
        REG_NR10 = 0x80;
        REG_NR11 = 0xBF;
        REG_NR12 = 0xF3;
        REG_NR14 = 0xBF;
        REG_NR21 = 0x3F;
        REG_NR22 = 0x00;
        REG_NR24 = 0xBF;
        REG_NR30 = 0x7F;
        REG_NR31 = 0xFF;
        REG_NR32 = 0x9F;
        REG_NR33 = 0xBF;
        REG_NR41 = 0xFF;
        REG_NR42 = 0x00;
        REG_NR44 = 0xBF;
        REG_NR50 = 0x77;
        REG_NR51 = 0xF3;
        REG_NR52 = 0xF1;
    }

    for (int i = 0; i < 0x10; i+= 2)
    {
        REG_WAVE_TABLE[i + 0] = 0x00;
        REG_WAVE_TABLE[i + 1] = 0xFF;
    }
}

unsigned apu_read_io(struct GbApu* apu, unsigned addr, unsigned time)
{
    assert((addr & 0xFF) >= 0x10 && (addr & 0xFF) <= 0x3F);
    addr &= 0x3F;

    if (addr >= 0x30 && addr <= 0x3F && channel_is_enabled(apu, ChannelType_WAVE))
    {
        #if 1
        // dmg behaviour
        if (time - apu->channels[ChannelType_WAVE].timestamp < 6)
        {
            return REG_WAVE_TABLE[apu->wave.position_counter >> 1];
        }
        else
        {
            return 0xFF;
        }
        #else
        // cgb always returns from the table.
        return REG_WAVE_TABLE[apu->wave.position_counter >> 1];
        #endif
    }

    return apu->io[addr] | IO_READ_VALUE[addr];
}

void apu_write_io(struct GbApu* apu, unsigned addr, unsigned value, unsigned time)
{
    assert((addr & 0xFF) >= 0x10 && (addr & 0xFF) <= 0x3F);
    addr &= 0x3F;

    // nr52 is always writeable
    if (addr == 0x26) // nr52
    {
        // check if it's now disabled
        if (is_apu_enabled(apu) && !(value & 0x80))
        {
            channel_sync_all(apu, time, 0);

            // reset everything aside from wave ram
            memset(apu->io + 0x10, 0, 0x17);
            apu->square[0].duty_index = 0;
            apu->square[1].duty_index = 0;
            apu->wave.sample_buffer = 0;
        }
        // check if it's now enabled
        else if (!is_apu_enabled(apu) && (value & 0x80))
        {
            REG_NR52 |= 0x80;
            apu->frame_sequencer.index = 0;
        }
    }
    // writes to the apu are ignore whilst disabled
    else if (!is_apu_enabled(apu))
    {
        return;
    }
    else if (addr == 0x24 || addr == 0x25) // nr50 | nr51
    {
        channel_sync_all(apu, time, 0);
        apu->io[addr] = value;
    }
    else if (addr >= 0x30 && addr <= 0x3F) // wave ram
    {
        channel_sync(apu, ChannelType_WAVE, time, 0);

        // if enabled, writes are ignored on dmg, allowed on cgb.
        if (channel_is_enabled(apu, ChannelType_WAVE))
        {
            // will this reset the position counter?
            // does the channel play this newly written value?
            REG_WAVE_TABLE[apu->wave.position_counter >> 1] = value;
        }
        else
        {
            apu->io[addr] = value;
        }
    }
    else
    {
        const unsigned num = IO_CHANNEL_NUM[addr] & 0x3;
        channel_sync(apu, num, time, 0);

        const unsigned nrxx = IO_CHANNEL_NUM[addr] & ~0x3;
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

void apu_frame_sequencer_clock(struct GbApu* apu, unsigned time, unsigned late)
{
    if (!is_apu_enabled(apu))
    {
        return;
    }

    switch (apu->frame_sequencer.index)
    {
        case 0: // len
        case 4:
            frame_sequencer_clock_len(apu, time, late);
            break;

        case 2: // len, sweep
        case 6:
            frame_sequencer_clock_len(apu, time, late);
            frame_sequencer_clock_sweep(apu, time, late);
            break;

        case 7: // vol
            frame_sequencer_clock_env(apu, time, late);
            break;
    }

    apu->frame_sequencer.index = (apu->frame_sequencer.index + 1) % 8;
}

void apu_set_channel_volume(struct GbApu* apu, unsigned channel_num, float volume)
{
    // clip range 0.0 - 1.0
    volume = volume < 0.0F ? 0.0F : volume > 1.0F ? 1.0F : volume;
    apu->channel_volume[channel_num] = volume;
}

void apu_set_master_volume(struct GbApu* apu, float volume)
{
    // clip range 0.0 - 1.0
    volume = volume < 0.0F ? 0.0F : volume > 1.0F ? 1.0F : volume;
    blip_wrap_set_volume(apu->blip, volume);
}

void apu_set_bass(struct GbApu* apu, int frequency)
{
    blip_wrap_set_bass(apu->blip, frequency);
}

void apu_set_treble(struct GbApu* apu, double treble_db)
{
    blip_wrap_set_treble(apu->blip, treble_db);
}

int apu_clear_clocks_needed(struct GbApu* apu, int sample_count)
{
    return blip_wrap_clocks_needed(apu->blip, sample_count);
}

void apu_end_frame(struct GbApu* apu, unsigned time, unsigned late)
{
    // catchup all the channels to the same point.
    channel_sync_all(apu, time, late);

    // clocks of all channels will be the same as they're synced above.
    const unsigned clock_duration = apu->channels[0].clock;

    // reset clocks.
    for (int i = 0; i < 4; i++)
    {
        assert(clock_duration == apu->channels[i].clock);
        apu->channels[i].clock = 0;
    }

    // make all samples up to this clock point available.
    blip_wrap_end_frame(apu->blip, clock_duration);
}

int apu_samples_avaliable(const struct GbApu* apu)
{
    return blip_wrap_samples_avail(apu->blip);
}

int apu_read_samples(struct GbApu* apu, short out[], int count)
{
    return blip_wrap_read_samples(apu->blip, out, count);
}

void apu_clear_samples(struct GbApu* apu)
{
    blip_wrap_clear(apu->blip);
}
