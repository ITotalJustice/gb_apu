#ifndef _GB_APU_H_
#define _GB_APU_H_

#include <stdint.h>
#include "blip_wrap.h"

#ifdef __cplusplus
extern "C" {
#endif

enum GbApuType
{
    GbApuType_DMG,
    GbApuType_CGB,
};

struct GbApuFrameSequencer
{
    unsigned index;
};

struct GbApuLen
{
    unsigned counter;
};

struct GbApuSweep
{
    unsigned freq_shadow_register;
    unsigned timer;
    unsigned enabled;
    unsigned did_negate;
};

struct GbApuEnvelope
{
    unsigned volume;
    unsigned timer;
    unsigned disable;
};

struct GbApuSquare
{
    unsigned duty_index;
};

struct GbApuWave
{
    unsigned sample_buffer;
    unsigned position_counter;
    unsigned just_accessed;
};

struct GbApuNoise
{
    unsigned lfsr;
};

struct GbApuChannel
{
    unsigned clock; /* clock used for blip_buf. */
    unsigned timestamp; /* timestamp since last tick(). */
    int amp[2]; /* last volume output left/right. */
    int frequency_timer; /* freq that's counted down every tick. */
};

struct GbApu
{
    blip_wrap_t* blip;
    float channel_volume[4];
    enum GbApuType type;

    /* for savestates, back up everything here. */
    struct GbApuChannel channels[4];
    struct GbApuLen len[4]; /* every channel has one. */
    struct GbApuEnvelope env[4]; /* all channels bar wave. */
    struct GbApuSweep sweep;
    struct GbApuSquare square[2];
    struct GbApuWave wave;
    struct GbApuNoise noise;
    struct GbApuFrameSequencer frame_sequencer;
    uint8_t io[0x40];
    /* end. */
};

/* ensure you call this at start-up */
void apu_init(struct GbApu*, double clock_rate, double sample_rate);
/* call to free allocated memory by blip buf. */
void apu_quit(struct GbApu*);
/* clock_rate should be the cpu speed of the system. */
void apu_reset(struct GbApu*, enum GbApuType type);

/* reads from io register, unused bits are set to 1. */
unsigned apu_read_io(struct GbApu*, unsigned addr, unsigned time);
/* writes to an io register. */
void apu_write_io(struct GbApu*, unsigned addr, unsigned value, unsigned time);
/* call this on the falling edge of bit 4/5 of DIV. */
void apu_frame_sequencer_clock(struct GbApu*, unsigned time, unsigned late);

/* channel volume, max range: 0.0 - 1.0. */
void apu_set_channel_volume(struct GbApu*, unsigned channel_num, float volume);
/* master volume, max range: 0.0 - 1.0. */
void apu_set_master_volume(struct GbApu*, float volume);
/* only available with Blip_Buffer. */
void apu_set_bass(struct GbApu*, int frequency);
/* only available with Blip_Buffer. */
void apu_set_treble(struct GbApu*, double treble_db);

/* returns how many cycles are needed until sample_count == apu_samples_avaliable() */
int apu_clear_clocks_needed(struct GbApu*, int sample_count);
/* call this when you want to read out samples. */
void apu_end_frame(struct GbApu*, unsigned time, unsigned late);
/* returns how many samples are available, call apu_end_frame() first. */
int apu_samples_avaliable(const struct GbApu*);
/* read stereo samples, returns the amount read. */
int apu_read_samples(struct GbApu*, short out[], int count);
/* removes all samples. */
void apu_clear_samples(struct GbApu*);

#ifdef __cplusplus
}
#endif

#endif /* _GB_APU_H_ */
