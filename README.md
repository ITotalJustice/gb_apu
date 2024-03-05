# gb_apu

implementation of the Gameboy apu in ~1k lines of code.

this uses blip_buf internally to resample the psg channels to your desired output rate.

when using this library, you have the choice between using blip_buf or Blip_buffer, the latter being c++.

using the latter will allow you to have bass and treble control, as well as sounding slightly better sounds slightly better. Other than that, both function the same and the api remains the same, and the exposed api doesn't change.

The `C` version compiles down to ~25 KiB in release mode.

The `CXX` version compiles down to ~34 KiB in release mode.

---

## Accurary

gb_apu passes all of [blarggs dmg_sound and cgb_sound tests](https://gbdev.gg8.se/wiki/articles/Test_ROMs) and all audio tests in [numism](https://github.com/pinobatch/numism).

The level of accurary does depend on your emulation of the Gameboy timer. For example, the `apu_frame_sequencer_clock()` should be called on the falling edge (bit goes from 1 -> 0) of bit 4 of `DIV`. In double speed mode, this is the falling edge of bit 5. Be aware that DIV can be written to which sets `DIV` to 0, which could cause an early clock, or, result in no clocks if `DIV` is written to frequently.

gb_apu currently doesn't accurately emulate ["zombie mode"](https://gbdev.gg8.se/wiki/articles/Gameboy_sound_hardware#Obscure_Behavior). Basic emulation of it exists, and can be enabled with `-DGB_APU_ZOMBIE=ON`.

---

## adding this to your project

first you need to choose between the `C` and `CPP` version. The exposed api is written in `C`, so as long as you have a `CPP` compiler, you should chose that version.

Please give credit if using any of this code
To add this to your project, add the following files depending on your language of choice:

### C

- gb_apu.c
- blip_wrap.c
- blip_buf.c

### CPP

- gb_apu.c
- blip_wrap.cpp
- Blip_Buffer.cpp

---

you can also use the included cmake file:

```cmake
set(GB_APU_CXX OFF) # set ON if wanting Blip_Buffer
set(GB_APU_ZOMBIE OFF) # set ON for zombie mode emulation
add_subdirectory(gb_apu)
target_link_libraries(your_exe PRIVATE gb_apu)
```

or use cmake's fetch content:

```cmake
include(FetchContent)

FetchContent_Declare(gb_apu
    GIT_REPOSITORY https://github.com/ITotalJustice/gb_apu.git
    GIT_TAG        v1.2.0
)

set(GB_APU_CXX OFF) # set ON if wanting Blip_Buffer
set(GB_APU_ZOMBIE OFF) # set ON for zombie mode emulation
FetchContent_MakeAvailable(gb_apu)

target_link_libraries(your_exe PRIVATE gb_apu)
```

---

## License

- gb_apu is licenced under MIT.
- blip_buf and Blip_Buffer are licenced under LGPL.

If you cannot use gpl code in your project, create a `blip_wrap.c/cpp` file with your own bandlimited synthesis code, or remove all `blip_wrap_` functions from `gb_apu.c` and sample the channels however you wish.

---

## Credits

- [blip_buf by blargg](https://code.google.com/archive/p/blip-buf/)
- [Blip_Buffer by blargg](https://code.google.com/archive/p/blip-buffer/)
- https://www.slack.net/~ant/bl-synth/
- https://gbdev.gg8.se/wiki/articles/Gameboy_sound_hardware
