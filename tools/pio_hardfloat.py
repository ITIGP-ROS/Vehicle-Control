"""
pio_hardfloat.py - put the FPU flags on the LINK line as well as the compile line.

WHY THIS FILE EXISTS
--------------------
Enabling the Cortex-M4F FPU (T5-3) needs -mfpu=fpv4-sp-d16 -mfloat-abi=hard on
BOTH steps:

  * compiling - so the compiler emits VFP instructions and the hard-float ABI;
    `build_flags` in platformio.ini already covers this.
  * LINKING   - because gcc chooses the newlib/libgcc MULTILIB from the flags on
    the link command. Without them it silently picks the soft-float
    `arm-none-eabi/lib/armv7e-m/` variant and the link fails with:

        error: firmware.elf uses VFP register arguments,
               .../armv7e-m/libc.a(lib_a-memset.o) does not

The `titiva` platform builder passes only `-mcpu=cortex-m4 -mthumb` to the link
step and does NOT forward `build_flags` there. platformio.ini's `link_flags` key
is not a valid PlatformIO option - the build prints
"Ignore unknown configuration option `link_flags`" and drops it, which is the
same root cause as the `--gc-sections`/`-Wl,-Map` finding in REVIEW 01 Part-4.
`extra_scripts` is the supported way to reach LINKFLAGS.

SCOPE: this script deliberately adds ONLY the two float flags. It does NOT
re-enable --gc-sections or -Wl,-Map, even though the same mechanism would work -
that is a separate, still-open finding and re-enabling dead-code stripping wants
its own review.

The hard-float multilib is present in the build toolchain (4.8.4):
    arm-none-eabi/lib/armv7e-m/fpu/libc.a
      == @mthumb@march=armv7e-m@mfloat-abi=hard@mfpu=fpv4-sp-d16
"""

Import("env")

FPU_FLAGS = ["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]

env.Append(LINKFLAGS=FPU_FLAGS)
