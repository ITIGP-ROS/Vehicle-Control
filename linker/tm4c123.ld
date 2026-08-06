MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

ENTRY(ResetISR)

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))  /* Vector table FIRST */
        *(.text*)
        *(.rodata*)
    } > FLASH

    /* ---- .ARM.extab / .ARM.exidx : the ARM exception-unwind tables ----------
     *
     * ⚠️ FIXED AT B6, 2026-08-06. THIS WAS A LATENT LINKER BUG, on the open list
     * as "the .ARM.exidx linker gap", and B6 is simply the commit whose extra
     * code finally detonated it:
     *
     *     ld: section .ARM.exidx loaded at [0xda30,0xda37] overlaps
     *         section .data loaded at [0xda30,0xdacf]
     *
     * CAUSE: the toolchain always emits .ARM.exidx, but this script gave it no
     * output section - so the linker placed it wherever the FLASH location
     * counter happened to be, i.e. immediately after .text. Meanwhile .data's
     * load address was hard-coded as ADDR(.text) + SIZEOF(.text) - the SAME
     * address. The two had been silently stacked on top of each other all along;
     * it only became an ERROR once .ARM.exidx became non-empty.
     *
     * ⚠️ HOW BAD WAS IT BEFORE? Harmless, but only by luck: while exidx was
     * empty it occupied 0 bytes, so nothing was actually overwritten. Had it
     * grown without the linker noticing the overlap, ResetISR's .data copy would
     * have loaded unwind-table bytes into .data as initial values - producing
     * silently corrupt initialised variables at boot, with no fault. Exactly the
     * class of bug that is nearly impossible to find from the symptom.
     *
     * FIX: give both tables a real output section in FLASH, then derive _etext
     * from where the FLASH counter ACTUALLY ends rather than assuming .text is
     * last. .data's load address follows _etext, so the copy source stays
     * correct by construction instead of by coincidence.
     *
     * KEPT rather than /DISCARD/-ed: 8 bytes is not worth the risk, since libgcc
     * unwind paths can reference __exidx_start/__exidx_end.
     *----------------------------------------------------------------------- */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH

    /* _etext = the first free FLASH address after EVERY loaded section. This is
     * the source address ResetISR copies .data from (startup_tm4c123.c). */
    . = ALIGN(4);
    _etext = .;

    .data : AT(_etext)
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > SRAM

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM
    
    _end = _ebss;  /* End of heap memory */
}

/* The REAL stack: the Cortex-M loads the initial SP from vector[0], which
 * startup_tm4c123.c fills with &__STACK_TOP. So the stack starts at the TOP of
 * SRAM and grows DOWN toward .bss.
 *
 * T5-4: there used to be a 512-byte `.stack` (NOLOAD) section placed just after
 * .bss, defining _stack_start/_stack_end. Nothing ever referenced those symbols
 * and nothing pointed the stack at that region - the stack has always been at
 * the top of RAM. It was a phantom that guarded nothing while implying the
 * opposite, so it has been removed and replaced with the real guard below. */
_estack = ORIGIN(SRAM) + LENGTH(SRAM);
__STACK_TOP = _estack;

/* Minimum head-room between the end of .bss and the stack top. Nothing enforces
 * the two never meet at RUN time, so enforce a floor at LINK time instead: if
 * static allocation ever grows enough to squeeze the stack, the build fails
 * rather than the vehicle corrupting .bss in the field.
 *
 * 4 KB is deliberately conservative. Measured worst case is far smaller - the
 * deepest call path is the super-loop (main -> JetsonComm_Poll -> Can_Receive ->
 * RingBuffer_Pop) plus one exception frame, and with the FPU enabled an
 * FPU-using handler adds ~68 bytes of lazy-stacked state. Today .bss ends at
 * 0x20002B58 leaving 21,672 bytes free, so this floor has ~17 KB of slack for
 * future statics before it would ever trip. */
_min_stack_headroom = 4K;
ASSERT((_estack - _ebss) >= _min_stack_headroom,
       "ERROR: less than 4K of RAM left between the end of .bss and the stack top")