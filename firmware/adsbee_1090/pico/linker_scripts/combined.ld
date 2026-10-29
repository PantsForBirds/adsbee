/* Based on GCC ARM embedded samples.
   Defines the following symbols for use by code:
    __exidx_start
    __exidx_end
    __etext
    __data_start__
    __preinit_array_start
    __preinit_array_end
    __init_array_start
    __init_array_end
    __fini_array_start
    __fini_array_end
    __data_end__
    __bss_start__
    __bss_end__
    __end__
    end
    __HeapLimit
    __StackLimit
    __StackTop
    __stack (== StackTop)
*/

MEMORY
{
    /* Original flash length: 16384k Bytes.
    FLASH MAP
        0x10000000	(176k)	FLASH_BL	Bootloader
        0x1002C000	(4k)	FLASH_HDR0	Application 0 Header
        0x1002D000	(8096k)	FLASH_APP0	Application 0 Data
        0x10815000	(4k)	FLASH_HDR1	Application 1 Header
        0x10816000	(8096k)	FLASH_APP1	Application 1 Data
        0x10FFE000	(8k)	FLASH_EXTRA	Extra
    */
    FLASH_BL(rx) : ORIGIN = 0x10000000, LENGTH = 176k
    FLASH_HDR0(rx) : ORIGIN = 0x10000000+176k, LENGTH = 4k
    FLASH_APP0(rx) : ORIGIN = 0x10000000+180k, LENGTH = 8096k
    FLASH_HDR1(rx) : ORIGIN = 0x10000000+8276k, LENGTH = 4k
    FLASH_APP1(rx) : ORIGIN = 0x10000000+8280k, LENGTH = 8096k
    FLASH_EXTRA(rx) : ORIGIN = 0x10000000+16376k, LENGTH = 8k
    RAM(rwx) : ORIGIN =  0x20000000, LENGTH = 256k
    SCRATCH_X(rwx) : ORIGIN = 0x20040000, LENGTH = 4k
    SCRATCH_Y(rwx) : ORIGIN = 0x20041000, LENGTH = 4k
}

ENTRY(_entry_point)

SECTIONS
{
    /* Second stage bootloader is prepended to the image. It must be 256 bytes big
       and checksummed. It is usually built by the boot_stage2 target
       in the Raspberry Pi Pico SDK
    */

    .flash_begin : {
        __flash_binary_start = .;
    } > FLASH_BL

    .boot3 : {
        __boot3_start__ = .;
        KEEP (*(.boot3))
        __boot3_end__ = .;
    } > FLASH_BL

    ASSERT(__boot3_end__ - __boot3_start__ <= 176k,
        "ERROR: Bootloader (custom bootloader plus Pico SDK boot2) must be <176k in size")

    /* The second stage will always enter the image at the start of .text.
       The debugger will use the ELF entry point, which is the _entry_point
       symbol if present, otherwise defaults to start of .text.
       This can be used to transfer control back to the bootrom on debugger
       launches only, to perform proper flash setup.
    */

    /* Application 0 Header */
    .hdr0 : {
        __hdr0_start__ = .;
        KEEP (*(.hdr0))
        __hdr0_end__ = .;
    } > FLASH_HDR0

    .text : {
        __logical_binary_start = .;
        KEEP (*(.vectors))
        KEEP (*(.binary_info_header))
        __binary_info_header_end = .;
        KEEP (*(.reset))
        /* TODO revisit this now memset/memcpy/float in ROM */
        /* bit of a hack right now to exclude all floating point and time critical (e.g. memset, memcpy) code from
         * FLASH_APP0 ... we will include any thing excluded here in .data below by default */
        *(.init)
        *(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .text*)
        /* The C++ exception unwinder only runs on a throw: keep it in flash. */
        *libgcc.a:unwind-arm.o(.text*) *libgcc.a:pr-support.o(.text*) *libgcc.a:libunwind.o(.text*)
        *(.fini)
        /* Pull all c'tors into .text */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)
        /* Followed by destructors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.eh_frame*)
        . = ALIGN(4);
    } > FLASH_APP0

    .rodata : {
        *(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .rodata*)
        . = ALIGN(4);
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.flashdata*)))
        . = ALIGN(4);
    } > FLASH_APP0

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH_APP0

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH_APP0
    __exidx_end = .;

    /* Machine inspectable binary information */
    . = ALIGN(4);
    __binary_info_start = .;
    .binary_info :
    {
        KEEP(*(.binary_info.keep.*))
        *(.binary_info.*)
    } > FLASH_APP0
    __binary_info_end = .;
    . = ALIGN(4);

    /* End of .text-like segments */
    __etext = .;

   .ram_vector_table (COPY): {
        *(.ram_vector_table)
    } > RAM

    .data : {
        __data_start__ = .;
        *(vtable)

        *(.time_critical*)

        /* remaining .text and .rodata; i.e. stuff we exclude above because we want it in RAM */
        *(.text*)
        . = ALIGN(4);
        *(.rodata*)
        . = ALIGN(4);

        *(.data*)

        . = ALIGN(4);
        *(.after_data.*)
        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__mutex_array_start = .);
        KEEP(*(SORT(.mutex_array.*)))
        KEEP(*(.mutex_array))
        PROVIDE_HIDDEN (__mutex_array_end = .);

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(SORT(.preinit_array.*)))
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        *(SORT(.fini_array.*))
        *(.fini_array)
        PROVIDE_HIDDEN (__fini_array_end = .);

        *(.jcr)
        . = ALIGN(4);
        /* All data end */
        __data_end__ = .;
    } > RAM AT> FLASH_APP0

    /* NOLOAD takes no flash, so __flash_binary_end is the end of what the .uf2 writes (AT+CONFIG_HASH). */
    .uninitialized_data (NOLOAD): {
        . = ALIGN(4);
        *(.uninitialized_data*)
    } > RAM

    /* Start and end symbols must be word-aligned */
    .scratch_x : {
        __scratch_x_start__ = .;
        *(.scratch_x.*)
        . = ALIGN(4);
        __scratch_x_end__ = .;
    } > SCRATCH_X AT > FLASH_APP0
    __scratch_x_source__ = LOADADDR(.scratch_x);

    .scratch_y : {
        __scratch_y_start__ = .;
        *(.scratch_y.*)
        . = ALIGN(4);
        __scratch_y_end__ = .;
    } > SCRATCH_Y AT > FLASH_APP0
    __scratch_y_source__ = LOADADDR(.scratch_y);

    .bss  : {
        . = ALIGN(4);
        __bss_start__ = .;
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    .heap (COPY):
    {
        __end__ = .;
        end = __end__;
        *(.heap*)
        __HeapLimit = .;
    } > RAM

    /* .stack*_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later
     *
     * stack1 section may be empty/missing if platform_launch_core1 is not used */

    /* by default we put core 0 stack at the end of scratch Y, so that if core 1
     * stack is not used then all of SCRATCH_X is free.
     */
     /*
     * NOTE: This has been modified by John to move Core 0 to scratch X and Core 1 to scratch Y to accommodate heavy
     * stack use on Core 0 (allows it to overflow into Heap instead of Core 1's stack).
     * See this post for more details: https://petewarden.com/2024/01/16/understanding-the-raspberry-pi-picos-memory-layout/
     */
    .stack1_dummy (COPY):
    {
        *(.stack1*)
    } > SCRATCH_Y
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > SCRATCH_X

    .flash_end : {
        __flash_binary_end = .;
    } > FLASH_APP0

    /* stack limit is poorly named, but historically is maximum heap ptr */
    __StackLimit = ORIGIN(RAM) + LENGTH(RAM);
    __StackOneTop = ORIGIN(SCRATCH_Y) + LENGTH(SCRATCH_Y);
    __StackTop = ORIGIN(SCRATCH_X) + LENGTH(SCRATCH_X);
    __StackOneBottom = __StackOneTop - SIZEOF(.stack1_dummy);
    __StackBottom = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed")

    ASSERT( __binary_info_header_end - __logical_binary_start <= 256, "Binary info must be in first 256 bytes of the binary")
    /* todo assert on extra code */
}

