ENTRY(Reset_Handler)
MEMORY {
    FLASH (rx) : ORIGIN = 0x08004000, LENGTH = 40K
    RAM (rw) : ORIGIN = 0x20000200, LENGTH = 7680
}
_estack = 0x20002000;
SECTIONS {
    .vectors ORIGIN(FLASH) : ALIGN(256) { KEEP(*(.vectors)) } > FLASH
    .devcfg : { KEEP(*(.devcfg)) } > FLASH
    .text : ALIGN(4) { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
    .ARM.extab : ALIGN(4) { *(.ARM.extab*) } > FLASH
    .ARM.exidx : ALIGN(4) { *(.ARM.exidx*) } > FLASH
    .data : ALIGN(4) {
        _sdata = .; *(.data*) . = ALIGN(4); _edata = .;
    } > RAM AT> FLASH
    _sidata = LOADADDR(.data);
    .bss (NOLOAD) : ALIGN(4) {
        _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .;
    } > RAM
    /DISCARD/ : { *(.comment*) *(.note*) *(.ARM.attributes*) }
    ASSERT(SIZEOF(.vectors) == 192, "48 vector entries required")
    ASSERT(ADDR(.devcfg) == 0x080040C0 && SIZEOF(.devcfg) == 32, "FW231 device block must be 32 B at 0x080040C0")
    ASSERT(_ebss <= 0x200017C0, "Reserve retained trace at17C0..17FF and2KiB stack")
    ASSERT(LOADADDR(.data) + SIZEOF(.data) <= 0x0800E000, "Image reaches preserved calibration pages")
}
