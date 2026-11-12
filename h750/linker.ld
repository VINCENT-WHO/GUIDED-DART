ENTRY(Reset_Handler)
MEMORY { FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
         RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
         AXIRAM (xrw) : ORIGIN = 0x24000000, LENGTH = 512K
         D2RAM (xrw) : ORIGIN = 0x30000000, LENGTH = 288K }
_estack = ORIGIN(RAM) + LENGTH(RAM) - 32;
SECTIONS {
 .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) } > FLASH
 .text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); } > FLASH
 .ARM.extab : { *(.ARM.extab*) } > FLASH
 .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
 .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH
 .init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH
 .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH
 _sidata = LOADADDR(.data);
 .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
 .bss (NOLOAD) : { . = ALIGN(32); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; PROVIDE(end = .); PROVIDE(_end = .); } > RAM
 .heap_stack (NOLOAD) : { . = ALIGN(8); . += 0x400; . += 0x2000; } > RAM
 .camera_buffer (NOLOAD) : { . = ALIGN(32); *(.camera_buffer*) . = ALIGN(32); } > AXIRAM
 .camera_d2 (NOLOAD) : { . = ALIGN(32); *(.camera_d2*) . = ALIGN(32); } > D2RAM
 /DISCARD/ : { *(.note.GNU-stack) }
}
