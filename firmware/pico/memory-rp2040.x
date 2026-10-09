/* Raspberry Pi Pico (RP2040, 2 MB flash). */
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}

SECTIONS {
    /* picotool looks for the binary info header in the first 512 bytes. */
    .boot_info : ALIGN(4)
    {
        KEEP(*(.boot_info));
    } > FLASH
} INSERT AFTER .vector_table;

/* .text starts after the binary info header. */
_stext = ADDR(.boot_info) + SIZEOF(.boot_info);

SECTIONS {
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;
