//! BCM2835/6/7 and BCM2711 register map (legacy peripherals).
//!
//! Sources: "BCM2835 ARM Peripherals", "BCM2711 ARM Peripherals", and the
//! Linux `clk-bcm2835`, `bcm2835-dma` drivers for the (undocumented) PLL
//! and A2W registers.

/// Bus address of the peripheral window as seen by DMA masters.
pub const BUS_PERIPH_BASE: u32 = 0x7e00_0000;

// Block offsets from the peripheral base.
pub const DMA: usize = 0x00_7000;
pub const PADS: usize = 0x10_0000;
pub const CM: usize = 0x10_1000;
pub const CM_LEN: usize = 0x2000;
pub const GPIO: usize = 0x20_0000;
pub const PCM: usize = 0x20_3000;
pub const PWM: usize = 0x20_c000;

/// Clock manager.
pub mod cm {
    pub const PASSWORD: u32 = 0x5a00_0000;

    pub const VPU: usize = 0x008;
    pub const H264: usize = 0x028;
    pub const ISP: usize = 0x030;
    pub const V3D: usize = 0x038;
    pub const CAM0: usize = 0x040;
    pub const CAM1: usize = 0x048;
    pub const CCP2: usize = 0x050;
    pub const DSI0E: usize = 0x058;
    pub const DPI: usize = 0x068;
    pub const GP0: usize = 0x070;
    pub const HSM: usize = 0x088;
    pub const PCM: usize = 0x098;
    pub const PWM: usize = 0x0a0;
    pub const SLIM: usize = 0x0a8;
    pub const SMI: usize = 0x0b0;
    pub const UART: usize = 0x0f0;
    pub const VEC: usize = 0x0f8;
    pub const DSI1E: usize = 0x158;
    pub const DFT: usize = 0x168;
    pub const SDRAM: usize = 0x1a8;
    pub const AVEO: usize = 0x1b8;
    pub const EMMC: usize = 0x1c0;
    pub const EMMC2: usize = 0x1d0;

    pub const fn gp(n: usize) -> usize {
        GP0 + 8 * n
    }
    /// Divider register of the clock whose control register is at `ctl`.
    pub const fn div(ctl: usize) -> usize {
        ctl + 4
    }

    // CTL bits.
    pub const SRC_MASK: u32 = 0xf;
    pub const ENAB: u32 = 1 << 4;
    pub const KILL: u32 = 1 << 5;
    pub const BUSY: u32 = 1 << 7;
    pub const MASH_SHIFT: u32 = 9;
    pub const MASH_MASK: u32 = 0x3 << 9;
    // DIV fields.
    pub const DIVI_SHIFT: u32 = 12;

    // Clock sources of the "peripheral" mux.
    pub const SRC_OSC: u32 = 1;
    pub const SRC_PLLA_PER: u32 = 4;
    pub const SRC_PLLC_PER: u32 = 5;
    pub const SRC_PLLD_PER: u32 = 6;
    // Additional sources of the VPU-type mux.
    pub const SRC_PLLC_CORE1: u32 = 8;
    pub const SRC_PLLC_CORE2: u32 = 9;

    // Frequency counter.
    pub const OSCCOUNT: usize = 0x100;
    pub const TCNTCTL: usize = 0x0c0;
    pub const TCNTCNT: usize = 0x0c4;
    pub const TCNT_SRC1_SHIFT: u32 = 12;
    pub const TCNT_GP0: u32 = 20;
    pub const TCNT_PWM: u32 = 24;

    // PLLs.
    pub const PLLA: usize = 0x104;
    pub const PLLC: usize = 0x108;
    pub const PLLD: usize = 0x10c;
    pub const LOCK: usize = 0x114;
    pub const PLL_ANARST: u32 = 1 << 8;
    pub const PLL_HOLDPER: u32 = 1 << 7;
    pub const PLL_LOADPER: u32 = 1 << 6;

    pub const A2W_PLLA_ANA0: usize = 0x1010;
    pub const A2W_PLLC_ANA0: usize = 0x1030;
    pub const A2W_PLLD_ANA0: usize = 0x1050;
    pub const A2W_XOSC_CTRL: usize = 0x1190;
    pub const A2W_PLLA_CTRL: usize = 0x1100;
    pub const A2W_PLLC_CTRL: usize = 0x1120;
    pub const A2W_PLLD_CTRL: usize = 0x1140;
    pub const A2W_PLLA_FRAC: usize = 0x1200;
    pub const A2W_PLLC_FRAC: usize = 0x1220;
    pub const A2W_PLLD_FRAC: usize = 0x1240;
    pub const A2W_PLLA_PER: usize = 0x1500;
    pub const A2W_PLLC_PER: usize = 0x1520;
    pub const A2W_PLLD_PER: usize = 0x1540;
    // Other PLL channels (for consumer detection).
    pub const A2W_PLLA_DSI0: usize = 0x1300;
    pub const A2W_PLLA_CORE: usize = 0x1400;
    pub const A2W_PLLA_CCP2: usize = 0x1600;
    pub const A2W_PLLC_CORE2: usize = 0x1320;
    pub const A2W_PLLC_CORE1: usize = 0x1420;
    pub const A2W_PLLC_CORE0: usize = 0x1620;
    pub const A2W_PLLD_DSI0: usize = 0x1340;
    pub const A2W_PLLD_CORE: usize = 0x1440;
    pub const A2W_PLLD_DSI1: usize = 0x1640;

    pub const A2W_CTRL_NDIV_MASK: u32 = 0x3ff;
    pub const A2W_CTRL_PDIV_SHIFT: u32 = 12;
    pub const A2W_CTRL_PDIV_MASK: u32 = 0x7 << 12;
    pub const A2W_CTRL_PWRDN: u32 = 1 << 16;
    pub const A2W_CTRL_PRST_DISABLE: u32 = 1 << 17;
    pub const A2W_FRAC_BITS: u32 = 20;
    pub const A2W_FRAC_MASK: u32 = (1 << 20) - 1;
    pub const A2W_CHANNEL_DISABLE: u32 = 1 << 8;
    pub const A2W_CHANNEL_DIV_MASK: u32 = 0xff;
    /// ANA1 bit 14: feedback pre-divider (BCM2835/6/7 only).
    pub const ANA1_FB_PREDIV: u32 = 1 << 14;
    /// Loop filter gains (field names as in Linux): KI and KP in ANA1, KA
    /// in ANA3. Linux and the firmware use KI 2, KP 8, KA 2.
    pub const ANA1_KI_SHIFT: u32 = 19;
    pub const ANA1_KI_MASK: u32 = 0x7 << 19;
    pub const ANA1_KP_SHIFT: u32 = 15;
    pub const ANA1_KP_MASK: u32 = 0xf << 15;
    pub const ANA3_KA_SHIFT: u32 = 7;
    pub const ANA3_KA_MASK: u32 = 0x7 << 7;
}

/// Pad control (drive strength) of the GPIO banks.
pub mod pads {
    pub const GPIO_0_27: usize = 0x2c;
    pub const GPIO_28_45: usize = 0x30;
    pub const PASSWORD: u32 = 0x5a00_0000;
    pub const SLEW_UNLIMITED: u32 = 1 << 4;
    pub const HYST: u32 = 1 << 3;
    pub const DRIVE_MASK: u32 = 0x7;
}

/// PWM controller.
pub mod pwm {
    pub const CTL: usize = 0x00;
    pub const STA: usize = 0x04;
    pub const DMAC: usize = 0x08;
    pub const RNG1: usize = 0x10;
    pub const FIF1: usize = 0x18;
    pub const CTL_PWEN1: u32 = 1 << 0;
    pub const CTL_MODE1: u32 = 1 << 1;
    pub const CTL_USEF1: u32 = 1 << 5;
    pub const CTL_CLRF1: u32 = 1 << 6;
    pub const CTL_MSEN1: u32 = 1 << 7;
    /// The control bits of channel 2 are channel 1's shifted by this.
    pub const CTL_CHAN2_SHIFT: u32 = 8;
    pub const DMAC_ENAB: u32 = 1 << 31;
    pub const DMAC_PANIC_SHIFT: u32 = 8;
    pub const STA_EMPT1: u32 = 1 << 1;
    pub const STA_BERR: u32 = 1 << 8;
    pub const STA_STA1: u32 = 1 << 9;
    pub const STA_ERRORS: u32 = 0x1fc;
    /// Words in the FIFO, shared by both channels.
    pub const FIFO_LEN: u32 = 16;
    pub const DREQ: u32 = 5;

    /// Range register of channel `chan` (0 or 1).
    pub const fn rng(chan: usize) -> usize {
        RNG1 + 0x10 * chan
    }
}

/// PCM/I2S controller.
pub mod pcm {
    pub const CS: usize = 0x00;
    pub const FIFO: usize = 0x04;
    pub const MODE: usize = 0x08;
    pub const TXC: usize = 0x10;
    pub const DREQ_A: usize = 0x14;
    pub const CS_EN: u32 = 1 << 0;
    pub const CS_TXON: u32 = 1 << 2;
    pub const CS_TXCLR: u32 = 1 << 3;
    pub const CS_DMAEN: u32 = 1 << 9;
    pub const CS_TXERR: u32 = 1 << 15;
    pub const CS_SYNC: u32 = 1 << 24;
    pub const MODE_FLEN_SHIFT: u32 = 10;
    pub const MODE_FSLEN_SHIFT: u32 = 0;
    pub const TXC_CH1EN: u32 = 1 << 30;
    pub const DREQ: u32 = 2;
}

/// Legacy DMA engine.
pub mod dma {
    pub const CHANNEL_SIZE: usize = 0x100;
    pub const CS: usize = 0x00;
    pub const CONBLK_AD: usize = 0x04;
    pub const TI: usize = 0x08;
    pub const SOURCE_AD: usize = 0x0c;
    pub const DEBUG: usize = 0x20;
    pub const INT_STATUS: usize = 0xfe0;
    pub const ENABLE: usize = 0xff0;

    pub const CS_ACTIVE: u32 = 1 << 0;
    pub const CS_END: u32 = 1 << 1;
    pub const CS_INT: u32 = 1 << 2;
    pub const CS_PAUSED: u32 = 1 << 4;
    pub const CS_ERROR: u32 = 1 << 8;
    pub const CS_PRIORITY_SHIFT: u32 = 16;
    pub const CS_PANIC_PRIORITY_SHIFT: u32 = 20;
    pub const CS_WAIT_FOR_OUTSTANDING_WRITES: u32 = 1 << 28;
    pub const CS_DISDEBUG: u32 = 1 << 29;
    pub const CS_ABORT: u32 = 1 << 30;
    pub const CS_RESET: u32 = 1 << 31;

    pub const TI_WAIT_RESP: u32 = 1 << 3;
    pub const TI_DEST_DREQ: u32 = 1 << 6;
    pub const TI_SRC_INC: u32 = 1 << 8;
    /// Burst length (BURST_LENGTH; 0: single transfers).
    pub const TI_BURST_LENGTH_SHIFT: u32 = 12;
    pub const TI_PERMAP_SHIFT: u32 = 16;
    pub const TI_NO_WIDE_BURSTS: u32 = 1 << 26;

    pub const DEBUG_ERRORS: u32 = 0x7;
    /// Control block size and alignment.
    pub const CB_SIZE: usize = 32;
    /// Byte offset of the `next` field in a control block.
    pub const CB_NEXT: usize = 20;
}

/// GPIO function select.
pub mod gpio {
    pub const fn fsel(pin: u32) -> usize {
        (pin as usize / 10) * 4
    }
    pub const fn fsel_shift(pin: u32) -> u32 {
        (pin % 10) * 3
    }
    pub const ALT0: u32 = 0b100;
    pub const ALT5: u32 = 0b010;
}
