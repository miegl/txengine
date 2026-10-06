//! RP1 register map.
//!
//! Sources: "RP1 Peripherals" datasheet (RP-008370-DS-1), the Linux RP1
//! device tree bindings (`include/dt-bindings/mfd/rp1.h`) and the clock /
//! DMA drivers (`clk-rp1.c`, `dw-axi-dmac`). Offsets are relative to the
//! start of PCIe BAR1, which maps RP1 address 0x4000_0000.

/// RP1 peripheral base as seen by bus masters inside RP1 (its DMA engine).
pub const SYS_PERIPH_BASE: u64 = 0xc0_4000_0000;
/// Offset to add to an ARM physical address to reach it from an RP1 bus
/// master (PCIe inbound window, `dma-ranges` of the RP1 node).
pub const HOST_MEM_DMA_OFFSET: u64 = 0x10_0000_0000;
/// Length of the peripheral part of BAR1 we map.
pub const PERIPH_LEN: usize = 0x40_0000;

/// Atomic register aliases (APB peripherals with "atomic access").
pub const ALIAS_SET: usize = 0x2000;
pub const ALIAS_CLR: usize = 0x3000;

// Peripheral blocks.
pub const CLOCKS_MAIN: usize = 0x01_8000;
pub const PWM0: usize = 0x09_8000;
pub const IO_BANK0: usize = 0x0d_0000;
pub const PADS_BANK0: usize = 0x0f_0000;
pub const DMA_TICK: usize = 0x15_8000;
pub const TICKS: usize = 0x17_4000;
pub const DMA: usize = 0x18_8000;
pub const BLOCK_LEN: usize = 0x4000;

/// Clock manager (offsets within CLOCKS_MAIN).
pub mod clk {
    pub const GPCLK_OE_CTRL: usize = 0x00000;
    pub const CLK_DMA_CTRL: usize = 0x00044;

    /// Control register of general purpose clock generator `n` (0..=5).
    pub const fn gp_ctrl(n: usize) -> usize {
        0x174 + 0x10 * n
    }
    pub const fn gp_div_int(n: usize) -> usize {
        gp_ctrl(n) + 0x4
    }
    pub const fn gp_div_frac(n: usize) -> usize {
        gp_ctrl(n) + 0x8
    }
    pub const fn gp_sel(n: usize) -> usize {
        gp_ctrl(n) + 0xc
    }

    pub const CTRL_ENABLE: u32 = 1 << 11;
    /// Read-only status: the generator is running.
    pub const CTRL_ENABLED: u32 = 1 << 28;
    pub const CTRL_AUXSRC_SHIFT: u32 = 5;
    pub const CTRL_AUXSRC_MASK: u32 = 0x1f << CTRL_AUXSRC_SHIFT;
    /// The fractional divider occupies the top 16 bits of DIV_FRAC.
    pub const DIV_FRAC_SHIFT: u32 = 16;

    // PLL blocks (offsets within CLOCKS_MAIN).
    pub const PLL_SYS: usize = 0x08000;
    pub const PLL_AUDIO: usize = 0x0c000;
    pub const PLL_VIDEO: usize = 0x10000;

    pub const PLL_CS: usize = 0x00;
    pub const PLL_PWR: usize = 0x04;
    pub const PLL_FBDIV_INT: usize = 0x08;
    pub const PLL_FBDIV_FRAC: usize = 0x0c;
    pub const PLL_PRIM: usize = 0x10;
    pub const PLL_SEC: usize = 0x14;

    pub const PLL_CS_LOCK: u32 = 1 << 31;
    pub const PLL_CS_REFDIV_MASK: u32 = 0x3f;
    pub const PLL_PWR_PD: u32 = 1 << 0;
    pub const PLL_PWR_DSMPD: u32 = 1 << 2;
    pub const PLL_PWR_POSTDIVPD: u32 = 1 << 3;
    pub const PLL_PWR_VCOPD: u32 = 1 << 5;
    pub const PLL_PRIM_DIV1_SHIFT: u32 = 16;
    pub const PLL_PRIM_DIV1_MASK: u32 = 0x7 << 16;
    pub const PLL_PRIM_DIV2_SHIFT: u32 = 12;
    pub const PLL_PRIM_DIV2_MASK: u32 = 0x7 << 12;
    pub const PLL_FRAC_BITS: u32 = 24;

    // Frequency counters.
    pub const FC0_REF_KHZ: usize = 0x21c;
    pub const FC0_MIN_KHZ: usize = 0x220;
    pub const FC0_MAX_KHZ: usize = 0x224;
    pub const FC0_DELAY: usize = 0x228;
    pub const FC0_INTERVAL: usize = 0x22c;
    pub const FC0_SRC: usize = 0x230;
    pub const FC0_STATUS: usize = 0x234;
    pub const FC0_RESULT: usize = 0x238;
    pub const FC_SIZE: usize = 0x20;
    pub const FC_STATUS_DONE: u32 = 1 << 4;
    pub const FC_STATUS_RUNNING: u32 = 1 << 8;
    pub const FC_RESULT_FRAC_BITS: u32 = 5;
}

/// GPIO bank 0 (offsets within IO_BANK0 / PADS_BANK0).
pub mod gpio {
    pub const fn ctrl(gpio: usize) -> usize {
        gpio * 8 + 4
    }
    pub const fn status(gpio: usize) -> usize {
        gpio * 8
    }
    pub const CTRL_FUNCSEL_MASK: u32 = 0x1f;
    pub const CTRL_OUTOVER_MASK: u32 = 0x3 << 12;
    pub const CTRL_OEOVER_MASK: u32 = 0x3 << 14;

    pub const fn pad(gpio: usize) -> usize {
        0x04 + gpio * 4
    }
    pub const PAD_SLEWFAST: u32 = 1 << 0;
    pub const PAD_DRIVE_SHIFT: u32 = 4;
    pub const PAD_DRIVE_MASK: u32 = 0x3 << 4;
    pub const PAD_OD: u32 = 1 << 7;
}

/// Tick generators (offsets within TICKS / DMA_TICK).
pub mod ticks {
    /// TICKS block: control/cycles registers of DMA tick generator `n` (0 or 1).
    pub const fn dma_ctrl(n: usize) -> usize {
        0x24 + 0xc * n
    }
    pub const fn dma_cycles(n: usize) -> usize {
        0x28 + 0xc * n
    }
    pub const CTRL_ENABLE: u32 = 1 << 0;
    pub const CTRL_RUNNING: u32 = 1 << 1;
    pub const CYCLES_MAX: u32 = 0x1ff;

    /// DMA_TICK block.
    pub const fn tick_en(n: usize) -> usize {
        0x8 * n
    }
    pub const fn tick_ctrl(n: usize) -> usize {
        0x8 * n + 0x4
    }
    pub const EN_REQ: u32 = 1 << 0;
    pub const EN_SINGLE: u32 = 1 << 1;
    pub const CTRL_FINISH_CLEAR: u32 = 1 << 0;
    pub const CTRL_DWELL_SHIFT: u32 = 4;
    pub const CTRL_DWELL_MASK: u32 = 0x1f << 4;
}

/// DMA handshake (DREQ) numbers, `RP1_DMA_*` in the device tree bindings.
pub mod dreq {
    pub const PWM0: u32 = 0x18;
    pub const DMA_TICK0: u32 = 0x30;
    pub const DMA_TICK1: u32 = 0x31;
}

/// Synopsys DesignWare AXI DMAC (offsets within DMA).
pub mod dmac {
    pub const NUM_CHANNELS: usize = 8;

    pub const ID: usize = 0x000;
    pub const COMPVER: usize = 0x008;
    pub const CFG: usize = 0x010;
    pub const CHEN_L: usize = 0x018;
    pub const CHEN_H: usize = 0x01c;
    pub const CFG_DMAC_EN: u32 = 1 << 0;

    pub const fn chan(n: usize) -> usize {
        0x100 + n * 0x100
    }
    // Channel register offsets.
    pub const CH_SAR: usize = 0x00;
    pub const CH_DAR: usize = 0x08;
    pub const CH_BLOCK_TS: usize = 0x10;
    pub const CH_CTL: usize = 0x18;
    pub const CH_CFG: usize = 0x20;
    pub const CH_LLP: usize = 0x28;
    pub const CH_STATUS: usize = 0x30;
    pub const CH_BLK_TFR_RESUMEREQ: usize = 0x48;
    pub const CH_INTSTATUS_ENA: usize = 0x80;
    pub const CH_INTSTATUS: usize = 0x88;
    pub const CH_INTSIGNAL_ENA: usize = 0x90;
    pub const CH_INTCLEAR: usize = 0x98;

    // CHEN (8-channel register map).
    pub const fn chen_en(ch: usize) -> u32 {
        1 << ch
    }
    pub const fn chen_en_we(ch: usize) -> u32 {
        1 << (ch + 8)
    }
    pub const fn chen_susp(ch: usize) -> u32 {
        1 << (ch + 16)
    }
    pub const fn chen_susp_we(ch: usize) -> u32 {
        1 << (ch + 24)
    }
    // CHEN_H holds abort bits.
    pub const fn chen_abort(ch: usize) -> u32 {
        1 << ch
    }
    pub const fn chen_abort_we(ch: usize) -> u32 {
        1 << (ch + 8)
    }

    // CTL (64-bit).
    pub const CTL_SINC_FIXED: u64 = 1 << 4;
    pub const CTL_DINC_FIXED: u64 = 1 << 6;
    pub const CTL_SRC_WIDTH_SHIFT: u32 = 8;
    pub const CTL_DST_WIDTH_SHIFT: u32 = 11;
    pub const CTL_SRC_MSIZE_SHIFT: u32 = 14;
    pub const CTL_DST_MSIZE_SHIFT: u32 = 18;
    pub const CTL_ARLEN_EN: u64 = 1 << 38;
    pub const CTL_ARLEN_SHIFT: u32 = 39;
    pub const CTL_AWLEN_EN: u64 = 1 << 47;
    pub const CTL_AWLEN_SHIFT: u32 = 48;
    pub const CTL_IOC_BLKTFR: u64 = 1 << 58;
    pub const CTL_LLI_LAST: u64 = 1 << 62;
    pub const CTL_LLI_VALID: u64 = 1 << 63;
    pub const WIDTH_32: u64 = 2;
    /// MSIZE encoding: 0 = 1 item, 1 = 4, 2 = 8, 3 = 16 ...
    pub const MSIZE_1: u64 = 0;
    pub const MSIZE_4: u64 = 1;

    // CFG (64-bit), "CFG2" layout used when there are more than 16 handshakes.
    pub const CFG_SRC_MULTBLK_SHIFT: u32 = 0;
    pub const CFG_DST_MULTBLK_SHIFT: u32 = 2;
    pub const CFG_SRC_PER_SHIFT: u32 = 4;
    pub const CFG_DST_PER_SHIFT: u32 = 11;
    pub const CFG_TT_FC_SHIFT: u32 = 32;
    pub const CFG_HS_SEL_SRC: u64 = 1 << 35;
    pub const CFG_HS_SEL_DST: u64 = 1 << 36;
    /// Channel priority (RP1's DMAC keeps it at 51:49 even with the CFG2 layout).
    pub const CFG_PRIORITY_SHIFT: u32 = 49;
    /// Priority bits of both register layouts (CFG: 51:49, CFG2: 54:52).
    pub const CFG_PRIORITY_MASK: u64 = 0x3f << 49;
    pub const MULTBLK_LL: u64 = 3;
    pub const TT_FC_MEM_TO_PER_DMAC: u64 = 1;

    // Interrupt status bits.
    pub const INT_BLOCK_TFR_DONE: u32 = 1 << 0;
    pub const INT_DMA_TFR_DONE: u32 = 1 << 1;
    pub const INT_LLI_INVALID: u32 = 1 << 13;
    pub const INT_ERR_MASK: u32 = (0x3f << 16) | (0x3ff << 5);
}

/// Linked list item of the AXI DMAC (64 bytes, 64-byte aligned).
pub mod lli {
    pub const SIZE: usize = 64;
    pub const SAR: usize = 0x00;
    pub const DAR: usize = 0x08;
    pub const BLOCK_TS: usize = 0x10;
    pub const LLP: usize = 0x18;
    pub const CTL: usize = 0x20;
    pub const STATUS: usize = 0x30;
}
