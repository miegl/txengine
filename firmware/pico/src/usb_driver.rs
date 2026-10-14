//! embassy-rp's USB device driver with fixes for two of its bugs, which
//! embassy-rp's main branch fixes too but no release yet (0.10); drop this
//! module when updating to one that does:
//!
//! * `Driver::new` writes zeros over the controller's registers, which on
//!   the RP2350 switches off the controller fixes `LINESTATE_TUNING`
//!   enables at reset (against metastable buffer control reads, receive
//!   hangs after bit-stuffing errors and repeated errors). They are switched
//!   back on.
//! * A control transfer's data or status stage waits until it completes.
//!   One the host has given up (after an error on the bus, or a cancelled
//!   request) never does, and the whole device waits with it, answering
//!   neither requests nor a bus reset, until it is unplugged. A stage now
//!   gives up when the host sends a new SETUP or resets the bus, as the USB
//!   specification has it. And as the controller does not touch EP0's
//!   buffers on a SETUP, the host's next token could take what the given-up
//!   stage armed (a stale reply: the transfers would run one behind until
//!   one timed out), so every SETUP revokes them at once ([`Ep0Guard`]), and
//!   the next stage restores them.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::interrupt::typelevel::{Binding, Handler, USBCTRL_IRQ};
use embassy_rp::pac;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::InterruptHandler;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::driver::{self, EndpointAddress, EndpointAllocError, EndpointError, EndpointType};

type Inner<'d> = embassy_rp::usb::Driver<'d, USB>;

/// The controller can revoke EP0's buffers (`EP_ABORT`: not on the
/// RP2040 B0 and B1, erratum RP2040-E2).
static CAN_ABORT: AtomicBool = AtomicBool::new(false);

pub struct Driver<'d>(Inner<'d>);

impl<'d> Driver<'d> {
    pub fn new(
        usb: Peri<'d, USB>,
        irq: impl Binding<USBCTRL_IRQ, Ep0Guard> + Binding<USBCTRL_IRQ, InterruptHandler<USB>>,
    ) -> Driver<'d> {
        let inner = Inner::new(usb, irq);
        // The reset value (the bus is not connected yet).
        #[cfg(feature = "rp2350")]
        pac::USB.linestate_tuning().write(|w| {
            w.set_dev_buff_control_double_read_fix(true);
            w.set_sie_rx_bitstuff_fix(true);
            w.set_sie_rx_chatter_se0_fix(true);
            w.set_dev_rx_err_quiesce(true);
            w.set_dev_ls_wake_fix(true);
        });
        // Revision 1 is B0 and B1.
        let can_abort = cfg!(feature = "rp2350") || pac::SYSINFO.chip_id().read().revision() >= 2;
        CAN_ABORT.store(can_abort, Ordering::Relaxed);
        Driver(inner)
    }
}

impl<'d> driver::Driver<'d> for Driver<'d> {
    type EndpointOut = <Inner<'d> as driver::Driver<'d>>::EndpointOut;
    type EndpointIn = <Inner<'d> as driver::Driver<'d>>::EndpointIn;
    type ControlPipe = ControlPipe<<Inner<'d> as driver::Driver<'d>>::ControlPipe>;
    type Bus = <Inner<'d> as driver::Driver<'d>>::Bus;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.0.alloc_endpoint_out(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.0.alloc_endpoint_in(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let (bus, pipe) = self.0.start(control_max_packet_size);
        (bus, ControlPipe(pipe))
    }
}

/// Bound to the USB interrupt ahead of embassy-rp's handler: a SETUP or a
/// bus reset revokes EP0's buffers, before the host's next token (the
/// controller NAKs EP0 until a stage of the new transfer restores them).
pub struct Ep0Guard;

impl Handler<USBCTRL_IRQ> for Ep0Guard {
    unsafe fn on_interrupt() {
        let ints = pac::USB.ints().read();
        if (ints.setup_req() || ints.bus_reset()) && CAN_ABORT.load(Ordering::Relaxed) {
            pac::USB.ep_abort().write(|w| {
                w.set_ep_in(0, true);
                w.set_ep_out(0, true);
            });
        }
    }
}

/// embassy-rp's control pipe, whose stages give up when the host has.
pub struct ControlPipe<C>(C);

impl<C: driver::ControlPipe> driver::ControlPipe for ControlPipe<C> {
    fn max_packet_size(&self) -> usize {
        self.0.max_packet_size()
    }

    async fn setup(&mut self) -> [u8; 8] {
        self.0.setup().await
    }

    async fn data_out(&mut self, buf: &mut [u8], first: bool, last: bool) -> Result<usize, EndpointError> {
        stage(self.0.data_out(buf, first, last)).await.unwrap_or(Err(EndpointError::Disabled))
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        stage(self.0.data_in(data, first, last)).await.unwrap_or(Err(EndpointError::Disabled))
    }

    async fn accept(&mut self) {
        stage(self.0.accept()).await;
    }

    async fn reject(&mut self) {
        if begin() {
            self.0.reject().await
        }
    }

    async fn accept_set_address(&mut self, addr: u8) {
        // Given up, the address is not taken.
        stage(self.0.accept_set_address(addr)).await;
    }
}

/// Run a stage, unless the host gives the transfer up first: a new SETUP
/// has come (`setup()` reads it next) or the bus was reset (`Bus::poll`
/// handles that). A stage normally completes within a fraction of a
/// millisecond, so the check rarely runs more than once.
async fn stage<F: Future>(f: F) -> Option<F::Output> {
    if !begin() {
        return None;
    }
    let superseded = async {
        while !superseded() {
            Timer::after_millis(1).await;
        }
    };
    match select(f, superseded).await {
        Either::First(r) => Some(r),
        Either::Second(()) => None,
    }
}

/// Start a stage: false if the host has started over already (EP0 stays
/// revoked for the next transfer); else EP0's buffers are restored (setup()
/// has rewritten them since the guard revoked them), and a later SETUP
/// interrupts.
fn begin() -> bool {
    let usb = pac::USB;
    // embassy-rp's handler masks SETUP_REQ when it fires, and only `setup()`
    // unmasks it: unmask it for the guard.
    let mut setup_req = pac::usb::regs::Int(0);
    setup_req.set_setup_req(true);
    // SAFETY: the atomic bit-set alias of INTE.
    unsafe { (usb.inte().as_ptr() as *mut u8).add(0x2000).cast::<u32>().write_volatile(setup_req.0) };
    // With the guard held off, so that no SETUP comes between the check and
    // the restore unnoticed.
    cortex_m::interrupt::free(|_| {
        if superseded() {
            return false;
        }
        if usb.ep_abort().read().0 != 0 {
            let t0 = Instant::now();
            while {
                let done = usb.ep_abort_done().read();
                !(done.ep_in(0) && done.ep_out(0)) && t0.elapsed() < Duration::from_millis(1)
            } {}
            usb.ep_abort().write_value(pac::usb::regs::EpAbort(0));
            usb.ep_abort_done().write(|w| {
                w.set_ep_in(0, true);
                w.set_ep_out(0, true);
            });
        }
        true
    })
}

/// The host has given the control transfer in progress up.
fn superseded() -> bool {
    let s = pac::USB.sie_status().read();
    s.setup_rec() || s.bus_reset()
}
