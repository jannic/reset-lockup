//! # Multicore Blinking Example
//!
//! This application blinks two LEDs on GPIOs 2 and 3 at different rates (3Hz
//! and 4Hz respectively.)
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]
#![allow(unused)]

use hal::clocks::Clock;
use hal::gpio::Pins;
use hal::multicore::{Multicore, Stack};
use hal::sio::Sio;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use embedded_hal::digital::{OutputPin, StatefulOutputPin};

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;
// pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static
/// values is reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything separately
/// and modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte
/// alignment, which allows the stack guard to take up the least amount of
/// usable RAM.
static mut CORE1_STACK: Stack<4096> = Stack::new();

#[inline(never)]
// #[link_section = ".data"]
fn delay() {
    unsafe {
        // This works for blink patterns when core runs from ROSC at ~6MHz
        // let real_cycles = 500000;

        // This works for blink patterns when core runs from from XOSC at 125MHz
        // let real_cycles = 10000000;

        // Use this to view the signal on a logic analyzer instead
        let real_cycles = 10;

        core::arch::asm!(
            // Use local labels to avoid R_ARM_THM_JUMP8 relocations which fail on thumbv6m.
            "1:",
            "subs {}, #1",
            "bne 1b",
            inout(reg) real_cycles => _,
            options(nomem, nostack),
        )
    }
}

fn blink(led: &mut impl OutputPin, count: u8) {
    for _ in 0..count {
        led.set_high().unwrap();
        delay();
        led.set_low().unwrap();
        delay();
    }
    delay();
}

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[rp2040_hal::entry]
fn main() -> ! {
    // unsafe {
        // (*pac::SIO::PTR).gpio_out_clr().write(|w| w.bits(1 << 25));
        // (*pac::PSM::PTR)
        //     .frce_off()
        //     .modify(|_, w| w.proc1().set_bit());
        // cortex_m::asm::delay(1000);
        // (*pac::PSM::PTR)
        //     .frce_off()
        //     .modify(|_, w| w.proc1().clear_bit());
    // }

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    // let core = pac::CorePeripherals::take().unwrap();

    // Set up the GPIO pins
    let mut sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led = pins.gpio25.into_push_pull_output();

    // pac.BUSCTRL.bus_priority().write(|w| w.proc0().set_bit());

    blink(&mut led, 1);

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    // let clocks = hal::clocks::init_clocks_and_plls(
    //     XTAL_FREQ_HZ,
    //     pac.XOSC,
    //     pac.CLOCKS,
    //     pac.PLL_SYS,
    //     pac.PLL_USB,
    //     &mut pac.RESETS,
    //     &mut watchdog,
    // )
    // .unwrap();

    blink(&mut led, 2);

    // Start up the second core to blink the second LED
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    blink(&mut led, 3);
    core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        main1();
    }).unwrap();
    blink(&mut led, 4);


    // unsafe {
    //     (*pac::PSM::PTR)
    //         .frce_off()
    //         .modify(|_, w| w.proc1().set_bit());
    //     }

    // hal::reset();
    cortex_m::peripheral::SCB::sys_reset();
}

#[inline(never)]
// #[link_section = ".data"]
fn main1() -> ! {
    // Get the second core's copy of the `CorePeripherals`, which are per-core.
    // Unfortunately, `cortex-m` doesn't support this properly right now,
    // so we have to use `steal`.
    // let core = unsafe { pac::CorePeripherals::steal() };
    // Set up the delay for the second core.
    // let mut delay = Delay::new(core.SYST, sys_freq);
    // Blink the second LED.
    loop {
        // cortex_m::asm::wfe();
        // cortex_m::asm::bkpt();
        // delay.delay_us(CORE1_DELAY)
        // cortex_m::asm::delay(10000);
        // let real_cycles = 5000;
        // unsafe {
        //     core::arch::asm!(
        //         // Use local labels to avoid R_ARM_THM_JUMP8 relocations which fail on thumbv6m.
        //         "1:",
        //         "subs {}, #1",
        //         "bne 1b",
        //         inout(reg) real_cycles => _,
        //         options(nomem, nostack),
        //     )
        // }
        delay();
    }
}

// End of file
