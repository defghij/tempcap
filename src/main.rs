#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(cell_update)]

use core::sync::atomic::AtomicBool;


pub mod types {
    use arduino_hal::port::mode::Output;
    use avr_hal_generic::{
        port::mode::{Input, OpenDrain},
        usart::{
            Usart,
            UsartWriter,
            UsartReader,
        }, clock::MHz16
    };
    use dht11::Dht11;
    use panic_halt as _;
    use atmega_hal::{
        port::{
            PD1,
            PD0, PD7
        }, Atmega
    };

    type Device = Atmega;
    type DeviceInterface = atmega_hal::pac::USART0;
    type InputPin = avr_hal_generic::port::Pin<Input, PD0>;
    type OutputPin = avr_hal_generic::port::Pin<Output, PD1>;
    type BaudRate = avr_hal_generic::clock::MHz16;
    pub type SerialConsole = Usart<Device, DeviceInterface, InputPin, OutputPin, BaudRate>;
    pub type SerialReader = UsartReader<Device, DeviceInterface, InputPin, OutputPin, BaudRate>;
    pub type SerialWriter = UsartWriter<Device, DeviceInterface, InputPin, OutputPin, BaudRate>;

    pub type Led = arduino_hal::port::Pin<arduino_hal::port::mode::Output, atmega_hal::port::PB5>;

    #[allow(non_snake_case)]
    pub mod DHT11 {
        use super::{ Dht11, OpenDrain, PD7, MHz16} ;
        pub type InputPin = avr_hal_generic::port::Pin<OpenDrain, PD7>;
        pub type Device = Dht11<InputPin>;
        pub type Delay =  avr_hal_generic::delay::Delay<MHz16>;
    }
}

use avr_device::{atmega328p::{
    TC0,
    TC1
}, interrupt::Mutex};
use avr_hal_generic::prelude::_unwrap_infallible_UnwrapInfallible;
use panic_halt as _;

use core::sync::atomic::Ordering;
use core::mem::MaybeUninit;
use dht11::Dht11;
use types::{
    SerialConsole,
    DHT11
};

#[derive(Clone, Copy)]
pub struct Measurement<T: Clone + Copy>(T,T);
impl<T: Clone + Copy> Measurement<T> {
    pub fn new(natural: T, fractional: T) -> Measurement<T> {
        Measurement(natural, fractional)
    }
}


static SENSOR_UPDATED: AtomicBool = AtomicBool::new(false);
static mut DHT11_SENSOR: MaybeUninit<sensor::Dht11Sensor> = MaybeUninit::uninit();
static mut TIMER0_COUTNER: u16 = 0;
static mut SECONDS: Mutex<core::cell::Cell<u32>> = Mutex::new(core::cell::Cell::new(0));


/// Module that contains all unsafe interactions with the `DHT11_SENSOR` global state.
pub mod sensor {
    use super::{
        DHT11_SENSOR,
        SENSOR_UPDATED,
        Measurement,
        Ordering,
        DHT11,
        Dht11,
    };

    pub struct Dht11Sensor {
        sensor: DHT11::Device,
        delay: DHT11::Delay, 
        measurement: Option<(Measurement<i16>, Measurement<u16>)>,
    } impl Dht11Sensor {
        pub fn new(pin: DHT11::InputPin) -> Dht11Sensor {
            let sensor= Dht11::new(pin);
            let delay = arduino_hal::Delay::new();
            Dht11Sensor { sensor, delay, measurement: None }
        }

        /// Private function to interact poll sensor and get temperature and humidity
        /// data. Data is adjusted then placed in `self.measurement`. This
        /// is the only function to populate measurement with Some(_) 
        /// data.
        fn record_measurement(&mut self) {
            let measurements: Option<(Measurement<i16>, Measurement<u16>)> = match self.sensor.perform_measurement(&mut self.delay) {
                Ok(reading) => {
                    let t = reading.temperature;
                    let h = reading.humidity;
                    let t_whole: i16 = ((t/10)  * 9) / 5 + 32;
                    let t_tenth: i16 = 0; // okay due to accuracy: +/1 C which drops the tens place
                    let h_tenth: u16 = h.rem_euclid(10);
                    let h_whole: u16 = h / 10;
                    let temperature: Measurement<i16> = Measurement(t_whole, t_tenth);
                    let humidity: Measurement<u16> = Measurement(h_whole, h_tenth);

                    Some((temperature, humidity))
                },
                Err(_) => None,
            };
            self.measurement = measurements;
        }

        /// Private function that returns the recorded measurements then
        /// unconditionally replaces it with None. The returned measurement
        /// could be None. This is the _only_ function to retrieve data.
        fn take_measurements(&mut self) -> Option<(Measurement<i16>,Measurement<u16>)> {
            let m = self.measurement;
            self.measurement = None;
            m
        }
    }

    /// Public function that gets a mutable pointer to the global sensor
    /// structure begins the polling process. Toggles the global update
    /// flag to indicate data is valid
    ///
    /// Note this function is only called in the interrupt.
    #[inline(always)]
    pub fn record_measurement() {
        
        // Get a mutable pointer to the sensor.
        let sensor: &mut Dht11Sensor = unsafe {
            &mut *DHT11_SENSOR.as_mut_ptr()
        };
        sensor.record_measurement();

        SENSOR_UPDATED.store(true, Ordering::SeqCst);
    }   

    /// Public function to retrieve current measurement from the sensor
    /// structure. This is accomplished by getting a mutable pointer to
    /// the global sensor state. Note that this function is only called
    /// in the main `loop` structure. Toggles the update flag to indicate
    /// data in sensor structure is now invalid.
    #[inline(always)]
    pub fn take_measurement() -> Option<(Measurement<i16>, Measurement<u16>)> {

        // Get a pointer to the sensor.
        let sensor: &mut Dht11Sensor = unsafe {
            &mut *DHT11_SENSOR.as_mut_ptr()
        };
        SENSOR_UPDATED.store(false, Ordering::SeqCst);
        sensor.take_measurements()
    }

}

///////////////////////////////////////////////////////////////////////////////////
//// Interrupt One: Sensor Polling

#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    avr_device::interrupt::free(|_cs| { sensor::record_measurement(); });
}

///  # TIMER
///  The below sets up a 1/4 Hz, or 1 every 4 seconds, interrupt timer.
///  We use TIMER1 because it has a 16b compare which is needed because
///  the compare value is larger than 2^8.  
///  ---------------------------------------------------------------------
///   interrupt_frequency = 1/4 (Hz) 
///                       = 16_000_000 / (prescaler * cmp_match_reg + 1)
///   then,
///   cmp_match_reg = (16_000_000 / (prescaler * interrupt_frequency)) - 1
///                 = (16_000_000 / (1024 * 1/4) ) - 1
///                 = (16_000_000 / 256 ) - 1
///                 = 62499              // < 2**16
///  ------------------------------------
pub fn setup_sensor_poll_interrupt(timer: TC1) {
    let timer1: TC1 = timer;                                    // Time with 16b compare
    timer1.tccr1a.write(| w: &mut _ | unsafe { w.bits(0) }    ); // Timer/Counter Control Register A 
    timer1.tccr1b.write(| w: &mut _ | w.cs1().prescale_1024() ); // Timer/Counter Control Register B: Clock Select
    timer1.ocr1a.write( | w: &mut _ | w.bits(62499)           ); // Output Compare Register
    timer1.tcnt1.write( | w: &mut _ | w.bits(0)               ); // Timer Counter
    timer1.timsk1.write(| w: &mut _ | w.ocie1a().set_bit()    ); // Enable timer interrupt
}

///////////////////////////////////////////////////////////////////////////////////
//// Interrupt Zero: Time stamp

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        unsafe {
            TIMER0_COUTNER += 1;
            if TIMER0_COUTNER.rem_euclid(1000) == 0 {
                    SECONDS.borrow(cs).update(|x| x + 1);
            }
        }
    });
}

///  # TIMER
///  The below sets up a 1KHz, or 1,000 severy second, interrupt timer.
///  We use TIMER0 because it has a 8b compare which is acceptable because
///  the compare value is smaller than 2^8.  
///  ---------------------------------------------------------------------
///   interrupt_frequency = 1000 (Hz) 
///                       = 16_000_000 / (prescaler * cmp_match_reg + 1)
///   then,
///   cmp_match_reg = (16_000_000 / (prescaler * interrupt_frequency)) - 1
///                 = (16_000_000 / (64 * 1000) ) - 1
///                 = (16_000_000 / 64_000 ) - 1
///                 = 246              // < 2^8
///  ------------------------------------
pub fn setup_timestamp_interrupt(timer: TC0) {
    let timer0: TC0 = timer;                                     // Time with 8b compare
    timer0.tccr0a.write(| w: &mut _ | unsafe { w.bits(0) }    ); // Timer/Counter Control Register A 
    timer0.tccr0b.write(| w: &mut _ | w.cs0().prescale_64()   ); // Timer/Counter Control Register B: Clock Select
    timer0.ocr0a.write( | w: &mut _ | w.bits(249)             ); // Output Compare Register
    timer0.tcnt0.write( | w: &mut _ | w.bits(0)               ); // Timer Counter
    timer0.timsk0.write(| w: &mut _ | w.ocie0a().set_bit()    ); // Enable timer interrupt
}


#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial: SerialConsole  = arduino_hal::default_serial!(dp, pins, 57200);
    let d7: DHT11::InputPin = pins.d7.into_opendrain_high();

    setup_sensor_poll_interrupt(dp.TC1);
    setup_timestamp_interrupt(dp.TC0);

    ufmt::uwrite!(serial, "{}","Seconds,Temperature,Humidity\n").unwrap_infallible();

    // This establishes a compiler enforced happens-before relationship between
    // A = {sensor initialization} and B ={interrupt_enable, sensor read/write access}
    // such that A precedes B.
    unsafe {
        DHT11_SENSOR = MaybeUninit::new(sensor::Dht11Sensor::new(d7));
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        avr_device::interrupt::enable();
    }

    loop {
        if SENSOR_UPDATED.load(Ordering::SeqCst) {

            // Enter an interrupt free critical section to retrieve
            // results from sensor and seconds from the timer.
            let data: Option<(u32, Measurement<i16>,Measurement<u16>)> =  
                avr_device::interrupt::free(|cs| { 
                    match sensor::take_measurement() {
                        None => None,
                        Some(sensor_data) => {
                            let seconds_elapsed = unsafe { SECONDS.borrow(cs).get() };
                            Some((seconds_elapsed, sensor_data.0, sensor_data.1))
                        }
                    }
                });

            if data.is_some() { // We got valid data, send it to console
                let s: u32 = data.unwrap().0;
                let t: Measurement<i16> = data.unwrap().1;
                let h: Measurement<u16> = data.unwrap().2;
                ufmt::uwrite!(serial, "{},{}.{},{}.{}\n", s, t.0, t.1, h.0, h.1).unwrap_infallible();
            }
        }
        avr_device::asm::sleep(); // sleep until an interrupt is triggered.
    }
}
