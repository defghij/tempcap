#![no_std]
#![no_main]

use avr_hal_generic::delay::Delay;
use panic_halt as _;
use arduino_hal::prelude::*;

use dht11::Dht11;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp,pins,57200);
    let d7 = pins.d7.into_opendrain_high();
    
    let mut dht11 = Dht11::new(d7);
    let mut delay = arduino_hal::Delay::new();

    ufmt::uwrite!(serial, "{}","Temperature,Humidity\n").unwrap();
    arduino_hal::delay_ms(200);

    // TODO: Set up an ISR that triggers every 10 second and prints to screen.


    loop {
        let measurements: Option<(i16, u16)> = match dht11.perform_measurement(&mut delay) {
            Ok(reading) => Some((reading.temperature, reading.humidity)),
            Err(_) => None,
        };
        if measurements.is_some() {
            let t = measurements.unwrap().0;
            let t_tenth = t.rem_euclid(10);
            let t_whole = t / 10;
            let h = measurements.unwrap().1;
            let h_tenth = h.rem_euclid(10);
            let h_whole = h / 10;
            
            ufmt::uwrite!(serial, "{}.{},{}.{}\n", t_whole, t_tenth, h_whole, h_tenth).unwrap();
        }
    
        arduino_hal::delay_ms(2000);
    }
}
