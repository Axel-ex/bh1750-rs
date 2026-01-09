//! # BH1750 driver
//! A platform-agnostic, 'no_std' compatible Rust driver for the BH1750 ambient light sensor using the `embedded-hal` traits.
//!
//! The I²C instruction set is based on the following datasheet: [BH1750 datasheet](https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf) \
//! All instructions are implemented and supported.
//!
//! The raw values read from the sensor are converted to lux, taking into account the resolution mode and measurement time register value.
//!
//! ## Usage
//! To use this driver, import it and an `embedded_hal` implementation, then create an instance of the driver.
//!
//! You can call the `get_one_time_measurement` function to get a single measurement from the sensor.
//!
//! Alternatively, you can call `start_continuous_measurement` to start continuous measurements and then call `get_current_measurement` to get the latest measurement.
//!
//! ## Example
//! This example uses the `esp-hal` crate to interface with the sensor on an ESP32 microcontroller.
//!
//! ```rust no_run
//! #![no_std]
//! #![no_main]
//!
//! use esp_backtrace as _;
//! use bh1750::{BH1750, Resolution};
//! use esp_hal::{
//!     clock::ClockControl,
//!     delay::Delay,
//!     peripherals::Peripherals,
//!     prelude::*,
//!     system::SystemControl,
//!     gpio::Io
//! };
//! use esp_hal::i2c::I2C;
//!
//! #[entry]
//! fn main() -> ! {
//!     esp_println::logger::init_logger_from_env();
//!
//!     let peripherals = Peripherals::take();
//!     let system = SystemControl::new(peripherals.SYSTEM);
//!     let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
//!     let delay = Delay::new(&clocks);
//!
//!     let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//!
//!     let i2c = I2C::new(
//!         peripherals.I2C0,
//!         io.pins.gpio8,
//!         io.pins.gpio9,
//!         100.kHz(),
//!         &clocks,
//!         None
//!     );
//!
//!     let mut bh1750 = BH1750::new(i2c, delay, false);
//!
//!     loop {
//!         let lux = bh1750.get_one_time_measurement(Resolution::High)
//!             .expect("Failed to read BH1750");
//!
//!         log::info!("Lux = {:.2}", lux);
//!
//!         delay.delay(1000.millis());
//!     }
//! }
//! ```

use crate::common::*;
use embedded_hal::delay;
use embedded_hal::i2c::I2c;

pub struct BH1750<I2C, DELAY> {
    com: I2C,
    delay: DELAY,
    address: u8,
    measurement_time_register: u8,
}

impl<I2C: I2c, DELAY: delay::DelayNs> BH1750<I2C, DELAY> {
    /// Create a new instance of the BH1750 driver
    ///
    /// # Arguments
    /// * `i2c` - The I2C bus the sensor is connected to
    /// * `delay` - The delay provider
    /// * `address_pin_high` - The state of the address pin on the sensor (This determines the I2C address)
    pub fn new(i2c: I2C, delay: DELAY, address_pin_high: bool) -> Self {
        Self {
            com: i2c,
            delay,
            address: if address_pin_high {
                STANDARD_ADDRESS_HIGH
            } else {
                STANDARD_ADDRESS_LOW
            },
            measurement_time_register: DEFAULT_MEASUREMENT_TIME_REGISTER,
        }
    }

    /// Create a new instance of the BH1750 driver with a custom I2C address
    /// This is useful if you have a sensor that has been modified to use a different address
    ///
    /// # Arguments
    /// * `i2c` - The I2C bus the sensor is connected to
    /// * `delay` - The delay provider
    /// * `address` - The I2C address of the sensor
    pub fn new_custom_address(i2c: I2C, delay: DELAY, address: u8) -> Self {
        Self {
            com: i2c,
            delay,
            address,
            measurement_time_register: DEFAULT_MEASUREMENT_TIME_REGISTER,
        }
    }

    /// Gets the current measurement from the sensor in lux
    /// The sensor is automatically set to power down mode after the measurement is taken
    ///
    /// This function is blocking and will wait for the measurement to complete
    ///
    /// # Arguments
    /// * `resolution` - The resolution to take the measurement at
    pub fn get_one_time_measurement(
        &mut self,
        resolution: Resolution,
    ) -> Result<f32, BH1750Error<I2C::Error>> {
        self.send_instruction(resolution.one_time_measurement_instruction())?;
        // Waiting 20% longer than the typical measurement time to be safe yields good results
        let safe_delay =
            typical_measurement_time_ms(resolution, self.measurement_time_register) * 12 / 10;
        self.delay.delay_ms(safe_delay);

        self.get_current_measurement(resolution)
    }

    /// Starts continuous measurement at a given resolution
    /// The measurements can be read using `get_current_measurement`
    ///
    /// # Arguments
    /// * `resolution` - The resolution to take the measurements at
    pub fn start_continuous_measurement(
        &mut self,
        resolution: Resolution,
    ) -> Result<(), BH1750Error<I2C::Error>> {
        self.send_instruction(resolution.continuous_measurement_instruction())
    }

    /// Gets the current measurement from the sensor in lux
    /// This function is non-blocking and will return the last measurement taken (possibly 0 if no measurement has been taken yet)
    ///
    /// The resolution argument is required to calculate the lux value from the raw sensor data
    ///
    /// You should call `start_continuous_measurement` before calling this function and wait for the measurement time to pass
    ///
    /// # Arguments
    /// * `resolution` - The resolution the measurement was taken at
    pub fn get_current_measurement(
        &mut self,
        resolution: Resolution,
    ) -> Result<f32, BH1750Error<I2C::Error>> {
        let mut data: [u8; 2] = [0; 2];
        self.com.read(self.address, &mut data)?;
        let raw = (data[0] as u16) << 8 | data[1] as u16;
        Ok(raw_to_lux(raw, resolution, self.measurement_time_register))
    }

    /// Sends the power down instruction to the sensor
    pub fn power_down(&mut self) -> Result<(), BH1750Error<I2C::Error>> {
        self.send_instruction(POWER_DOWN_INSTRUCTION)
    }

    /// Sends the power on instruction to the sensor
    pub fn power_on(&mut self) -> Result<(), BH1750Error<I2C::Error>> {
        self.send_instruction(POWER_ON_INSTRUCTION)
    }

    /// Sends the reset instruction to the sensor
    pub fn reset(&mut self) -> Result<(), BH1750Error<I2C::Error>> {
        self.send_instruction(RESET_INSTRUCTION)
    }

    /// Sets the measurement time register value
    /// This value is used to adjust the measurement time of the sensor
    /// Higher values result in longer measurement times and higher accuracy
    ///
    /// The measurement time register value must be between 31 and 254 (inclusive), default is 69
    ///
    /// # Arguments
    /// * `value` - The value to set the measurement time register to (31-254)
    pub fn set_measurement_time_register(
        &mut self,
        value: u8,
    ) -> Result<(), BH1750Error<I2C::Error>> {
        if !(MIN_MEASUREMENT_TIME_REGISTER..=MAX_MEASUREMENT_TIME_REGISTER).contains(&value) {
            return Err(BH1750Error::MeasurementTimeOutOfRange);
        }

        self.send_instruction(CHANGE_MTREG_HIGH_INSTRUCTION | (value >> 5))?;
        self.send_instruction(CHANGE_MTREG_LOW_INSTRUCTION | (value & 0b0001_1111))?;

        self.measurement_time_register = value;

        Ok(())
    }

    fn send_instruction(&mut self, instr: u8) -> Result<(), BH1750Error<I2C::Error>> {
        self.com.write(self.address, &[instr])?;
        Ok(())
    }
}
