pub const DEFAULT_MEASUREMENT_TIME_REGISTER: u8 = 69;
pub const MIN_MEASUREMENT_TIME_REGISTER: u8 = 31;
pub const MAX_MEASUREMENT_TIME_REGISTER: u8 = 254;

pub const STANDARD_ADDRESS_HIGH: u8 = 0x5c;
pub const STANDARD_ADDRESS_LOW: u8 = 0x23;

pub const POWER_DOWN_INSTRUCTION: u8 = 0b0000_0000;
pub const POWER_ON_INSTRUCTION: u8 = 0b0000_0001;
pub const RESET_INSTRUCTION: u8 = 0b0000_0111;
pub const CHANGE_MTREG_HIGH_INSTRUCTION: u8 = 0b0100_0000; // Last 3 bits are the value
pub const CHANGE_MTREG_LOW_INSTRUCTION: u8 = 0b0110_0000; // Last 5 bits are the value

///// Enum representing the possible errors that can occur when using the BH1750 driver
#[derive(Debug, Copy, Clone)]
pub enum BH1750Error<I2CError> {
    /// The measurement time register value is outside its allowed range
    MeasurementTimeOutOfRange,
    /// I²C error
    I2C(I2CError),
}

impl<I2CError> From<I2CError> for BH1750Error<I2CError> {
    fn from(err: I2CError) -> Self {
        BH1750Error::I2C(err)
    }
}

/// Enum representing the possible resolution modes of the sensor
#[derive(Debug, Copy, Clone)]
pub enum Resolution {
    High,  // 1 lx resolution
    High2, // 0.5 lx resolution
    Low,   // 4 lx resolution
}

impl Resolution {
    pub const fn one_time_measurement_instruction(&self) -> u8 {
        match self {
            Resolution::High => 0b0010_0000,
            Resolution::High2 => 0b0010_0001,
            Resolution::Low => 0b0010_0011,
        }
    }

    pub const fn continuous_measurement_instruction(&self) -> u8 {
        match self {
            Resolution::High => 0b0001_0000,
            Resolution::High2 => 0b0001_0001,
            Resolution::Low => 0b0001_0011,
        }
    }

    pub const fn typical_delay_ms(&self) -> u32 {
        match self {
            Resolution::High => 120,
            Resolution::High2 => 120,
            Resolution::Low => 16,
        }
    }
}

// helpers shared by both implementation
pub fn raw_to_lux(raw: u16, resolution: Resolution, mtreg: u8) -> f32 {
    let mut lux = raw as f32 / 1.2;

    if let Resolution::High2 = resolution {
        lux /= 2.0;
    }

    if mtreg != DEFAULT_MEASUREMENT_TIME_REGISTER {
        lux *= DEFAULT_MEASUREMENT_TIME_REGISTER as f32 / mtreg as f32;
    }

    lux
}

pub fn typical_measurement_time_ms(res: Resolution, mtreg: u8) -> u32 {
    let mut d = res.typical_delay_ms();
    if mtreg != DEFAULT_MEASUREMENT_TIME_REGISTER {
        d = d * mtreg as u32 / DEFAULT_MEASUREMENT_TIME_REGISTER as u32;
    }
    d
}
