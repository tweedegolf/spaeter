#![cfg_attr(not(test), no_std)]

use core::ops::{Add, AddAssign, Sub, SubAssign};

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct AnchorId(u32);

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub enum AnchorMessage {
    SignalPeak {
        timestamp: Timestamp,
        peak: signal_detector::SignalPeak,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Timestamp {
    nanos: u64,
    subnanos: u32,
}

impl Timestamp {
    pub const fn new(nanos: u64, subnanos: u32) -> Self {
        Self { nanos, subnanos }
    }

    pub const fn from_micros(value: u64) -> Self {
        Self::new(value * 1000, 0)
    }

    pub const fn from_millis(value: u64) -> Self {
        Self::new(value * 1_000_000, 0)
    }

    pub const fn from_secs(value: u64) -> Self {
        Self::new(value * 1_000_000_000, 0)
    }

    pub fn from_nanos_f64(nanos: f64) -> Self {
        if nanos.is_nan() {
            panic!("Cannot construct timestamp from NAN");
        }
        if nanos < 0.0 {
            panic!("Cannot construct negative timestamp");
        }

        let nanos_fract = nanos - (nanos as u64) as f64;

        Self::new(nanos as u64, (nanos_fract * (u32::MAX as f64 + 1.0)) as u32)
    }

    pub fn as_nanos_f64(&self) -> f64 {
        self.nanos as f64 + (self.subnanos as f64 / (u32::MAX as f64 + 1.0))
    }

    pub const fn as_nanos(&self) -> u64 {
        self.nanos
    }

    pub const fn as_micros(&self) -> u64 {
        self.nanos / 1000
    }

    pub const fn as_millis(&self) -> u64 {
        self.nanos / 1_000_000
    }

    pub const fn as_secs(&self) -> u64 {
        self.nanos / 1_000_000_000
    }
}

impl PartialOrd for Timestamp {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(&other))
    }
}

impl Ord for Timestamp {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        match self.nanos.cmp(&other.nanos) {
            core::cmp::Ordering::Equal => {}
            ord => return ord,
        }
        self.subnanos.cmp(&other.subnanos)
    }
}

impl Add for Timestamp {
    type Output = Timestamp;

    fn add(self, rhs: Self) -> Self::Output {
        let (new_subnanos, overflowed) = self.subnanos.overflowing_add(rhs.subnanos);

        Self {
            nanos: self.nanos + rhs.nanos + if overflowed { 1 } else { 0 },
            subnanos: new_subnanos,
        }
    }
}

impl AddAssign for Timestamp {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Sub for Timestamp {
    type Output = Timestamp;

    fn sub(self, rhs: Self) -> Self::Output {
        let (new_subnanos, overflowed) = self.subnanos.overflowing_sub(rhs.subnanos);

        Self {
            nanos: self
                .nanos
                .checked_sub(rhs.nanos)
                .expect("Timestamp underflow")
                .checked_sub(if overflowed { 1 } else { 0 })
                .expect("Timestamp underflow"),
            subnanos: new_subnanos,
        }
    }
}

impl SubAssign for Timestamp {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}
