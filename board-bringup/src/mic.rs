use crate::MicInterface;
use stm32f7xx_hal::gpio::PinState;

#[derive(Copy, Clone, Default, Debug)]
pub enum Gain {
    /// GAIN = VDD , gain set to 40dB.
    G40dB,
    /// GAIN = GND, gain set to 50dB.
    G50dB,
    /// GAIN = Unconnected, uncompressed gain set to 60dB.
    #[default]
    G60dB,
}

#[derive(Copy, Clone, Default, Debug)]
pub enum AttackRelease {
    /// A/R = GND: Attack/Release Ratio is 1:500
    Fast,
    /// A/R = VDD : Attack/Release Ratio is 1:2000
    Medium,
    /// A/R = Unconnected: Attack/Release Ratio is 1:4000
    #[default]
    Slow,
}

impl AttackRelease {
    pub fn time(self) -> fugit::MillisDuration<u32> {
        fugit::MillisDuration::<u32>::millis(match self {
            AttackRelease::Fast => 120,
            AttackRelease::Medium => 480,
            AttackRelease::Slow => 960,
        })
    }
}

impl MicInterface<false> {
    pub fn enable(mut self) -> MicInterface<true> {
        self.enable.set_high();

        MicInterface::<true> {
            audio_in: self.audio_in,
            enable: self.enable,
            gain: self.gain,
            ar: self.ar,
        }
    }
}

impl MicInterface<true> {
    pub fn disable(mut self) -> MicInterface<false> {
        self.enable.set_low();

        MicInterface::<false> {
            audio_in: self.audio_in,
            enable: self.enable,
            gain: self.gain,
            ar: self.ar,
        }
    }

    pub fn set_ar(&mut self, ar: AttackRelease) {
        let pin = &mut self.ar;
        match ar {
            AttackRelease::Fast => pin.make_push_pull_output_in_state(PinState::Low),
            AttackRelease::Medium => pin.make_push_pull_output_in_state(PinState::High),
            AttackRelease::Slow => pin.make_floating_input(),
        }
    }

    pub fn set_gain(&mut self, gain: Gain) {
        let pin = &mut self.gain;
        match gain {
            Gain::G40dB => pin.make_push_pull_output_in_state(PinState::High),
            Gain::G50dB => pin.make_push_pull_output_in_state(PinState::Low),
            Gain::G60dB => pin.make_floating_input(),
        }
    }
}
