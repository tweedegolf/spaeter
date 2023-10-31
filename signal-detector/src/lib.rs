#![cfg_attr(not(test), no_std)]

use heapless::Vec;
use microfft::Complex32;

const FFT_OUTPUT_SIZE: usize = 128;
const MAX_TRACKED_PEAKS: usize = 8;

pub struct SignalDetector {
    sample_freq: f32,
}

impl SignalDetector {
    pub const fn new(sample_freq: f32) -> Self {
        Self { sample_freq }
    }

    pub fn feed(&mut self, data: &[f32], mut f: impl FnMut(usize, &[SignalPeak])) {
        const CHUNK_SIZE: usize = 32;
        const SAMPLE_SIZE: usize = FFT_OUTPUT_SIZE * 2;

        for (i, chunk) in data.chunks(CHUNK_SIZE).enumerate() {
            let mut zero_padded_chunk = [0.0; SAMPLE_SIZE];
            zero_padded_chunk[..CHUNK_SIZE].copy_from_slice(chunk);

            let next_signal = microfft::real::rfft_256(&mut zero_padded_chunk);

            let mut peaks = Self::find_peaks(next_signal, self.sample_freq);
            peaks.retain(|peak| !peak.freq.is_nan());
            peaks.sort_unstable_by(|x, y| x.magnitude.total_cmp(&y.magnitude).reverse());

            f(i * CHUNK_SIZE, &peaks);
        }
    }

    fn find_peaks(
        fft_bins: &[Complex32; FFT_OUTPUT_SIZE],
        sample_freq: f32,
    ) -> Vec<SignalPeak, MAX_TRACKED_PEAKS> {
        let peaks = calculate_peaks(fft_bins);

        let mut signal_peaks = Vec::new();

        for peak in peaks {
            let start_index = peak.start.unwrap_or(0);
            let end_index = peak.end.unwrap_or(fft_bins.len());

            let bins_in_peak = &fft_bins[start_index..end_index];
            let (frequency_sum, magnitude_sum) = bins_in_peak.iter().enumerate().fold(
                (0.0, 0.0),
                |(frequency_sum, magnitude_sum), (i, bin)| {
                    let magnitude = bin.norm_sqr();
                    let freq =
                        (start_index + i) as f32 * (sample_freq / fft_bins.len() as f32) / 2.0;

                    (frequency_sum + freq * magnitude, magnitude_sum + magnitude)
                },
            );

            signal_peaks
                .push(SignalPeak {
                    freq: frequency_sum / magnitude_sum,
                    magnitude: magnitude_sum,
                })
                .ok()
                .unwrap();
        }

        signal_peaks
    }
}

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct SignalPeak {
    pub freq: f32,
    pub magnitude: f32,
}

fn calculate_peaks(bins: &[Complex32; FFT_OUTPUT_SIZE]) -> Vec<Peak, MAX_TRACKED_PEAKS> {
    let mut peaks = heapless::Vec::<Peak, MAX_TRACKED_PEAKS>::new();

    let mut current_peak: Option<Peak> = None;

    let mut going_up = true;

    for (index, bin) in bins.iter().enumerate() {
        let bin_magnitude = bin.norm_sqr();

        match &mut current_peak {
            Some(peak) if going_up && bin_magnitude >= peak.top * 0.9 => {
                if bin_magnitude >= peak.top {
                    peak.top = bin_magnitude;
                }
            }
            Some(peak) if going_up && bin_magnitude < peak.top * 0.9 => {
                going_up = false;
            }
            Some(peak) if !going_up && bin_magnitude > bins[index - 1].norm_sqr() => {
                peak.end = Some(index - 1);

                if peaks.is_full() {
                    let min_peak = peaks.iter_mut().min().unwrap();
                    if peak > min_peak {
                        *min_peak = *peak;
                    }
                } else {
                    peaks.push(*peak).ok().unwrap();
                }

                current_peak = None;
                going_up = true;
            }
            Some(_) if !going_up => {}
            None => {
                current_peak = Some(Peak {
                    top: bin_magnitude,
                    start: if index == 0 { None } else { Some(index) },
                    end: None,
                })
            }
            _ => unreachable!(),
        }
    }

    if let Some(peak) = current_peak {
        if peaks.is_full() {
            let min_peak = peaks.iter_mut().min().unwrap();
            if peak > *min_peak {
                *min_peak = peak;
            }
        } else {
            peaks.push(peak).ok().unwrap();
        }
    }

    peaks.sort_unstable();
    peaks
}

#[derive(Clone, Copy)]
struct Peak {
    top: f32,
    start: Option<usize>,
    end: Option<usize>,
}

impl PartialOrd for Peak {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Peak {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.top.total_cmp(&other.top)
    }
}

impl PartialEq for Peak {
    fn eq(&self, other: &Self) -> bool {
        self.top.total_cmp(&other.top).is_eq() && self.start == other.start && self.end == other.end
    }
}

impl Eq for Peak {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signal_change() {
        let mut audio = [0.0; 1024];
        const SAMPLE_FREQUENCY: f32 = 44100.0;
        const FREQUENCY_PER_BIN: f32 = SAMPLE_FREQUENCY / FFT_OUTPUT_SIZE as f32 / 2.0;

        add_signal(
            &mut audio[200..800],
            200,
            4000.0,
            1.0,
            0.0,
            SAMPLE_FREQUENCY,
        );
        add_signal(
            &mut audio[400..600],
            400,
            18000.0,
            0.5,
            3.0,
            SAMPLE_FREQUENCY,
        );
        add_random_noise(&mut audio, 0.1);

        let mut detector = SignalDetector::new(SAMPLE_FREQUENCY);

        let mut signals: std::vec::Vec<Signal> = vec![];

        detector.feed(&audio, |sample_index, peaks| {
            let mut open_signals = signals
                .iter_mut()
                .filter(|signal| signal.end == 0)
                .collect::<std::vec::Vec<_>>();
            let mut new_signals = vec![];

            for peak in peaks {
                if let Some((index, signal)) =
                    open_signals.iter_mut().enumerate().find(|(_, signal)| {
                        peak.freq > signal.average_frequency() - FREQUENCY_PER_BIN
                            && peak.freq < signal.average_frequency() + FREQUENCY_PER_BIN
                    })
                {
                    signal.peaks.push(*peak);
                    open_signals.remove(index);
                } else {
                    new_signals.push(Signal {
                        peaks: vec![*peak],
                        start: sample_index,
                        end: 0,
                    });
                }
            }

            // Everything that is still open has stopped
            for open_signal in open_signals {
                open_signal.end = sample_index;
            }

            signals.append(&mut new_signals);
        });

        signals.sort_by(|a, b| {
            a.average_magnitude()
                .total_cmp(&b.average_magnitude())
                .reverse()
        });

        println!("{:#0.1?}", signals);
    }

    #[derive(Debug)]
    pub struct Signal {
        pub peaks: std::vec::Vec<SignalPeak>,
        pub start: usize,
        pub end: usize,
    }

    impl Signal {
        pub fn average_frequency(&self) -> f32 {
            self.peaks.iter().map(|peak| peak.freq).sum::<f32>() / self.peaks.len() as f32
        }

        pub fn average_magnitude(&self) -> f32 {
            self.peaks.iter().map(|peak| peak.magnitude).sum::<f32>() / self.peaks.len() as f32
        }
    }

    fn add_signal(
        samples: &mut [f32],
        start_sample_index: usize,
        frequency: f32,
        amplitude: f32,
        phase: f32,
        sample_frequency: f32,
    ) {
        samples.iter_mut().enumerate().for_each(|(i, sample)| {
            let i = i + start_sample_index;
            *sample += amplitude
                * (2.0 * std::f32::consts::PI * frequency * (i as f32 / sample_frequency) + phase)
                    .sin();
        });
    }

    fn add_random_noise(samples: &mut [f32], amplitude: f32) {
        samples
            .iter_mut()
            .for_each(|sample| *sample += (rand::random::<f32>() - 0.5) * amplitude * 2.0);
    }
}
