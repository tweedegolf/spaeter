use signal_detector::SignalPeak;
use spaeter_core::{Timestamped, Timestamp};

#[derive(Debug)]
pub struct Signal {
    peaks: Vec<Timestamped<SignalPeak>>,
}

impl Signal {
    pub fn new(initial_peak: Timestamped<SignalPeak>) -> Self {
        Self {
            peaks: vec![initial_peak],
        }
    }

    pub fn add_peak(&mut self, value: Timestamped<SignalPeak>) {
        let insert_index = self
            .peaks
            .iter()
            .enumerate()
            .find(|(_, peak)| value.timestamp <= peak.timestamp)
            .map(|(index, _)| index)
            .unwrap_or(self.peaks.len());

        self.peaks.insert(insert_index, value);
    }

    pub fn average_frequency(&self) -> f32 {
        self.peaks.iter().map(|peak| peak.value.freq).sum::<f32>() / self.peaks.len() as f32
    }

    pub fn start_time(&self) -> Timestamp {
        self.peaks.first().unwrap().timestamp
    }

    pub fn end_time(&self) -> Timestamp {
        self.peaks.last().unwrap().timestamp
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SignalSummary {
    start: Timestamp,
    end: Timestamp,
    frequency: f32,
}

impl SignalSummary {
    pub fn average_frequency(&self) -> f32 {
        self.frequency
    }

    pub fn start_time(&self) -> Timestamp {
        self.start
    }

    pub fn end_time(&self) -> Timestamp {
        self.end
    }
}

impl From<Signal> for SignalSummary {
    fn from(value: Signal) -> Self {
        Self {
            start: value.start_time(),
            end: value.end_time(),
            frequency: value.average_frequency(),
        }
    }
}
