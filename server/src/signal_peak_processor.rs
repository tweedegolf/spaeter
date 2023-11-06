use std::{
    collections::HashMap,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use signal_detector::SignalPeak;
use spaeter_core::{AnchorId, Timestamp, Timestamped};
use tokio::sync::Mutex;

use crate::{signal::Signal, signal_correlator::SignalCorrelator};

const MAX_SIGNAL_FREQUENCY_DEVIATION: f32 = 400.0; // In hertz (depends on the fft accuracy)

pub struct SignalPeakProcessor {
    signals: Mutex<HashMap<AnchorId, Vec<Signal>>>,
    correlator: &'static SignalCorrelator,
}

impl SignalPeakProcessor {
    pub fn new(correlator: &'static SignalCorrelator) -> Self {
        Self {
            signals: Mutex::new(HashMap::new()),
            correlator,
        }
    }

    pub async fn register_signal_peak(&self, anchor: AnchorId, value: Timestamped<SignalPeak>) {
        let mut signals = self.signals.lock().await;

        let anchor_signals = signals.entry(anchor).or_insert_with(|| Vec::new());

        // Is there an existing signal we can add this peak to?
        let found_signal = anchor_signals.iter_mut().find(|signal| {
            value.timestamp.difference_to(signal.end_time()) < Timestamp::from_millis(200)
                && (signal.average_frequency() - value.value.freq).abs()
                    <= MAX_SIGNAL_FREQUENCY_DEVIATION
        });

        match found_signal {
            Some(found_signal) => found_signal.add_peak(value),
            None => anchor_signals.push(Signal::new(value)),
        }
    }

    pub async fn run(&self) -> ! {
        loop {
            let mut signals = self.signals.lock().await;
            let current_time = Timestamp::new(
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64,
                0,
            );

            for (anchor, signals) in signals.iter_mut() {
                for finished_signal in signals.extract_if(|signal| {
                    signal.end_time().difference_to(current_time) > Timestamp::from_millis(300)
                }) {
                    tokio::spawn(
                        self.correlator
                            .register_signal(*anchor, finished_signal.into()),
                    );
                }
            }

            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    }
}
