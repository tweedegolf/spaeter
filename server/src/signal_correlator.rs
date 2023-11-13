use std::time::{Duration, SystemTime, UNIX_EPOCH};

use spaeter_core::{AnchorId, Timestamp};
use tokio::sync::Mutex;

use crate::{signal::SignalSummary, signal_locator::SignalLocator};

pub struct SignalCorrelator {
    buffer: Mutex<Vec<(AnchorId, SignalSummary)>>,
    locator: &'static SignalLocator,
}

impl SignalCorrelator {
    pub fn new(locator: &'static SignalLocator) -> Self {
        Self {
            buffer: Mutex::new(Vec::new()),
            locator,
        }
    }

    pub async fn run(&self) -> ! {
        loop {
            let mut buffer = self.buffer.lock().await;

            let current_time = Timestamp::new(
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64,
                0,
            );

            while let Some((_, signal)) = buffer.first() {
                if current_time.difference_to(signal.end_time()) < crate::SIGNAL_CORRELATOR_TIMEOUT
                {
                    break;
                }

                // Search for all similar signals
                let start_time = signal.start_time();
                let end_time = signal.end_time();
                let frequency = signal.average_frequency();
                let magnitude = signal.average_magnitude();

                let correlated_signals = buffer
                    .extract_if(|(_, signal)| {
                        (signal.average_frequency() - frequency).abs() < 50.0
                            && signal.average_magnitude() > magnitude * 0.5
                            && signal.average_magnitude() < magnitude * 1.5
                            && signal.start_time().difference_to(start_time)
                                < Timestamp::from_millis(100)
                            && signal.end_time().difference_to(end_time)
                                < Timestamp::from_millis(100)
                    })
                    .collect::<Vec<_>>();

                if magnitude > 100.0 {
                    log::debug!(
                        "Correlated strong signals @{:7.1}hz, {:7.1} - {}",
                        frequency,
                        magnitude,
                        correlated_signals.len()
                    );
                } else {
                    log::trace!(
                        "Correlated weak signals @{:7.1}hz, {:7.1} - {}",
                        frequency,
                        magnitude,
                        correlated_signals.len()
                    );
                }

                tokio::task::spawn_blocking(|| self.locator.locate_signal(correlated_signals));
            }

            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    }

    pub async fn register_signal(&self, anchor: AnchorId, signal: SignalSummary) {
        if signal.average_magnitude() > 100.0 {
            log::debug!("Registered strong signal:\n{signal:7.1?}");
        } else {
            log::trace!("Registered weak signal:\n{signal:7.1?}");
        }

        self.buffer.lock().await.push((anchor, signal));
    }
}
