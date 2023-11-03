use std::collections::HashMap;

use crate::signal::SignalSummary;
use glam::Vec3;
use spaeter_core::AnchorId;
use tdoa_solver::TdoaEntry;

const SIGNAL_SPEED: f32 = 343.0;

pub struct SignalLocator {
    anchor_locations: HashMap<AnchorId, Vec3>,
}

impl SignalLocator {
    pub fn new(anchor_locations: HashMap<AnchorId, Vec3>) -> Self {
        Self { anchor_locations }
    }

    pub fn locate_signal(&self, correlated_signals: Vec<(AnchorId, SignalSummary)>) {
        let Some(earliest_signal) = correlated_signals
            .iter()
            .map(|(_, signal)| signal.start_time())
            .min()
        else {
            return;
        };

        let mut entries = correlated_signals
            .iter()
            .map(|(anchor, signal)| {
                TdoaEntry::new(
                    self.anchor_locations[anchor],
                    (signal.start_time() - earliest_signal).as_nanos_f64() as f32,
                )
            })
            .collect::<Vec<_>>();

        let locations = tdoa_solver::solve(&mut entries, SIGNAL_SPEED, 5);

        println!(
            "Found locations. time: {earliest_signal:?}, freq: {}, locations: {locations:?}",
            correlated_signals.first().unwrap().1.average_frequency()
        );
    }
}
