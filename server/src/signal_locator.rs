use std::collections::HashMap;

use crate::signal::SignalSummary;
use glam::Vec3;
use spaeter_core::{topics::DetectedLocation, AnchorId};
use tdoa_solver::TdoaEntry;
use tokio::sync::mpsc::Sender;

const SIGNAL_SPEED: f32 = 343.0;

pub struct SignalLocator {
    anchor_locations: HashMap<AnchorId, Vec3>,
    detected_location_sender: Sender<DetectedLocation>,
}

impl SignalLocator {
    pub fn new(
        anchor_locations: HashMap<AnchorId, Vec3>,
        location_sender: Sender<DetectedLocation>,
    ) -> Self {
        Self {
            anchor_locations,
            detected_location_sender: location_sender,
        }
    }

    pub fn locate_signal(&self, mut correlated_signals: Vec<(AnchorId, SignalSummary)>) {
        // Remove all anchors we don't know so we don't crash later on
        correlated_signals.retain(|(id, _)| self.anchor_locations.contains_key(id));

        if correlated_signals.len() < 3 {
            return;
        }

        let earliest_signal = correlated_signals
            .iter()
            .map(|(_, signal)| signal.start_time())
            .min()
            .unwrap();

        let average_magnitude = correlated_signals
            .iter()
            .map(|x| x.1.average_magnitude())
            .sum::<f32>()
            / correlated_signals.len() as f32;

        let average_frequency = correlated_signals
            .iter()
            .map(|x| x.1.average_frequency())
            .sum::<f32>()
            / correlated_signals.len() as f32;

        let mut entries = correlated_signals
            .iter()
            .map(|(anchor, signal)| {
                TdoaEntry::new(
                    self.anchor_locations[anchor],
                    (signal.start_time() - earliest_signal).as_nanos_f64() as f32,
                )
            })
            .collect::<Vec<_>>();

        let mut locations = tdoa_solver::solve(&mut entries, SIGNAL_SPEED, 5);

        locations
            .iter_mut()
            .for_each(|(_, confidence)| *confidence = *confidence * average_magnitude / 1000.0);
        locations.retain(|(_, confidence)| *confidence > 1.0);

        if locations.len() > 0 {
            log::info!(
                "Found locations. time: {earliest_signal:?}, freq: {:7.1}, locations:\n{locations:5.2?}",
                correlated_signals.first().unwrap().1.average_frequency()
            );

            let sender = self.detected_location_sender.clone();
            tokio::spawn(async move {
                for location in locations {
                    sender
                        .send(DetectedLocation {
                            location: location.0,
                            frequency: average_frequency,
                            confidence: location.1,
                            timestamp: earliest_signal,
                        })
                        .await
                        .unwrap();
                }
            });
        }
    }
}
