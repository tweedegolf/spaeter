#![feature(extract_if)]

use rumqttc::{AsyncClient, Event, Incoming, MqttOptions, Publish, QoS};
use spaeter_core::{topics::DETECTED_LOCATION_TOPIC, Timestamp, TopicData};
use std::{collections::HashMap, error::Error, time::Duration};

use crate::{
    signal_correlator::SignalCorrelator, signal_locator::SignalLocator,
    signal_peak_processor::SignalPeakProcessor,
};

mod signal;
mod signal_correlator;
mod signal_locator;
mod signal_peak_processor;

const PEAK_TO_SIGNAL_TIMEOUT: Timestamp = Timestamp::from_millis(500);
const SIGNAL_CORRELATOR_TIMEOUT: Timestamp = Timestamp::from_millis(800);

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    pretty_env_logger::init();

    let config = spaeter_core::config::Config::load(None)?;

    let (detected_locations_sender, mut detected_locations_receiver) =
        tokio::sync::mpsc::channel(128);

    // Set up the signal processing
    let signal_locator = Box::leak(Box::new(SignalLocator::new(
        HashMap::from_iter(
            config
                .anchors
                .iter()
                .map(|anchor| (anchor.id, anchor.location)),
        ),
        detected_locations_sender,
    )));
    let signal_correlator = Box::leak(Box::new(SignalCorrelator::new(signal_locator)));
    let signal_peak_processor = Box::leak(Box::new(SignalPeakProcessor::new(signal_correlator)));
    tokio::spawn(signal_correlator.run());
    tokio::spawn(signal_peak_processor.run());

    let mqtt_address = std::env::var("MQTT")
        .expect("Environment variable `MQTT` is required with a host url to an MQTT broker");

    log::info!("Connecting to the MQTT server at '{}'...", mqtt_address);

    let mut mqttoptions = MqttOptions::parse_url(mqtt_address)?;
    mqttoptions.set_keep_alive(Duration::from_secs(5));

    let (client, mut eventloop) = AsyncClient::new(mqttoptions, 16);
    client
        .subscribe(
            spaeter_core::topics::signal_peak_topic(None).as_str(),
            QoS::ExactlyOnce,
        )
        .await
        .unwrap();

    // Just loop on incoming messages.
    log::info!("Waiting for messages...");

    tokio::spawn(async move {
        let mut buffer = [0; 1024];
        loop {
            let detected_location = detected_locations_receiver.recv().await.unwrap();
            let len = detected_location.serialize(&mut buffer).unwrap();
            if let Err(e) = client
                .publish(
                    DETECTED_LOCATION_TOPIC,
                    QoS::ExactlyOnce,
                    false,
                    &buffer[..len],
                )
                .await
            {
                log::error!("Could not publish the detected location: {e}");
            }
        }
    });

    loop {
        match eventloop.poll().await {
            Ok(Event::Incoming(Incoming::Publish(message))) => {
                tokio::spawn(process_message(message, signal_peak_processor));
            }
            Ok(x) => {
                log::trace!("{x:?}");
            }
            Err(e) => log::error!("Mqtt connection error: {e}"),
        };
    }
}

async fn process_message(message: Publish, signal_peak_processor: &SignalPeakProcessor) {
    if let Some(anchor_id) = spaeter_core::topics::parse_signal_peak_topic(&message.topic) {
        match spaeter_core::topics::SignalPeakPayload::deserialize(&message.payload) {
            Ok(signal_peak) => {
                log::trace!("Received signal peak: \n\t- {anchor_id:?}\n\t- {signal_peak:?}");
                signal_peak_processor
                    .register_signal_peak(anchor_id, signal_peak)
                    .await;
            }
            Err(e) => {
                log::error!("Error deserializing signal peak: {e:?}");
            }
        }
    }
}
