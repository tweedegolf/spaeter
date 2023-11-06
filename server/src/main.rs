#![feature(extract_if)]

use rumqttc::{AsyncClient, Event, Incoming, MqttOptions, Publish, QoS};
use spaeter_core::TopicData;
use std::{collections::HashMap, error::Error, time::Duration};

use crate::{
    signal_correlator::SignalCorrelator, signal_locator::SignalLocator,
    signal_peak_processor::SignalPeakProcessor,
};

mod signal;
mod signal_correlator;
mod signal_locator;
mod signal_peak_processor;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let config = spaeter_core::config::Config::load(None)?;

    // Set up the signal processing
    let signal_locator = Box::leak(Box::new(SignalLocator::new(HashMap::from_iter(
        config
            .anchors
            .iter()
            .map(|anchor| (anchor.id, anchor.location)),
    ))));
    let signal_correlator = Box::leak(Box::new(SignalCorrelator::new(signal_locator)));
    let signal_peak_processor = Box::leak(Box::new(SignalPeakProcessor::new(signal_correlator)));
    tokio::spawn(signal_correlator.run());
    tokio::spawn(signal_peak_processor.run());

    let mqtt_address = std::env::var("MQTT")
        .expect("Environment variable `MQTT` is required with a host url to an MQTT broker");

    println!("Connecting to the MQTT server at '{}'...", mqtt_address);

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
    println!("Waiting for messages...");

    loop {
        match eventloop.poll().await {
            Ok(Event::Incoming(Incoming::Publish(message))) => {
                tokio::spawn(process_message(message, signal_peak_processor));
            }
            Ok(_x) => {
                // println!("{_x:?}");
            }
            Err(e) => println!("Mqtt connection error: {e}"),
        };
    }
}

async fn process_message(message: Publish, signal_peak_processor: &SignalPeakProcessor) {
    if let Some(anchor_id) = spaeter_core::topics::parse_signal_peak_topic(&message.topic) {
        match spaeter_core::topics::SignalPeakPayload::deserialize(&message.payload) {
            Ok(signal_peak) => {
                // println!("Received signal peak: \n\t- {anchor_id:?}\n\t- {signal_peak:?}");
                signal_peak_processor
                    .register_signal_peak(anchor_id, signal_peak)
                    .await;
            }
            Err(e) => {
                println!("Error deserializing signal peak: {e:?}");
            }
        }
    }
}
