#![feature(extract_if)]

use futures_util::StreamExt;
use glam::Vec3;
use paho_mqtt::MQTT_VERSION_5;
use spaeter_core::{AnchorId, TopicData};
use std::{error::Error, process, time::Duration};

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
    // Set up the signal processing
    let signal_locator = Box::leak(Box::new(SignalLocator::new(
        [
            (AnchorId(0), Vec3::new(0.0, 0.0, 0.0)),
            (AnchorId(1), Vec3::new(10.0, 0.0, 0.0)),
            (AnchorId(2), Vec3::new(0.0, 10.0, 0.0)),
            (AnchorId(3), Vec3::new(10.0, 10.0, 10.0)),
        ]
        .into(),
    )));
    let signal_correlator = Box::leak(Box::new(SignalCorrelator::new(signal_locator)));
    let signal_peak_processor = Box::leak(Box::new(SignalPeakProcessor::new(signal_correlator)));
    tokio::spawn(signal_correlator.run());
    tokio::spawn(signal_peak_processor.run());

    let mqtt_address = std::env::var("MQTT")
        .expect("Environment variable `MQTT` is required with a host url to an MQTT broker");

    println!("Connecting to the MQTT server at '{}'...", mqtt_address);

    // Create the client. Use an ID for a persistent session.
    let create_opts = paho_mqtt::CreateOptionsBuilder::new()
        .server_uri(mqtt_address)
        .client_id("spaeter-server")
        .finalize();

    // Create the client connection
    let mut cli = paho_mqtt::AsyncClient::new(create_opts).unwrap_or_else(|e| {
        println!("Error creating the client: {:?}", e);
        process::exit(1);
    });

    // Get message stream before connecting.
    let mut strm = cli.get_stream(25);

    // Connect with MQTT v5 and a persistent server session (no clean start).
    // For a persistent v5 session, we must set the Session Expiry Interval
    // on the server. Here we set that requests will persist for an hour
    // (3600sec) if the service disconnects or restarts.
    let conn_opts = paho_mqtt::ConnectOptionsBuilder::with_mqtt_version(MQTT_VERSION_5)
        .clean_start(false)
        .properties(paho_mqtt::properties![paho_mqtt::PropertyCode::SessionExpiryInterval => 3600])
        .finalize();

    // Make the connection to the broker
    cli.connect(conn_opts).await?;

    let topics: &[&str] = &[&spaeter_core::topics::signal_peak_topic(None)];
    let qos: &[i32] = &[1];

    println!("Subscribing to topics: {:?}", topics);
    let sub_opts = vec![paho_mqtt::SubscribeOptions::with_retain_as_published(); topics.len()];
    cli.subscribe_many_with_options(topics, qos, &sub_opts, None)
        .await?;

    // Just loop on incoming messages.
    println!("Waiting for messages...");

    // Note that we're not providing a way to cleanly shut down and
    // disconnect. Therefore, when you kill this app (with a ^C or
    // whatever) the server will get an unexpected drop and then
    // should emit the LWT message.

    while let Some(msg_opt) = strm.next().await {
        if let Some(msg) = msg_opt {
            tokio::spawn(process_message(msg, signal_peak_processor));
        } else {
            // A "None" means we were disconnected. Try to reconnect...
            println!("Lost connection. Attempting reconnect.");
            while let Err(err) = cli.reconnect().await {
                println!("Error reconnecting: {}", err);
                tokio::time::sleep(Duration::from_millis(1000)).await;
            }
        }
    }

    Ok(())
}

async fn process_message(message: paho_mqtt::Message, signal_peak_processor: &SignalPeakProcessor) {
    if let Some(anchor_id) = spaeter_core::topics::parse_signal_peak_topic(message.topic()) {
        match spaeter_core::topics::SignalPeakPayload::deserialize(message.payload()) {
            Ok(signal_peak) => {
                println!("Received signal peak: \n\t- {anchor_id:?}\n\t- {signal_peak:?}");
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
