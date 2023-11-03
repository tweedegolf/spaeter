use std::{
    process,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use paho_mqtt as mqtt;
use signal_detector::SignalPeak;
use spaeter_core::{
    topics::{signal_peak_topic, SignalPeakPayload},
    AnchorId, Timestamp, TopicData,
};

fn main() {
    // Create a client & define connect options
    let cli = mqtt::Client::new("tcp://localhost:1883").unwrap_or_else(|err| {
        println!("Error creating the client: {:?}", err);
        process::exit(1);
    });

    let conn_opts = mqtt::ConnectOptionsBuilder::new()
        .keep_alive_interval(Duration::from_secs(20))
        .clean_session(true)
        .finalize();

    // Connect and wait for it to complete or fail
    if let Err(e) = cli.connect(conn_opts) {
        println!("Unable to connect:\n\t{:?}", e);
        process::exit(1);
    }

    let current_time = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();

    for anchor in [AnchorId(0), AnchorId(1), AnchorId(2), AnchorId(3)] {
        for ts in [
            Timestamp::from_millis(0),
            Timestamp::from_millis(1),
            Timestamp::from_millis(2),
            Timestamp::from_millis(3),
        ] {
            let mut buffer = [0; 512];

            let len = SignalPeakPayload {
                timestamp: Timestamp::new(current_time.as_nanos() as u64, 0) + ts + Timestamp::from_millis(anchor.0 as u64 * 5),
                value: SignalPeak {
                    freq: 1000.0,
                    magnitude: 10.0,
                },
            }
            .serialize(&mut buffer)
            .unwrap();

            // Create a message and publish it
            let msg = mqtt::Message::new(
                signal_peak_topic(Some(anchor)).as_str(),
                &buffer[..len],
                0, // At most once
            );
            if let Err(e) = cli.publish(msg) {
                println!("Error sending message: {:?}", e);
            }
        }
    }

    // Disconnect from the broker
    cli.disconnect(None).unwrap();
}
