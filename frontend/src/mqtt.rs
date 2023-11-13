use std::time::Duration;

use bevy::prelude::*;
use rumqttc::{Client, Event, Incoming, MqttOptions};
use spaeter_core::{topics::DetectedLocation, TopicData};
use tokio::sync::mpsc::Receiver;

#[derive(Resource)]
pub struct MqttConnection {
    detected_locations: Receiver<DetectedLocation>,
}

impl MqttConnection {
    pub fn new(url: String) -> Self {
        let mut mqttoptions = MqttOptions::parse_url(url).unwrap();
        mqttoptions.set_keep_alive(Duration::from_secs(5));

        let (mut client, mut connection) = Client::new(mqttoptions, 10);
        client
            .subscribe(
                spaeter_core::topics::DETECTED_LOCATION_TOPIC,
                rumqttc::QoS::AtMostOnce,
            )
            .unwrap();

        let (sender, receiver) = tokio::sync::mpsc::channel(32);

        std::thread::spawn(move || loop {
            for result in connection.iter() {
                match result {
                    Ok(Event::Incoming(Incoming::Publish(message))) => {
                        if message.topic == spaeter_core::topics::DETECTED_LOCATION_TOPIC {
                            let location = DetectedLocation::deserialize(&message.payload).unwrap();
                            sender.blocking_send(location).unwrap();
                        }
                    }
                    Ok(_) => {}
                    Err(e) => println!("{e}"),
                }
            }
        });

        MqttConnection {
            detected_locations: receiver,
        }
    }

    pub fn get_new_detected_locations(&mut self) -> impl Iterator<Item = DetectedLocation> + '_ {
        std::iter::from_fn(|| self.detected_locations.try_recv().ok())
    }
}
