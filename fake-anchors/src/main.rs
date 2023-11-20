use std::{
    thread,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use glam::{vec3, Vec3};
use rand::Rng;
use rumqttc::{Client, MqttOptions, QoS};
use signal_detector::{SignalDetector, SignalPeak};
use spaeter_core::{topics::signal_peak_topic, AnchorId, Timestamp, Timestamped, TopicData};

const SPEED_OF_SOUND: f64 = 343.0;
const SAMPLE_SPEED: f64 = 44100.0;
const TIME_PER_SAMPLE: f64 = 1.0 / SAMPLE_SPEED;

fn main() {
    let config = spaeter_core::config::Config::load(None).unwrap();

    let mut mqttoptions = MqttOptions::new("fake-anchors", "127.0.0.1", 1883);
    mqttoptions.set_keep_alive(Duration::from_secs(5));

    let (mut client, mut connection) = Client::new(mqttoptions, 10);

    thread::spawn(move || {
        let mut fake_anchors = config
            .anchors
            .iter()
            .map(|anchor| FakeAnchor {
                id: anchor.id,
                location: anchor.location,
                signal_detector: SignalDetector::new(SAMPLE_SPEED as f32),
            })
            .collect::<Vec<_>>();

        let max_position = fake_anchors
            .iter()
            .map(|a| a.location)
            .reduce(Vec3::max)
            .unwrap();
        let min_position = fake_anchors
            .iter()
            .map(|a| a.location)
            .reduce(Vec3::min)
            .unwrap();

        for _ in 0..10 {
            let test_position = vec3(
                rand::thread_rng().gen_range(min_position.x..max_position.x),
                rand::thread_rng().gen_range(min_position.y..max_position.y),
                rand::thread_rng().gen_range(min_position.z..max_position.z),
            );

            let mut test_samples = [0.0; 4410usize.next_multiple_of(64)];

            let freq1 = rand::thread_rng().gen_range(500.0..15000.0);
            let freq2 = rand::thread_rng().gen_range(500.0..15000.0);
            let freq3 = rand::thread_rng().gen_range(500.0..15000.0);

            // add_signal(&mut test_samples[100..200], 100, freq1, 1.0, 0.0);
            add_signal(&mut test_samples[700..1500], 700, freq2, 1.0, 0.0);
            // add_signal(&mut test_samples[3510..4000], 3510, freq3, 1.0, 0.0);

            println!("Signal\n\tfreq1: {freq1:.1},\n\tfreq2: {freq2:.1},\n\tfreq3: {freq3:.1},\n\tlocation: {test_position}");

            let current_time = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();

            for anchor in fake_anchors.iter_mut() {
                let id = anchor.id;

                let mut test_samples = test_samples.clone();
                add_random_noise(&mut test_samples, 0.1);

                anchor.feed(
                    &test_samples,
                    test_position,
                    Timestamp::new(current_time.as_nanos() as u64, 0),
                    |payload| {
                        // println!("{payload:?}");

                        let mut buffer = [0; 1024];

                        let len = payload.serialize(&mut buffer).unwrap();

                        client
                            .publish(
                                signal_peak_topic(Some(id)).as_str(),
                                QoS::ExactlyOnce,
                                false,
                                &buffer[..len],
                            )
                            .unwrap();
                    },
                );
            }

            std::thread::sleep(Duration::from_secs_f32(0.5));
        }
    });

    while let Ok(_) = connection.recv_timeout(Duration::from_millis(600)) {}
}

struct FakeAnchor {
    id: AnchorId,
    location: Vec3,
    signal_detector: SignalDetector,
}

impl FakeAnchor {
    fn feed(
        &mut self,
        samples: &[f32],
        audio_source_location: Vec3,
        samples_start_time: Timestamp,
        mut f: impl FnMut(Timestamped<SignalPeak>),
    ) {
        let sound_delay = Timestamp::from_nanos_f64(
            self.location.distance(audio_source_location) as f64 / SPEED_OF_SOUND * 1_000_000_000.0,
        );

        self.signal_detector.feed(samples, |index, peaks| {
            let timestamp =
                Timestamp::from_nanos_f64(index as f64 * TIME_PER_SAMPLE * 1_000_000_000.0)
                    + samples_start_time
                    + sound_delay;
            for peak in peaks {
                f(Timestamped {
                    timestamp,
                    value: *peak,
                })
            }
        });
    }
}

fn add_signal(
    samples: &mut [f32],
    start_sample_index: usize,
    frequency: f32,
    amplitude: f32,
    phase: f32,
) {
    samples.iter_mut().enumerate().for_each(|(i, sample)| {
        let i = i + start_sample_index;
        *sample += amplitude
            * (2.0 * std::f32::consts::PI * frequency * (i as f32 / SAMPLE_SPEED as f32) + phase)
                .sin();
    });
}

fn add_random_noise(samples: &mut [f32], amplitude: f32) {
    samples
        .iter_mut()
        .for_each(|sample| *sample += (rand::random::<f32>() - 0.5) * amplitude * 2.0);
}
