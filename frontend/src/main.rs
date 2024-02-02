use std::{
    collections::VecDeque,
    time::{SystemTime, UNIX_EPOCH},
};

use bevy::{diagnostic::LogDiagnosticsPlugin, pbr::NotShadowCaster, prelude::*};
use bevy_flycam::prelude::*;
use bevy_inspector_egui::{
    prelude::ReflectInspectorOptions, quick::WorldInspectorPlugin, InspectorOptions,
};
use mqtt::MqttConnection;
use spaeter_core::{topics::DetectedLocation, Timestamp};

mod mqtt;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WorldInspectorPlugin::new()) // Debug window
        .add_plugins(NoCameraPlayerPlugin) // Fly cam
        .add_plugins(LogDiagnosticsPlugin::default())
        // .add_plugins(FrameTimeDiagnosticsPlugin)
        .add_systems(Startup, setup)
        .insert_resource(MqttConnection::new(
            "mqtt://127.0.0.1:1883?client_id=spaeter-frontend".into(),
        ))
        .register_type::<LocationStats>()
        .insert_resource(LocationStats {
            confidences: Default::default(),
        })
        .add_systems(Update, mqtt_update)
        .add_systems(Update, location_sphere_runner)
        .run();
}

/// set up a simple 3D scene
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            shadows_enabled: true,
            ..Default::default()
        },
        transform: Transform::from_xyz(0.0, 0.0, 0.0).with_rotation(Quat::from_euler(
            EulerRot::XYZ,
            -2.5,
            0.7,
            0.0,
        )),
        ..Default::default()
    });
    commands.insert_resource(AmbientLight {
        brightness: 0.3,
        color: Color::Rgba {
            red: 0.6,
            green: 0.4,
            blue: 0.8,
            alpha: 1.0,
        },
    });
    commands.spawn(PbrBundle {
        mesh: meshes.add(shape::Plane::from_size(10.0).into()),
        material: materials.add(StandardMaterial { ..default() }),
        transform: Transform::from_xyz(5.0, 0.0, 5.0),
        ..Default::default()
    });

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-12.0, 15.0, 9.0).with_rotation(Quat::from_euler(
                EulerRot::XYZ,
                -1.36,
                -0.829,
                -1.288,
            )),
            ..default()
        },
        FlyCam,
    ));
}

fn mqtt_update(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut connection: ResMut<MqttConnection>,
    mut stats: ResMut<LocationStats>,
) {
    for detected_location in connection.get_new_detected_locations() {
        println!("{detected_location:?}");

        stats.add_confidence(detected_location.confidence);

        commands.spawn((
            PbrBundle {
                mesh: meshes.add(
                    shape::Icosphere {
                        radius: 1.0,
                        subdivisions: 4,
                    }
                    .try_into()
                    .unwrap(),
                ),
                material: materials.add(StandardMaterial {
                    base_color: Color::hsla(
                        detected_location.frequency / 40_000.0 * 360.0,
                        (detected_location.confidence / stats.high_confidence_value())
                            .clamp(0.0, 1.0)
                            .sqrt(),
                        0.5,
                        (detected_location.confidence / stats.high_confidence_value())
                            .clamp(0.0, 1.0),
                    ),
                    alpha_mode: AlphaMode::Blend,
                    unlit: true,
                    ..default()
                }),
                transform: Transform::from_translation(detected_location.location)
                    .with_scale(Vec3::new(0.0, 0.0, 0.0)),
                ..Default::default()
            },
            DetectedLocationComponent(detected_location),
            NotShadowCaster,
        ));
    }
}

#[derive(Component)]
struct DetectedLocationComponent(DetectedLocation);

#[derive(Resource, Reflect, InspectorOptions)]
#[reflect(InspectorOptions)]
struct LocationStats {
    confidences: VecDeque<f32>,
}

impl LocationStats {
    fn add_confidence(&mut self, value: f32) {
        self.confidences.push_back(value);

        if self.confidences.len() > 1024 {
            self.confidences.pop_front();
        }
    }

    fn high_confidence_value(&self) -> f32 {
        let average = self.confidences.iter().sum::<f32>() / self.confidences.len() as f32;
        average
    }
}

const MAX_AGE_SECS: f32 = 10.0;
const MAX_AGE_SIZE: f32 = 2.0;

fn location_sphere_runner(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    time: Res<Time>,
    stats: Res<LocationStats>,
    mut query: Query<(
        Entity,
        &mut Transform,
        &DetectedLocationComponent,
        &mut Handle<StandardMaterial>,
    )>,
) {
    let current_time = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();

    for (entity, mut transform, location, material) in &mut query {
        let confidence = (location.0.confidence / stats.high_confidence_value()).clamp(0.0, 1.0);

        let age = location
            .0
            .timestamp
            .difference_to(Timestamp::new(current_time.as_nanos() as u64, 0))
            .as_micros() as f32
            / (MAX_AGE_SECS * 1_000_000.0)
            / confidence.sqrt();

        if age >= 1.0 {
            commands.entity(entity).despawn();
        } else {
            let target_scale = Vec3::ONE * age.sqrt() * MAX_AGE_SIZE * confidence;
            let current_scale = transform.scale;
            transform.scale += (target_scale - current_scale) * time.delta_seconds();

            materials
                .get_mut(material.id())
                .unwrap()
                .base_color
                .set_a(((1.0 - age.sqrt()) * confidence).powf(2.0));
        }
    }
}
