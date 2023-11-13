use std::{error::Error, path::Path};

use glam::Vec3;

use crate::AnchorId;

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Anchor {
    pub id: AnchorId,
    pub location: Vec3,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Config {
    pub anchors: Vec<Anchor>,
}

impl Config {
    pub fn load(path: Option<&Path>) -> Result<Self, Box<dyn Error>> {
        let paths = match path {
            Some(p) => vec![p.into()],
            None => std::env::current_dir()?
                .ancestors()
                .map(|path| path.join("server-config.toml"))
                .collect(),
        };

        let mut last_error = None;

        for path in paths {
            match std::fs::read_to_string(path) {
                Ok(config_string) => return Ok(toml::from_str(&config_string)?),
                Err(e) => last_error = Some(e),
            }
        }

        Err(last_error.unwrap().into())
    }
}
