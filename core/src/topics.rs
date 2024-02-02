use crate::{AnchorId, Timestamp, Timestamped, TopicData};
use core::fmt::Write;
use glam::Vec3;
use signal_detector::SignalPeak;

const SIGNAL_PEAK_TOPIC: &str = "spaeter/anchor/+/signal_peak";
pub type SignalPeakPayload = Timestamped<SignalPeak>;
pub fn signal_peak_topic(
    anchor_id: Option<AnchorId>,
) -> heapless::String<{ SIGNAL_PEAK_TOPIC.len() + 10 }> {
    let mut s = heapless::String::new();

    let (l, r) = SIGNAL_PEAK_TOPIC.split_once('+').unwrap();
    s.push_str(l).unwrap();
    if let Some(anchor_id) = anchor_id {
        write!(s, "{:08X}", anchor_id.0).unwrap();
    } else {
        s.push('+').unwrap();
    }
    s.push_str(r).unwrap();

    s
}
pub fn parse_signal_peak_topic(topic: &str) -> Option<AnchorId> {
    let (l, r) = SIGNAL_PEAK_TOPIC.split_once('+').unwrap();

    let anchor_string = topic.strip_prefix(l)?.strip_suffix(r)?;

    Some(AnchorId(u32::from_str_radix(anchor_string, 16).ok()?))
}

#[derive(Debug, Clone)]
pub enum Error {
    Ser(serde_json_core::ser::Error),
    De(serde_json_core::de::Error),
}

impl From<serde_json_core::de::Error> for Error {
    fn from(v: serde_json_core::de::Error) -> Self {
        Self::De(v)
    }
}

impl From<serde_json_core::ser::Error> for Error {
    fn from(v: serde_json_core::ser::Error) -> Self {
        Self::Ser(v)
    }
}

impl TopicData for SignalPeakPayload {
    type Error = Error;

    fn serialize(&self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(serde_json_core::to_slice(self, buffer)?)
    }

    fn deserialize(buffer: &[u8]) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        Ok(serde_json_core::from_slice(buffer).map(|(t, _)| t)?)
    }
}

pub const DETECTED_LOCATION_TOPIC: &str = "spaeter/detected_location";

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct DetectedLocation {
    pub location: Vec3,
    pub frequency: f32,
    pub confidence: f32,
    pub timestamp: Timestamp,
}

impl TopicData for DetectedLocation {
    type Error = Error;

    fn serialize(&self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(serde_json_core::to_slice(self, buffer)?)
    }

    fn deserialize(buffer: &[u8]) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        Ok(serde_json_core::from_slice(buffer).map(|(t, _)| t)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signal_peak_topic_format() {
        assert_eq!(
            &signal_peak_topic(Some(AnchorId(0x123F))),
            "spaeter/anchor/0000123F/signal_peak"
        );
        assert_eq!(&signal_peak_topic(None), "spaeter/anchor/+/signal_peak");
    }
}
