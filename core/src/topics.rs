use crate::{AnchorId, Timestamped, TopicData};
use core::fmt::Write;
use signal_detector::SignalPeak;

pub const SIGNAL_PEAK_TOPIC: &str = "spaeter/anchor/+/signal_peak";
pub type SignalPeakPayload = Timestamped<SignalPeak>;
pub fn signal_peak_topic(
    anchor_id: AnchorId,
) -> heapless::String<{ SIGNAL_PEAK_TOPIC.len() + 10 }> {
    let mut s = heapless::String::new();

    let (l, r) = SIGNAL_PEAK_TOPIC.split_once("+").unwrap();
    s.push_str(l).unwrap();
    write!(s, "{:08X}", anchor_id.0).unwrap();
    s.push_str(r).unwrap();

    s
}

impl TopicData for SignalPeakPayload {
    type Error = postcard::Error;

    fn serialize<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a mut [u8], Self::Error> {
        postcard::to_slice(self, buffer)
    }

    fn deserialize<'a>(buffer: &'a [u8]) -> Result<(Self, &'a [u8]), Self::Error>
    where
        Self: Sized,
    {
        postcard::take_from_bytes(buffer)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signal_peak_topic_format() {
        assert_eq!(&signal_peak_topic(AnchorId(0x123F)), "spaeter/anchor/0000123F/signal_peak");
    }
}
