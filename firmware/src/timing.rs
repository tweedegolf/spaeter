use core::ops::Div;
use fixed::types::I96F32;
use fugit::HertzU32;
use libm::{ceil, pow, sqrt};
use statime::time::{Duration, Time};

#[derive(Copy, Clone, Debug, defmt::Format, Eq, PartialEq)]
pub struct SampleIndex(pub u64);

#[derive(Copy, Clone, Debug, defmt::Format, Eq, PartialEq, Ord, PartialOrd)]
pub struct TimerValue(pub u64);

impl Div<HertzU32> for TimerValue {
    type Output = statime::time::Duration;

    fn div(self, rhs: HertzU32) -> Self::Output {
        let secs = statime::time::Duration::from_secs(self.0 as i64) / u64::from(rhs.to_Hz());
        secs
    }
}

pub type Observation = (TimerValue, Time);

pub struct TimerObservations {
    obs: heapless::Deque<Observation, { Self::LEN }>,
    tim2_clk: HertzU32,
    adc_clk: HertzU32,
}

impl TimerObservations {
    const LEN: usize = 128;

    // At 12 bits per datasheet
    const ADCCLK_PER_SAMPLE: u64 = 15;
    // assumes the sample is taken exactly at the end
    const ADCCLK_SAMPLING_TIME: u64 = 3;

    pub fn new(tim2_clk: HertzU32, adc_clk: HertzU32) -> Self {
        assert_eq!(tim2_clk.to_Hz() % adc_clk.to_Hz(), 0);
        Self {
            obs: Default::default(),
            tim2_clk,
            adc_clk,
        }
    }

    pub fn push(&mut self, timer: TimerValue, timestamp: statime::time::Time) {
        if let Some(&(last_timer, last_timestamp)) = self.obs.back() {
            assert!(
                timer > last_timer,
                "Timer has to be monotonically increasing"
            );

            if timestamp <= last_timestamp {
                // Clock jumped backwards... clear previous measurements
                self.clear();
            }
        }

        if self.obs.is_full() {
            self.obs.pop_front();
        }

        self.obs
            .push_back((timer, timestamp))
            .expect("Queue can not be full because we checked");
    }

    fn sample_idx_to_timer_value(&self, idx: SampleIndex) -> TimerValue {
        assert_eq!(self.tim2_clk.to_Hz() % self.adc_clk.to_Hz(), 0);
        let timer_clks_per_adc_clk = u64::from(self.tim2_clk / self.adc_clk);

        let sampling_start = idx.0 * Self::ADCCLK_PER_SAMPLE * timer_clks_per_adc_clk;
        let sample_offset = Self::ADCCLK_SAMPLING_TIME * timer_clks_per_adc_clk;

        TimerValue(sampling_start + sample_offset)
    }

    pub fn avg_timer_rate(&self) -> Option<HertzU32> {
        if self.obs.len() < 2 {
            return None;
        }

        let first = self.obs.front().unwrap();
        let last = self.obs.back().unwrap();

        Some(Self::rate(&[first, last]))
    }

    fn rate([first, last]: &[&Observation; 2]) -> HertzU32 {
        let delta_cnt = I96F32::from_num(last.0 .0) - I96F32::from_num(first.0 .0);
        let delta_t = last.1 - first.1;

        let rate = I96F32::from_num(1_000_000_000) * delta_cnt / delta_t.nanos();

        HertzU32::Hz(rate.to_num())
    }

    pub fn rate_std_dev(&self) -> Option<HertzU32> {
        let avg = self.avg_timer_rate()?.to_Hz() as f64;

        let squared_error = self
            .obs
            .iter()
            .map_windows(Self::rate)
            .map(|rate| {
                let rate = rate.to_Hz() as f64;
                pow(rate - avg, 2.0)
            })
            .sum::<f64>()
            .div(self.obs.len() as f64 - 1.0);

        Some(HertzU32::Hz(ceil(sqrt(squared_error)) as _))
    }

    pub fn timer_start(&self) -> Option<Time> {
        let avg_rate = self.avg_timer_rate()?;

        let avg_start = self
            .obs
            .iter()
            .map(|&(timer, clock)| {
                let timer_start = clock - (timer / avg_rate);
                timer_start.nanos().to_num::<u128>()
            })
            .sum::<u128>()
            .div(self.obs.len() as u128);

        Some(Time::from_nanos(avg_start as u64))
    }

    pub fn timer_start_fast_and_inaccurate(&self) -> Option<Time> {
        let obs = self.obs.back()?;
        let duration_since_start = obs.0 / self.avg_timer_rate()?;
        let timer_start = obs.1 - duration_since_start;
        Some(timer_start)
    }

    fn to_timestamp(&self, t: TimerValue) -> Option<Time> {
        let time_since_start = t / self.avg_timer_rate()?;
        Some(self.timer_start()? + time_since_start)
    }

    pub fn time_std_dev(&self) -> Option<Duration> {
        if self.obs.len() < 2 {
            return None;
        }

        let squared_error = self
            .obs
            .iter()
            .map(|&(timer, time)| {
                let should = self.to_timestamp(timer).unwrap();
                let diff = should - time;

                pow(diff.seconds(), 2.0)
            })
            .sum::<f64>()
            .div(self.obs.len() as f64);

        let error = sqrt(squared_error);
        let error = error / 1.07026154578459; // Magic correction factor -- determined by experiment
        Some(Duration::from_seconds(error))
    }

    pub fn sample_time(&self, idx: SampleIndex) -> Option<Time> {
        let timer_value = self.sample_idx_to_timer_value(idx);
        self.to_timestamp(timer_value)
    }

    pub fn clear(&mut self) {
        self.obs.clear()
    }

    pub fn is_ready(&self) -> bool {
        self.obs.len() >= 2
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fugit::{Rate, RateExtU32};
    use rand::distributions::Distribution;
    use rand::thread_rng;
    use rand_distr::Normal;
    use statime::time::{Duration, Time};

    #[test]
    #[cfg(not(miri))]
    fn sample_idx_to_value() {
        let obs = TimerObservations::new(10u32.MHz(), 5u32.MHz());

        let offset = TimerObservations::ADCCLK_SAMPLING_TIME * 2;
        assert_eq!(
            obs.sample_idx_to_timer_value(SampleIndex(0)),
            TimerValue(offset)
        );
        assert_eq!(
            obs.sample_idx_to_timer_value(SampleIndex(100)),
            TimerValue(offset + 100 * 2 * TimerObservations::ADCCLK_PER_SAMPLE)
        );
    }

    #[test]
    #[cfg(not(miri))]
    fn timer_rate() {
        let mut obs = TimerObservations::new(10u32.MHz(), 5u32.MHz());

        obs.push(TimerValue(0), Time::from_millis(0));
        obs.push(TimerValue(500), Time::from_millis(500));
        obs.push(TimerValue(1000), Time::from_millis(1_000));

        assert!(obs.is_ready());
        assert_eq!(obs.avg_timer_rate(), Some(1.kHz()));
        assert_eq!(obs.rate_std_dev(), Some(HertzU32::Hz(0)));
        assert_eq!(obs.time_std_dev(), Some(Duration::from_secs(0)));
        assert_eq!(obs.timer_start(), Some(Time::from_secs(0)));
        assert_eq!(
            obs.to_timestamp(TimerValue(42_000)),
            Some(Time::from_secs(42))
        );
    }

    #[test]
    #[cfg(not(miri))]
    fn end2end() {
        generic_e2e(
            1.MHz(),
            Time::from_secs(258_204_777), // 1978-03-08
            Duration::from_millis(100),
        );
        generic_e2e(
            54.MHz(),
            Time::from_secs(1_000_000),
            Duration::from_millis(75),
        );
        generic_e2e(
            1.MHz(),
            Time::from_secs(2_461_446_000),
            Duration::from_millis(60 * 60 * 24),
        );
        generic_e2e(
            101.MHz(),
            Time::from_secs(2_461_446_000),
            Duration::from_millis(60 * 60 * 24),
        );

        fn generic_e2e(adc_clk: Rate<u32, 1, 1>, start_time: Time, sample_interval: Duration) {
            let tim2_clk = adc_clk * 2;
            let samples: usize = 128;

            // Fill observations
            let mut observations = TimerObservations::new(tim2_clk, adc_clk);
            for i in 0..samples {
                let timestamp = start_time + sample_interval * i;
                let timer =
                    (tim2_clk.to_Hz() as i128 * i as i128 * sample_interval.nanos_rounded())
                        / 1_000_000_000;

                let timer = TimerValue(timer.try_into().unwrap());

                observations.push(timer, timestamp);

                if !observations.is_ready() {
                    continue;
                }

                assert_eq!(observations.avg_timer_rate().unwrap(), tim2_clk);
                assert_eq!(observations.timer_start().unwrap(), start_time);

                assert_eq!(observations.time_std_dev().unwrap(), Duration::from_secs(0));
                assert_eq!(observations.rate_std_dev().unwrap(), HertzU32::Hz(0));
            }

            for idx in 0..samples {
                let idx = SampleIndex(idx as u64 * 7);

                let adc_period = Duration::from_secs(1) / adc_clk.to_Hz();
                let sample_offset = adc_period * TimerObservations::ADCCLK_SAMPLING_TIME;

                let calculated_time = observations.sample_time(idx).unwrap();
                let calculated_time = calculated_time.nanos();

                let expected_time = start_time
                    + sample_offset
                    + adc_period * TimerObservations::ADCCLK_PER_SAMPLE * idx.0;
                let expected_time = expected_time.nanos();

                let err = calculated_time.abs_diff(expected_time);

                assert!(err < adc_period.nanos());
            }
        }
    }

    // We assume just taking the rate between first and last element is equal to averaging all rates
    #[test]
    #[cfg(not(miri))]
    fn average_rate_optimization() {
        let adc_clk = 54.MHz();
        let tim2_clk = adc_clk * 2;
        let start_time = Time::from_secs(364 * 24 * 60 * 60);
        let sample_interval = Duration::from_millis(100);
        let dist = Normal::new(0.0, 10_000.0).unwrap();
        let rng = &mut thread_rng();

        let mut obs = TimerObservations::new(tim2_clk, adc_clk);
        let mut errs = vec![];

        for i in 0..128 {
            let err_nanos = dist.sample(rng) as i64;
            errs.push(err_nanos);
            let err = Duration::from_nanos(err_nanos);
            let timestamp = start_time + sample_interval * i + err;
            let timer = (tim2_clk.to_Hz() as i128 * i as i128 * sample_interval.nanos_rounded())
                / 1_000_000_000;

            let timer = TimerValue(timer.try_into().unwrap());

            obs.push(timer, timestamp);

            if !obs.is_ready() {
                continue;
            }

            let fast_avg = obs.avg_timer_rate().unwrap();
            let slow_avg = obs
                .obs
                .iter()
                .map_windows(|ab| TimerObservations::rate(ab))
                .map(|rate| rate.to_Hz() as usize)
                .sum::<usize>()
                / (obs.obs.len() - 1);

            let error = fast_avg.to_Hz().abs_diff(slow_avg as u32);
            let max_err = 2 * obs.obs.len() as u32;
            assert!(
                error <= max_err, // We allow two Hz of error per element being summed
                "Error too big... is: {error}, should at most be: {max_err}"
            );
        }
    }

    #[test]
    #[cfg(not(miri))]
    fn std_dev() {
        let adc_clk = 54.MHz();
        let tim2_clk = adc_clk * 2;
        let start_time = Time::from_secs(364 * 24 * 60 * 60);
        let sample_interval = Duration::from_millis(100);
        let dist = Normal::new(0.0, 1_000.0).unwrap();
        let rng = &mut thread_rng();

        let mut obs = TimerObservations::new(tim2_clk, adc_clk);
        let mut errs = vec![];

        let mut rate_errs = vec![];
        let mut time_errs = vec![];
        let mut true_errs = vec![];

        for i in 0..(2 * TimerObservations::LEN) {
            let err_nanos = dist.sample(rng) as i64;
            errs.push(err_nanos);
            let err = Duration::from_nanos(err_nanos);
            let timestamp = start_time + sample_interval * i + err;
            let timer = (tim2_clk.to_Hz() as i128 * i as i128 * sample_interval.nanos_rounded())
                / 1_000_000_000;

            let timer = TimerValue(timer.try_into().unwrap());

            obs.push(timer, timestamp);

            if obs.obs.len() < TimerObservations::LEN {
                continue;
            }
            if errs.len() > obs.obs.len() {
                errs.remove(0);
            }
            assert_eq!(obs.obs.len(), errs.len());

            let err_mean = errs.iter().map(|&e| e as f64).sum::<f64>() / errs.len() as f64;
            let expected_dev = errs
                .iter()
                .map(|&e| (e as f64 - err_mean).powi(2))
                .sum::<f64>()
                .div(errs.len() as f64)
                .sqrt();
            let calculated_dev = obs.time_std_dev().unwrap().nanos_lossy();
            time_errs.push(calculated_dev);
            true_errs.push(expected_dev);

            let _error = (expected_dev - calculated_dev).abs();
            let _max_err = 20.0;

            let _start: f64 = obs.timer_start().unwrap().nanos().to_num();
            let _rate = obs.avg_timer_rate().unwrap().to_Hz();
            let rate_err = obs.rate_std_dev().unwrap().to_Hz();
            rate_errs.push(rate_err as f64);
        }

        let avg_time_e = time_errs.iter().sum::<f64>().div(time_errs.len() as f64);
        let avg_rate_e = rate_errs.iter().sum::<f64>().div(rate_errs.len() as f64);
        let avg_true_e = true_errs.iter().sum::<f64>().div(true_errs.len() as f64);

        dbg!(avg_time_e, avg_time_e / avg_true_e);
        dbg!(avg_true_e);
        dbg!(avg_rate_e, avg_rate_e / avg_true_e);
    }
}
