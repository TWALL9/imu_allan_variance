use std::{
    collections::BTreeMap,
    iter,
    time::{Duration, SystemTime},
};

use std::f64::consts::FRAC_PI_2 as PI_2;

use anyhow::Result;
use log::info;
use rayon::prelude::*;

use super::messages;

type Vec6 = [f64; 6];

fn rad_to_deg(rad: f64) -> f64 {
    rad * 180.0 / PI_2
}

fn range<'a>(
    messages: &'a [messages::Imu],
    start: SystemTime,
    end: SystemTime,
) -> &'a [messages::Imu] {
    let start = messages.partition_point(|x| x.ts <= start);
    let end = messages.partition_point(|x| x.ts <= end);

    &messages[start..end]
}

#[derive(Debug)]
pub struct VarianceCalculator {
    measure_rate: f64,
    sequence_duration: Option<f64>,
    sequence_offset: f64,
}

impl VarianceCalculator {
    pub fn new(measure_rate: f64, sequence_duration: Option<f64>, sequence_offset: f64) -> Self {
        Self {
            measure_rate,
            sequence_duration,
            sequence_offset,
        }
    }

    pub fn run(&self, messages: &[messages::Imu]) -> Result<Vec<(f64, Vec6)>> {
        if messages.is_empty() {
            return Err(anyhow::anyhow!("Empty IMU messages!"));
        }

        let first_timestamp = messages.iter().next().unwrap().ts;

        let offset_timestamp = first_timestamp + Duration::new(self.sequence_offset as u64, 0);

        let end_timestamp = match self.sequence_duration {
            Some(ts) => offset_timestamp + Duration::new(ts as u64, 0),
            None => {
                let last_message = messages
                    .last()
                    .unwrap_or_else(|| panic!("Somehow no last message?"));
                last_message.ts
            }
        };

        let range = range(messages, offset_timestamp, end_timestamp);

        let averages = self.calc_averages(range);
        let deviations = self.calc_deviations(&averages);

        Ok(deviations)
    }

    fn calc_averages(&self, messages: &[messages::Imu]) -> BTreeMap<i32, Vec<Vec6>> {
        let averages: Vec<(i32, Vec<Vec6>)> = (1..10000)
            .into_par_iter()
            .map(|period| self.average_calc(messages, period))
            .collect();

        let mut averages_map: BTreeMap<i32, Vec<Vec6>> = BTreeMap::new();
        for (key, val) in averages {
            averages_map.insert(key, val);
        }

        averages_map
    }

    fn average_calc(&self, messages: &[messages::Imu], period: i32) -> (i32, Vec<Vec6>) {
        let mut averages = vec![];
        let period_time = period as f64 * 0.1;

        let max_bin_size = (period_time * self.measure_rate) as usize;

        for i in (0..(messages.len() - max_bin_size)).step_by(max_bin_size) {
            let mut current_average = Vec6::default();

            for j in 0..max_bin_size {
                current_average[0] += messages[i + j].linear_acceleration.x;
                current_average[1] += messages[i + j].linear_acceleration.y;
                current_average[2] += messages[i + j].linear_acceleration.z;

                current_average[3] += rad_to_deg(messages[i + j].angular_velocity.x);
                current_average[4] += rad_to_deg(messages[i + j].angular_velocity.y);
                current_average[5] += rad_to_deg(messages[i + j].angular_velocity.z);
            }

            current_average
                .iter_mut()
                .for_each(|x| *x /= max_bin_size as f64);
            averages.push(current_average);
        }

        info!(
            "Computed {:?} averages for period {} ({}) left",
            averages.len(),
            period_time,
            (10000 - period)
        );

        (period, averages)
    }

    fn calc_deviations(&self, averages_map: &BTreeMap<i32, Vec<Vec6>>) -> Vec<(f64, Vec6)> {
        (1..10000)
            .into_par_iter()
            .map(|period| self.deviation_calc(&averages_map[&period], period))
            .collect()
    }

    fn deviation_calc(&self, averages: &[Vec6], period: i32) -> (f64, Vec6) {
        let period_time: f64 = period as f64 * 0.1;
        let num_averages = averages.len();

        info!(
            "Computed {} bins for sampling period {} out of {} measurements.",
            num_averages, period_time, 0
        );

        let mut allan_variance = Vec6::default();
        let mut previous: Option<Vec6> = None;
        for i in 0..num_averages {
            let curr = averages[i];
            if let Some(prev) = previous {
                for j in 0..5 {
                    allan_variance[j] += (curr[j] - prev[j]).powi(2);
                    allan_variance[j] /= 2.0 * (num_averages - 1) as f64;
                }
            }

            previous = Some(curr);
        }

        let mut allan_deviation = allan_variance.clone();
        allan_deviation.iter_mut().for_each(|x| *x = x.sqrt());

        (period_time, allan_deviation)
    }
}
