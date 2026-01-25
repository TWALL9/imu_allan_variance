use std::{
    collections::BTreeMap,
    time::{Duration, SystemTime},
};

use anyhow::Result;
use log::info;
use rayon::prelude::*;

use super::messages;

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

    pub fn run(&self, messages: &[messages::Imu]) -> Result<BTreeMap<i32, Vec<[f64; 6]>>> {
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

        self.calc_variance(range)
    }

    fn calc_variance(&self, messages: &[messages::Imu]) -> Result<BTreeMap<i32, Vec<[f64; 6]>>> {
        let variance_calc = |messages: &[messages::Imu], period: i32| -> (i32, Vec<[f64; 6]>) {
            let mut averages = vec![];
            let period_time = period as f64 * 0.1;

            let max_bin_size = (period_time * self.measure_rate) as usize;

            let mut current_average: [f64; 6] = [0.0; 6];

            for i in (0..(messages.len() - max_bin_size)).step_by(max_bin_size) {
                for j in 0..max_bin_size {
                    current_average[0] += messages[i + j].linear_acceleration.x;
                    current_average[1] += messages[i + j].linear_acceleration.y;
                    current_average[2] += messages[i + j].linear_acceleration.z;

                    current_average[3] +=
                        messages[i + j].angular_velocity.x * 180.0 / std::f64::consts::FRAC_PI_2;
                    current_average[4] +=
                        messages[i + j].angular_velocity.y * 180.0 / std::f64::consts::FRAC_PI_2;
                    current_average[5] +=
                        messages[i + j].angular_velocity.z * 180.0 / std::f64::consts::FRAC_PI_2;
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
        };

        let variances: Vec<(i32, Vec<[f64; 6]>)> = (1..10000)
            .into_par_iter()
            .map(|period| variance_calc(messages, period))
            .collect();

        let mut averages_map: BTreeMap<i32, Vec<[f64; 6]>> = BTreeMap::new();
        for (key, val) in variances {
            averages_map.insert(key, val);
        }
        Ok(averages_map)
    }
}
