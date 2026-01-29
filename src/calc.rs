use std::{
    collections::BTreeMap,
    f64::consts::PI,
    time::{Duration, SystemTime},
};

use anyhow::Result;
use log::info;
use nalgebra::Vector6;
use rayon::prelude::*;

use super::messages;

pub type Vec6 = Vector6<f64>;

fn rad_to_deg(rad: f64) -> f64 {
    rad * 180.0 / PI
}

pub fn range(messages: &[messages::Imu], start: SystemTime, end: SystemTime) -> &[messages::Imu] {
    let start = messages.partition_point(|x| x.ts <= start);
    let end = messages.partition_point(|x| x.ts <= end);

    &messages[start..end]
}

pub fn averages_non_overlapping(
    messages: &[messages::Imu],
    cluster_size: usize,
) -> Result<Vec<Vec6>> {
    if cluster_size == 0 {
        return Err(anyhow::anyhow!("Cluster size is too small to use"));
    }

    let n_clusters = messages.len() / cluster_size;
    if n_clusters < 2 {
        return Err(anyhow::anyhow!(
            "avar averaging over too many messages: {} messages with cluster of {}",
            messages.len(),
            cluster_size
        ));
    }

    let mut averages = Vec::with_capacity(n_clusters);
    for i in 0..n_clusters {
        let start = i * cluster_size;
        let end = start + cluster_size;
        let sum: Vec6 = messages[start..end]
            .iter()
            .map(|m| {
                Vec6::new(
                    m.linear_acceleration.x,
                    m.linear_acceleration.y,
                    m.linear_acceleration.z,
                    rad_to_deg(m.angular_velocity.x),
                    rad_to_deg(m.angular_velocity.y),
                    rad_to_deg(m.angular_velocity.z),
                )
            })
            .sum();
        averages.push(sum / cluster_size as f64);
    }

    Ok(averages)
}

pub fn allan_variance(averages: &[Vec6]) -> Vec6 {
    let mut sum_squares = Vec6::default();
    let mut prev: Option<&Vec6> = None;
    for avg in averages.iter() {
        if let Some(p) = prev {
            let diff: Vec6 = avg - p;
            let diff_squared = diff.map(|x| x * x);
            sum_squares += diff_squared;
        }
        prev = Some(avg);
    }

    let mut avar = Vec6::default();

    for i in 0..6 {
        avar[i] = 0.5 * sum_squares[i] / (averages.len() as f64 - 1.0);
    }

    avar
}

/// avar calc that is a reimplementation of the ROS version
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

    pub fn run(
        &self,
        messages: &[messages::Imu],
        min_period: i32,
        max_period: i32,
    ) -> Result<Vec<(f64, Vec6)>> {
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

        let averages = self.calc_averages(range, min_period, max_period);
        let deviations = self.calc_deviations(&averages);

        Ok(deviations)
    }

    fn calc_averages(
        &self,
        messages: &[messages::Imu],
        min_period: i32,
        max_period: i32,
    ) -> BTreeMap<i32, Vec<Vec6>> {
        let averages: Vec<(i32, Vec<Vec6>)> = (min_period..max_period)
            .into_par_iter()
            .map(|period| {
                let av = self.average_calc(messages, period);

                info!(
                    "Computed {:?} averages for period {} ({}) left",
                    av.1.len(),
                    period as f64 * 0.1,
                    (max_period - period)
                );

                av
            })
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
                current_average += Vec6::new(
                    messages[i + j].linear_acceleration.x,
                    messages[i + j].linear_acceleration.y,
                    messages[i + j].linear_acceleration.z,
                    rad_to_deg(messages[i + j].angular_velocity.x),
                    rad_to_deg(messages[i + j].angular_velocity.y),
                    rad_to_deg(messages[i + j].angular_velocity.z),
                );
            }

            current_average
                .iter_mut()
                .for_each(|x| *x /= max_bin_size as f64);
            averages.push(current_average);
        }

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
        let mut previous: Option<&Vec6> = None;
        for curr in averages {
            if let Some(prev) = previous {
                for j in 0..5 {
                    allan_variance[j] += (curr[j] - prev[j]).powi(2);
                    allan_variance[j] *= 0.5 / (num_averages - 1) as f64;
                }
            }

            previous = Some(curr);
        }

        let mut allan_deviation = allan_variance;
        allan_deviation.iter_mut().for_each(|x| *x = x.sqrt());

        (period_time, allan_deviation)
    }
}
