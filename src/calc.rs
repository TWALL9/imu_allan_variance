//! Calculation system for IMU measurements
//! In an attempt to keep it compatible with Kalibr, some quirks were retained from
//! the original ROS `allan_variance_ros2` package
//! 1. Angular velocity is recorded as deg/s rather than rad/s

use std::f64::consts::PI;

use anyhow::Result;
use nalgebra::Vector6;

use super::messages;

/// `Vec6` is merely an efficient wrapper for IMU measurements
/// Index 0-2: linear acceleration in m/s^2
/// Index 3-5: angular velocity in deg/s
pub type Vec6 = Vector6<f64>;

/// Converts radians to degrees
fn rad_to_deg(rad: f64) -> f64 {
    rad * 180.0 / PI
}

/// Generates a series of averages used to compute the Allan variance of a set
/// The averages are based on `cluster_size` groupings in `messages`
/// The averages are based on distinct groupings of messages, there is no overlap
/// Typically, `cluster_size` is computed as tau / sampling time
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

/// Compute the Allan variance of each element of a Vec6
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
