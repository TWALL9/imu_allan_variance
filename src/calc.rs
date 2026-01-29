use std::f64::consts::PI;

use anyhow::Result;
use nalgebra::Vector6;

use super::messages;

pub type Vec6 = Vector6<f64>;

fn rad_to_deg(rad: f64) -> f64 {
    rad * 180.0 / PI
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
