use std::{collections::HashMap, fs, path::PathBuf};

use anyhow::Result;
use mcap;
use memmap2::Mmap;
use serde::Deserialize;

#[derive(Debug, Default, Clone, Copy, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Default, Clone, Copy, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Debug, Default, Clone, Copy, Deserialize)]
pub struct Timestamp {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Debug, Default, Deserialize)]
pub struct Header {
    pub ts: Timestamp,
    pub frame_id: String,
}

pub type Covariance = [f64; 9];

#[derive(Debug, Default, Deserialize)]
pub struct Imu {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: Covariance,
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: Covariance,
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: Covariance,
}

pub fn create_imu_messages(path: &PathBuf) -> Result<HashMap<String, Vec<Imu>>> {
    let fd = fs::File::open(path)?;
    let mapped = unsafe { Mmap::map(&fd) }?;

    let mut messages = HashMap::new();

    for message in mcap::MessageStream::new(&mapped)? {
        let message = message?;
        let topic = message.channel.topic.clone();

        let imu: Imu = cdr::deserialize(&message.data)?;

        let topic_messages = messages.entry(topic).or_insert(vec![]);
        topic_messages.push(imu);
    }

    Ok(messages)
}
