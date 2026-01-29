//! Read out contents of an mcap file and extract IMU information from it
//! The mcap file (ros2 bag) contains serialized sensor_msgs/msg/Imu messages, which
//! must be deserialized in their entirety. We're only interested in some portions of the
//! message, which are extracted into a new struct

use std::{
    collections::HashMap,
    fs,
    path::PathBuf,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use anyhow::Result;
use log::info;
use memmap2::Mmap;
use serde::Deserialize;

/// Representation of ROS2 geometry_msgs/msg/Vector3
#[derive(Debug, Default, Clone, Copy, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Representation of ROS2 geometry_msgs/msg/Quaternion
#[derive(Debug, Default, Clone, Copy, Deserialize)]
struct Quaternion {
    pub _x: f64,
    pub _y: f64,
    pub _z: f64,
    pub _w: f64,
}

/// Representation of ROS2 builtin_interfaces/Time
#[derive(Debug, Default, Clone, Copy, Deserialize)]
pub struct Timestamp {
    pub sec: i32,
    pub nanosec: u32,
}

impl Timestamp {
    pub fn to_system_time(self) -> SystemTime {
        UNIX_EPOCH + Duration::new(self.sec as u64, self.nanosec)
    }
}

/// Representation of ROS2 std_msgs/msg/Header
#[derive(Debug, Default, Deserialize)]
struct Header {
    pub ts: Timestamp,
    pub _frame_id: String,
}

/// Representation of covariance fields in sensor_msgs/msg/Imu
type Covariance = [f64; 9];

/// Representation of ROS2 sensor_msgs/msg/Imu
/// Used internally for deserialization and conversion to `messages::Imu`
#[derive(Debug, Default, Deserialize)]
struct ImuInternal {
    pub header: Header,
    pub _orientation: Quaternion,
    pub _orientation_covariance: Covariance,
    pub angular_velocity: Vector3,
    pub _angular_velocity_covariance: Covariance,
    pub linear_acceleration: Vector3,
    pub _linear_acceleration_covariance: Covariance,
}

impl From<ImuInternal> for Imu {
    fn from(val: ImuInternal) -> Self {
        Imu {
            ts: val.header.ts.to_system_time(),
            angular_velocity: val.angular_velocity,
            linear_acceleration: val.linear_acceleration,
        }
    }
}

/// IMU data used for calculating Allan variance
#[derive(Debug)]
pub struct Imu {
    pub ts: SystemTime,
    pub angular_velocity: Vector3,
    pub linear_acceleration: Vector3,
}

impl PartialEq for Imu {
    fn eq(&self, other: &Self) -> bool {
        self.ts == other.ts
    }
}

impl Eq for Imu {}

impl PartialOrd for Imu {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Imu {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.ts.cmp(&other.ts)
    }
}

/// Read out the data from an mcap file, parse IMU data, and store it in a map indexable by topic
pub fn create_imu_messages(path: &PathBuf) -> Result<HashMap<String, Vec<Imu>>> {
    let fd = fs::File::open(path)?;
    let mapped = unsafe { Mmap::map(&fd) }?;

    let mut imu_messages = HashMap::new();

    for message in mcap::MessageStream::new(&mapped)? {
        let message = message?;
        let topic = message.channel.topic.clone();
        let msg_type = match message.channel.schema.clone() {
            Some(schema) => schema.name.clone(),
            _ => "unknown".to_string(),
        };

        if msg_type == "sensor_msgs/msg/Imu" {
            let imu = cdr::deserialize::<ImuInternal>(&message.data)?;

            if !imu_messages.contains_key(&topic) {
                info!("Found new topic: {}", topic);
            }

            let topic_messages = imu_messages.entry(topic).or_insert(vec![]);
            topic_messages.push(imu.into());
        }
    }

    for (_, imu_msgs) in imu_messages.iter_mut() {
        imu_msgs.sort();
    }

    Ok(imu_messages)
}
