//! Parsing for the original ROS yaml file.
//! In order to support multiple topics, the original yaml file must be modified to encapsulate
//! parameters with the topic they represent. See the sample config.yaml for example.

use std::{fs, path::PathBuf};

use anyhow::Result;
use serde::Deserialize;

fn default_imu_rate() -> f64 {
    100.0
}

fn default_sequence_offset() -> f64 {
    0.0
}

/// Serde-compatible representation of the configuration in each topic
/// Unused members are holdovers from the ROS version (where they may not have been used either)
#[derive(Debug, Deserialize)]
pub struct TopicConfig {
    pub imu_topic: String,
    /// In original code: imu_rate was only used to calculate imu_skip_, which wasn't used
    #[serde(default = "default_imu_rate")]
    pub _imu_rate: f64,
    /// Measure rate aka 1 / sampling time
    pub measure_rate: f64,
    /// Set to None for entire bag
    pub sequence_duration: Option<f64>,
    #[serde(default = "default_sequence_offset")]
    pub sequence_offset: f64,
}

#[derive(Debug, Deserialize)]
pub struct Config(Vec<TopicConfig>);

impl IntoIterator for Config {
    type Item = TopicConfig;
    type IntoIter = std::vec::IntoIter<TopicConfig>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

/// Parse the top-level config
pub fn load_config(path: &PathBuf) -> Result<Config> {
    let contents = fs::read_to_string(path)?;
    let config = serde_yaml_ng::from_str(&contents)?;

    Ok(config)
}
