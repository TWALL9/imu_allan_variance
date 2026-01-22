use std::{fs, path::PathBuf};

use anyhow::Result;
use serde::Deserialize;

fn default_imu_rate() -> f64 {
    100.0
}

#[derive(Debug, Deserialize)]
pub struct TopicConfig {
    pub imu_topic: String,
    #[serde(default = "default_imu_rate")]
    pub imu_rate: f64,
    pub measure_rate: f64,
    pub sequence_duration: f64,
    pub sequence_offset: Option<f64>,
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

pub fn load_config(path: &PathBuf) -> Result<Config> {
    let contents = fs::read_to_string(path)?;
    let config = serde_yaml_ng::from_str(&contents)?;

    Ok(config)
}
