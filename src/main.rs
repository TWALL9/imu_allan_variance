use std::{path::PathBuf, time::SystemTime};

use anyhow::{Context, Result};
use clap::Parser;
use log::{error, info};

mod avar_calc;
mod config;
mod messages;

#[derive(Parser, Debug)]
struct Args {
    bag_path: PathBuf,
    #[arg(long, default_value = "./config.yaml")]
    config_path: PathBuf,
    #[arg(long, default_value = "./output")]
    output_path: PathBuf,
}

fn main() -> Result<()> {
    let start = SystemTime::now();

    let args = Args::parse();

    let config = config::load_config(&args.config_path).context("Could not load config")?;

    let messages = messages::create_imu_messages(&args.bag_path)?;

    let mut missing_topics = vec![];

    for topic_config in config {
        if messages.contains_key(&topic_config.imu_topic) {
            info!(
                "Starting Allan variance computation for {}",
                topic_config.imu_topic
            );
        } else {
            error!(
                "Topic {} not in available topics, skipping!",
                topic_config.imu_topic
            );
            missing_topics.push(topic_config.imu_topic);
        }
    }

    let end = SystemTime::now();

    info!("Processing done, took {:?}", end.duration_since(start));

    if !missing_topics.is_empty() {
        error!(
            "Some topics were skipped, could not find {:?} in {:?}",
            missing_topics,
            messages.keys()
        );
    }

    Ok(())
}
