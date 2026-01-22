use std::{collections::HashMap, fs, path::PathBuf};

use anyhow::{Context, Result};
use clap::Parser;
use log::{error, info};

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
    let args = Args::parse();

    let config = config::load_config(&args.config_path).context("Could not load config")?;

    let messages = messages::create_imu_messages(&args.bag_path)?;

    for topic_config in config {
        if messages.contains_key(&topic_config.imu_topic) {
            info!(
                "Starting Allan variance computation for {}",
                topic_config.imu_topic
            );
        } else {
            error!("Topic {} not in available topics", topic_config.imu_topic);
        }
    }

    Ok(())
}
