use std::{collections::HashMap, fs, path::PathBuf};

use anyhow::{Context, Result};
use clap::Parser;
use log::{error, info};
use memmap2::Mmap;

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

fn map_mcap(path: &PathBuf) -> Result<Mmap> {
    let fd = fs::File::open(path).context("Could not open file")?;
    unsafe { Mmap::map(&fd) }.context("Couldn't map MCAP file")
}

fn create_imu_messages(mapped: Mmap) -> Result<HashMap<String, Vec<messages::Imu>>> {
    let mut messages = HashMap::new();

    for message in
        mcap::MessageStream::new(&mapped).context("Could not convert to message stream")?
    {
        let message = message?;
        let topic = message.channel.topic.clone();

        let imu: messages::Imu =
            cdr::deserialize(&message.data).context("Could not parse mcap message into IMU")?;

        let topic_messages = messages.entry(topic).or_insert(vec![]);
        topic_messages.push(imu);
    }

    Ok(messages)
}

fn main() -> Result<()> {
    let args = Args::parse();

    let config = config::load_config(&args.config_path).context("Could not load config")?;

    let mapped = map_mcap(&args.bag_path)?;

    let messages = create_imu_messages(mapped)?;

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
