use std::{
    path::PathBuf,
    time::{Duration, SystemTime},
};

use anyhow::{Context, Result};
use clap::Parser;
use log::{error, info};
use rayon::prelude::*;

mod avar_calc;
mod config;
mod messages;

#[derive(Parser, Debug)]
struct Args {
    bag_path: PathBuf,
    #[arg(long, default_value = "./config.yaml")]
    config_path: PathBuf,
    #[arg(long, default_value = "./output")]
    output_path: String,
}

fn message_range<'a>(
    messages: &'a [messages::Imu],
    duration: Option<f64>,
    offset: f64,
) -> &'a [messages::Imu] {
    let first = messages.first().unwrap().ts;
    let start = first + Duration::from_millis(offset as u64);

    let end = match duration {
        Some(d) => start + Duration::from_millis(d as u64),
        None => messages.last().unwrap().ts,
    };

    avar_calc::range(messages, start, end)
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

            let imu_data = &messages[&topic_config.imu_topic];
            if imu_data.is_empty() {
                error!("No messages for topic {}", topic_config.imu_topic);
            }

            let imu_selection = message_range(
                &imu_data,
                topic_config.sequence_duration,
                topic_config.sequence_offset,
            );

            let mut variances: Vec<(f64, avar_calc::Vec6)> = (1..10000)
                .into_par_iter()
                .map(
                    |p| match avar_calc::avar_calc(imu_selection, topic_config.measure_rate, p) {
                        Ok(avar) => avar,
                        Err(e) => {
                            error!("{:?}", e);
                            (0.0, avar_calc::Vec6::default())
                        }
                    },
                )
                .collect();

            variances.retain(|(tau, _)| *tau > 0.0);

            // let variance_calc = avar_calc::VarianceCalculator::new(
            //     topic_config.measure_rate,
            //     topic_config.sequence_duration,
            //     topic_config.sequence_offset,
            // );

            // variance_calc.run(&messages[&topic_config.imu_topic], 1, 10000)?;

            let _out_path = stringify!(args.output_path + "/" + &topic_config.imu_topic + ".csv");
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
