use std::{
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
    time::{Duration, SystemTime},
};

use anyhow::{Context, Result};
use clap::Parser;
use log::{error, info};
use rayon::prelude::*;

mod calc;
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

fn message_range(
    messages: &[messages::Imu],
    duration: Option<f64>,
    offset: f64,
) -> &[messages::Imu] {
    let first = messages.first().unwrap().ts;
    let start = first + Duration::from_millis(offset as u64);

    let end = match duration {
        Some(d) => start + Duration::from_millis(d as u64),
        None => messages.last().unwrap().ts,
    };

    calc::range(messages, start, end)
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
                imu_data,
                topic_config.sequence_duration,
                topic_config.sequence_offset,
            );

            let mut variances: Vec<(f64, calc::Vec6)> = (1..10000)
                .into_par_iter()
                .map(|p| {
                    // Sampling periods from 0.1 to 1000s
                    let tau = p as f64 * 0.1;
                    let cluster_size = (tau * topic_config.measure_rate) as usize;
                    match calc::avar_non_overlapping(imu_selection, cluster_size) {
                        Ok(avar) => (tau, avar),
                        Err(e) => {
                            error!("{:?}", e);
                            (std::f64::NAN, calc::Vec6::default())
                        }
                    }
                })
                .collect();

            variances.retain(|(tau, _)| *tau != std::f64::NAN);

            // let variance_calc = calc::VarianceCalculator::new(
            //     topic_config.measure_rate,
            //     topic_config.sequence_duration,
            //     topic_config.sequence_offset,
            // );

            // variance_calc.run(&messages[&topic_config.imu_topic], 1, 10000)?;

            let out_path = stringify!(args.output_path + "/" + &topic_config.imu_topic + ".csv");
            let file = File::create(out_path)?;
            let mut stream = BufWriter::new(file);
            for (tau, avar) in variances {
                writeln!(
                    stream,
                    "{:.19} {:.7} {:.7} {:.7} {:.7} {:.7} {:.7}",
                    tau, avar[0], avar[1], avar[2], avar[3], avar[4], avar[5],
                )?;
            }
            stream.flush()?;
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
