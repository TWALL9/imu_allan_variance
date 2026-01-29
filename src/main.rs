use std::{
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
    time::{Duration, SystemTime},
};

use anyhow::{Context, Result};
use clap::Parser;
use log::{LevelFilter, error, info};
use rayon::prelude::*;

mod calc;
mod config;
mod messages;

#[derive(Parser, Debug)]
struct Args {
    bag_path: PathBuf,
    #[arg(long, default_value = "./config.yaml")]
    config_file: PathBuf,
    #[arg(long, default_value = "./output")]
    output_path: String,
}

/// Extract the IMU messages from the set based on a starting time, offset into the set
/// and a duration
fn message_range(
    messages: &[messages::Imu],
    offset_s: f64,
    duration_s: Option<f64>,
) -> &[messages::Imu] {
    let first = messages.first().unwrap().ts;
    let start = first + Duration::from_secs_f64(offset_s);

    let end = match duration_s {
        Some(d) => start + Duration::from_secs_f64(d),
        None => messages.last().unwrap().ts,
    };

    let start_idx = messages.partition_point(|x| x.ts <= start);
    let end_idx = messages.partition_point(|x| x.ts <= end);

    &messages[start_idx..end_idx]
}

/// Outputs deviances to file, deliminated by a space
/// Creates any directories that need creating
fn write_to_file(
    output_path: &str,
    topic: &str,
    deviances: &[(f64, calc::Vec6)],
) -> std::io::Result<()> {
    let mut out_path = PathBuf::new();
    out_path.push(output_path);

    if !out_path.exists() {
        std::fs::create_dir_all(out_path.clone())?;
    }

    // Convert a ROS topic into a usable filename
    // Topics have a leading / to remove, and replace all namespacing with _
    let topic_as_path = topic.replacen("/", "", 1).replace("/", "_");
    out_path.push(topic_as_path);
    out_path.set_extension("csv");

    log::warn!("output path: {:?}", out_path);
    let file = File::create(out_path)?;
    let mut stream = BufWriter::new(file);

    for (tau, avar) in deviances {
        writeln!(
            stream,
            "{:.19} {:.7} {:.7} {:.7} {:.7} {:.7} {:.7}",
            tau, avar[0], avar[1], avar[2], avar[3], avar[4], avar[5],
        )?;
    }
    stream.flush()
}

fn main() -> Result<()> {
    env_logger::Builder::new()
        .target(env_logger::Target::Stdout)
        .filter(None, LevelFilter::Info)
        .init();

    let start = SystemTime::now();

    let args = Args::parse();

    info!("Bag Path = {:?}", args.bag_path);
    info!("Config File = {:?}", args.config_file);
    info!("Output folder = {:?}", args.output_path);

    let config = config::load_config(&args.config_file).context("Could not load config")?;

    let messages = messages::create_imu_messages(&args.bag_path)?;

    let mut num_messages = 0;
    let mut num_topics = 0;
    for (_, m) in messages.iter() {
        num_messages += m.len();
        num_topics += 1;
    }
    info!(
        "Finished buffering data: {} measurements across {} topics",
        num_messages, num_topics
    );

    let mut missing_topics = vec![];

    for topic_config in config {
        if messages.contains_key(&topic_config.imu_topic) {
            info!("Topic {:?} found in the bag!", topic_config.imu_topic);
            info!("Configuration...{:?}", topic_config);

            let imu_data = &messages[&topic_config.imu_topic];
            if imu_data.is_empty() {
                error!("No messages for topic {}", topic_config.imu_topic);
                continue;
            }

            let imu_selection = message_range(
                imu_data,
                topic_config.sequence_offset,
                topic_config.sequence_duration,
            );

            log::info!(
                "Total measurements: {}, {} in target window",
                imu_data.len(),
                imu_selection.len()
            );

            // I do not know why, but the ROS Allan variance package saves deviances
            // instead of variances. It even calls them variances.
            let mut deviances: Vec<(f64, calc::Vec6)> = (1..10000)
                .into_par_iter()
                .map(|p| {
                    // Sampling periods from 0.1 to 1000s
                    let tau = p as f64 * 0.1;
                    let cluster_size = (tau * topic_config.measure_rate) as usize;
                    // TODO: overlapping averages
                    match calc::averages_non_overlapping(imu_selection, cluster_size) {
                        Ok(averages) => {
                            info!(
                                "Computed {} averages for sampling period {:.3?}",
                                averages.len(),
                                tau
                            );
                            let avar = calc::allan_variance(&averages);
                            let adev = avar.map(|x| x.sqrt());
                            info!("Allan variance for tau {:.3?} = {:?}", tau, avar);
                            info!("Allan deviance for tau {:3?} = {:?}", tau, adev);
                            (tau, adev)
                        }
                        Err(e) => {
                            error!("Error calculating Allan variance for {:.3?}: {:?}", tau, e);
                            // In order to let `par_iter` do its thing while allowing for error prints
                            // just put some marked data into the resulting Vec that can be removed later.
                            (f64::NAN, calc::Vec6::default())
                        }
                    }
                })
                .collect();

            // Remove any unusable deviances...deviant deviances?
            deviances.retain(|(tau, _)| !tau.is_nan());

            write_to_file(&args.output_path, &topic_config.imu_topic, &deviances)?;
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
