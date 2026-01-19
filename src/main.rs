mod messages;

use std::{collections::HashMap, fs, path::PathBuf};

use anyhow::{Context, Result};
use clap::Parser;
use memmap2::Mmap;

#[derive(Parser, Debug)]
struct Args {
    bag_path: PathBuf,
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

        let m_v = messages.entry(topic).or_insert(vec![]);
        m_v.push(imu);
    }

    Ok(messages)
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mapped = map_mcap(&args.bag_path)?;

    let messages = create_imu_messages(mapped).context("Could not convert MCAP data")?;

    println!("{:?}", messages);

    Ok(())
}
