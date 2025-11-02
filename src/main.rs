use std::{path::PathBuf, process, fs};

use clap::Parser;
use log::*;
use anyhow::{Context, Result};
use memmap2::Mmap;

#[derive(Parser, Debug)]
struct Args {
    bag_path: PathBuf,
}

fn map_mcap(path: &PathBuf) -> Result<Mmap> {
    let fd = fs::File::open(path).context("Could not open file")?;
    unsafe { Mmap::map(&fd) }.context("Couldn't map MCAP file")
}

fn run() -> Result<()> {
    let args = Args::parse();

    let _mapped = map_mcap(&args.bag_path)?;

    Ok(())
}

fn main() {
    run().unwrap_or_else(|e| {
        error!("{e:?}");
        process::exit(1);
    });
}
