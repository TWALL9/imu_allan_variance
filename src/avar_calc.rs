use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::Result;

use super::messages;

#[derive(Debug)]
pub struct VarianceCalculator {
    imu_rate: f64,
    measure_rate: f64,
    sequence_duration: Option<f64>,
    sequence_offset: f64,
    out_path: String,
}

impl VarianceCalculator {
    pub fn new(
        imu_rate: f64,
        measure_rate: f64,
        sequence_duration: Option<f64>,
        sequence_offset: f64,
        out_path: &str,
    ) -> Self {
        Self {
            imu_rate,
            measure_rate,
            sequence_duration,
            sequence_offset,
            out_path: out_path.to_string(),
        }
    }

    fn range<'a>(
        &self,
        messages: &'a [messages::Imu],
        start: SystemTime,
        end: SystemTime,
    ) -> &'a [messages::Imu] {
        let start = messages.partition_point(|x| x.header.ts.to_system_time() <= start);
        let end = messages.partition_point(|x| x.header.ts.to_system_time() <= end);

        &messages[start..end]
    }

    pub fn run(&mut self, messages: &Vec<messages::Imu>) -> Result<()> {
        if messages.is_empty() {
            return Err(anyhow::anyhow!("Empty IMU messages!"));
        }

        let first_timestamp = if let Some(first_message) = messages.iter().next() {
            first_message.header.ts
        } else {
            messages::Timestamp::default()
        };

        let first_timestamp = first_timestamp.to_system_time();
        let offset_timestamp = first_timestamp + Duration::new(self.sequence_offset as u64, 0);

        let end_timestamp = match self.sequence_duration {
            Some(ts) => offset_timestamp + Duration::new(ts as u64, 0),
            None => {
                if let Some(last_message) = messages.last() {
                    last_message.header.ts.to_system_time()
                } else {
                    panic!("Somehow no last message in this vector?");
                }
            }
        };

        let range = self.range(messages, offset_timestamp, end_timestamp);

        self.calc_variance(range)
    }

    fn calc_variance<'a>(&mut self, messages: &'a [messages::Imu]) -> Result<()> {
        Ok(())
    }
}
