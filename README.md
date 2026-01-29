# imu_allan_variance

Parses through a ROS2 bag to calculate the Allan variance of IMU messages.

This is a standalone, non-ROS application that uses ROS data to generate data for usage in [Kalibr](https://github.com/ethz-asl/kalibr).

The only component of the ROS ecosystem that this package requires is [mcap](https://github.com/foxglove/mcap), which is external to ROS anyway.

## Moving from allan_variance_ros2
There are several forks of `allan_variance_ros2`:

- [Gautham-Ramkumar03](https://github.com/Gautham-Ramkumar03/allan_variance_ros2), which is a fork of AutoLiv
- [AutoLiv Research](https://github.com/Autoliv-Research/allan_variance_ros2), which is a fork of ori-drs
- [ori-drs](https://github.com/ori-drs/allan_variance_ros)

This application attempts to be a drop-in replacement for these packages, outputting data in the same format that they do, but some configuration data must be modified in order for this application to work.

The primary modification is that the `config.yaml` file used in `allan_variance_ros2` must be modified to be a yaml list, as this application supports calculating the Allan variance over multiple topics.

```yaml
imu_topic: "/imu_broadcaster/imu"
imu_rate: 100
measure_rate: 100 # Rate to which imu data is subsampled
sequence_duration: 3600 # duration of the sequence (in seconds)
sequence_offset: 0 # sequence start within the recorded data (in seconds)

```
needs to become
```yaml
- imu_topic: "/imu_broadcaster/imu"
  imu_rate: 100
  measure_rate: 100 # Rate to which imu data is subsampled
  sequence_duration: 3600 # duration of the sequence (in seconds)
  sequence_offset: 0 # sequence start within the recorded data (in seconds)
# and other topics go below...
- imu_topic: "/some/other/topic"
  imu_rate: 10
  measure_rate: 10
  sequence_duration: 60
  sequence_offset: 10
```

## Running
`imu_allan_variance` Requires a single parameter: the location of the .mcap file to read, and optional arguments for the following:

- `--config-file` config file location
- `--output-path` output file directory (will be created if it doesn't exist)

```bash
cargo run -- /path/to/your/bag/file.mcap --config-file /path/to/your/config_file.yaml --output-path /path/to/output/directory
```

Depending on the amount of topics and IMU data, calculation can be time-consuming. If you are finding that it is taking too long, run with the `--release` flag.