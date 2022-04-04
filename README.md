# Image Data Publisher Package

A ROS2 package for publishing common image datases 

## Supported Datasets

The following datasets are currently supported: 
- EuroC Mav Dataset

## Usage

```
ros2 run data_publisher data_publisher_node  --ros-args -p dataset_path:=<dataset-path> -p period_ms:=<period>
```

## Published Topics

### EuRoC MAV Dataset
 - `/left/image_raw`
 - `/right/image_raw`

