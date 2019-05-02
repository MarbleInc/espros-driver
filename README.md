## espros-driver

## Usage

### Command line

espros_data parameter over rides the show_* parameters

### Launch files

##### espros_nogui.launch

show_* parameters specify which topics to publish.  show_grayscale disables all other topics.

confidence_bits zeros any pixels flagged as bad data

orient_* parameters mirror the output on the indicated axis



##### espros_xyz.launch

Publishes PointCloud2 to espros/distance/points. Subscribes to espros/distance/image_raw


## Subscribed topics

### Configuration

espros/param

Published topics cannot be enabled/disabled via this method; they can only be specified at driver launch.  Output validation and orientation can be configured via this method.

To publish from the command line: rostopic pub --once espros_param diagnostic_msgs/KeyValue   '{"key" : "-parameter name-", "value" : "-integer-"}'


## Published topics

### Distance data

espros/distance/image_raw

espros/distance/camera_info


### False color distance image

espros/color_distance/image_raw

expros/color_distance/camera_info


### Grayscale data

espros/grayscale/image_raw

espros/grayscale/camera_info


### Interleaved destance and amplitude data

espros/interleave/image_raw

espros/interleave/camera_info


### Amplitude data

espros/amplitude/image_raw

espros/amplitude/camera_info


### Point cloud

espros/distance/points
