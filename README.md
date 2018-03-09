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

Publishes PointCloud2. Subscribes to espros_distance/image_raw


## Subscribed topics

### Configuration

espros_param

Published topics cannot be enabled/disabled via this method; they can only be specified at driver launch.  Output validation and orientation can be configured via this method.

To publish from the command line: rostopic pub --once espros_param diagnostic_msgs/KeyValue   '{"key" : "-parameter name-", "value" : "-integer-"}'


## Published topics

### Distance data

espros_distance/image_raw

espros_distance/camera_info


### False color distance image

espros_color_distance/image_raw

expros_color_distance/camera_info


### Grayscale data

espros_grayscale/image_raw

espros_grayscale/camera_info


### Interleaved destance and amplitude data

espros_interleave/image_raw

espros_interleave/camera_info


### Amplitude data

espros_amplitude/image_raw

espros_amplitude/camera_info


### Point cloud

espros_distance/points
