# espros-driver

QT - ROS requirements:

sudo apt-get install ros-kinetic-qt-create

sudo apt-get install ros-kinetic-qt-build

sudo apt-get install ros-kinetic-qt-ros

sudo apt-get install ros-kinetic-qt-gui

sudo apt-get install ros-kinetic-qt-gui-app

sudo apt-get install ros-kinetic-qt-gui-core

sudo apt-get install ros-kinetic-qt-gui-cpp


Usage - distance image: rosrun espros_nogui espros_nogui _espros_data:=0

Usage - amplitude image: rosrun espros_nogui espros_nogui _espros_data:=1

Defaults to distance; retains last param.

Publishes to topics espros_distnace, espros_amplitude.  Image format MONO16.

Publishes combined distance and amplitude to topic espros_amplitude_distance; image format value is 'ESPROS32'.

Device configuration exposed in espros_nogui/config.launch.  
