# espros-driver

QT - ROS requirements:

sudo apt-get install ros-kinetic-qt-create

sudo apt-get install ros-kinetic-qt-build

sudo apt-get install ros-kinetic-qt-ros

sudo apt-get install ros-kinetic-qt-gui

sudo apt-get install ros-kinetic-qt-gui-app

sudo apt-get install ros-kinetic-qt-gui-core

sudo apt-get install ros-kinetic-qt-gui-cpp

If you get a build error like:

usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN"

wrap line 50 in '#ifndef Q_MOC_RUN ... #endif', as well as the closing '}' near the bottom of the file. It may be indicated by '// namespace impl'.
