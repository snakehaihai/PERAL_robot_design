cd ~/haloball
colcon build --packages-select wheels_velocity_msgs --symlink-install
. ./install/setup.bash
colcon build --symlink-install
. ./install/setup.bash
