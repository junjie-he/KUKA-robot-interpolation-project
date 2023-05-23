This package can only be launched in the big robot hall and connect to the KIA_5GHZ WIFI

Here are the commands for launch the package in terminal:

At workspace run following commands:

colcon build --packages-select waam_interpolation

source install/setup.bash; ros2 launch waam_interpolation kuka_launch,py
