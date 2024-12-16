source dds.bash
source install/setup.bash
ros2 launch power_consumption_monitor power_consumption_monitor.launch.xml period:=1.0 method:=perf

