echo $CWD
source install/setup.bash
./scripts/setenv.bash
sudo ln -s /opt/ros/humble/lib/librcl_yaml_param_parser.so /usr/lib/librcl_yaml_param_parser.so
sudo ln -s /opt/ros/humble/lib/librcl_logging_interface.so /usr/lib/librcl_logging_interface.so

