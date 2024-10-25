#colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=Off -DCMAKE_CXX_FLAGS="-w"
#colcon build --cmake-clean-cache --symlink-install --cmake-args -DBUILD_TESTING=Off -DCMAKE_CXX_FLAGS="-w"
colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w"
