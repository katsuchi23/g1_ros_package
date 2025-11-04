# Build livox_ros_driver2
cd src/driver/livox_ros_driver2
./build.sh humble

# Build fast_lio
cd ../../..
colcon build --packages-select fast_lio

# fast_lio_localization
colcon build --packages-select fast_lio_localization

# g1_controller & navigation
colcon build --symlink-install --packages-select g1_controller
colcon build --symlink-install --packages-select g1_navigation

# Make script executable
chmod +x src/navigation/g1_navigation/scripts/semantic_map_pc_client.py
chmod +x src/navigation/g1_navigation/scripts/test_connection.py
