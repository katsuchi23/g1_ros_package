"""Launch robot_state_publisher using the existing g1_23dof.urdf.

This launch file reads the URDF file shipped in the package at
`g1_description/g1_23dof.urdf` and provides it as the `robot_description`
parameter to `robot_state_publisher`.

Usage: ros2 launch g1_navigation robot_description.launch.py

The URDF file is expected to live in the package share of `g1_navigation`
at `g1_description/g1_23dof.urdf`. If your package layout differs, adjust
the path below.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
	pkg_share = get_package_share_directory('g1_navigation')

	# Path to the existing URDF in this repository
	urdf_path = os.path.join(pkg_share, 'g1_description', 'g1_23dof.urdf')

	robot_description = ''
	try:
		with open(urdf_path, 'r') as f:
			robot_description = f.read()
	except FileNotFoundError:
		# Keep parameter empty; node will still start but without a description
		# which is helpful during development — user will see the error in the log.
		robot_description = ''

	rsp_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		parameters=[{
			'robot_description': robot_description,
			# 'use_sim_time': True  # enable if you want sim time
		}]
	)

	return LaunchDescription([rsp_node])

