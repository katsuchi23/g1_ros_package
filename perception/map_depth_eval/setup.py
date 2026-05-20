from setuptools import find_packages, setup


package_name = "map_depth_eval"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/map_depth_eval.launch.py",
                "launch/tabletop_depth_enhancer.launch.py",
                "launch/live_rgbd_pointcloud.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rey",
    maintainer_email="reynaldywidjaja3@gmail.com",
    description="Project a static 3D map into a camera depth image and backproject it.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_cloud_publisher = map_depth_eval.map_cloud_publisher:main",
            "depth_projection_evaluator = map_depth_eval.depth_projection_evaluator:main",
            "tabletop_depth_enhancer = map_depth_eval.tabletop_depth_enhancer:main",
            "live_rgbd_pointcloud = map_depth_eval.live_rgbd_pointcloud:main",
        ],
    },
)
