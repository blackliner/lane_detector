from setuptools import setup, find_packages

package_name = "lane_detector"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    py_modules=[],
    zip_safe=True,
    install_requires=["setuptools", "opencv-python", "natsort"],
    author="Florian Berchtold",
    maintainer="Florian Berchtold",
    keywords=["ROS2"],
    description="Basic lane detector",
    license="Apache License, Version 2.0",
    test_suite="test",
    entry_points={"console_scripts": ["lane_detector = lane_detector.lane_detector:main",],},
)
