from setuptools import setup

package_name = 'ros2_web_launcher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mat',
    maintainer_email='none@none',
    description='ROS2 web launcher node for managing and streaming launch files',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ros_launcher_node = ros2_web_launcher.ros_launcher_node:main'
        ],
    },
)
