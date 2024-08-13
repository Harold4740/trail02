from setuptools import setup, find_packages

package_name = 'kuka_arm_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        ('lib/' + package_name,
            ['src/gazebo_camera_node.py' , 'src/cam_coord.py' , 'src/pick_and_place.py']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='Hareesh',
    maintainer_email='hareesh@gmail.com',
    description='Pick and place object detection for a KUKA arm using ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_camera_node = kuka_arm_pkg.gazebo_camera_node:main',
            'coordinate_subscriber = kuka_arm_pkg.cam_coord:main',
            'robot_arm_mover = kuka_arm_pkg.pick_and_place:main',
        ],
    },
)

