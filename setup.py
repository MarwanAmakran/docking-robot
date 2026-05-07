from setuptools import find_packages, setup

package_name = 'docking_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='marwanama1109@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'aruco_detector = docking_robot.aruco_detector:main',
		'camera_publisher = docking_robot.camera_publisher:main',
		'docking_controller = docking_robot.docking_controller:main',
		'serial_bridge = docking_robot.serial_bridge:main',
		'battery_publisher = docking_robot.battery_publisher:main',
        ],
    },
)
