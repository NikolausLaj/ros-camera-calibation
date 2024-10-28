from setuptools import find_packages, setup

package_name = 'camera_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_calibration_launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='nikolaus.lajtai@gmx.at',
    description='ROS 2 Camera Calibration Node using OpenCV',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_calibration = camera_calibration.camera_calibration:main',
        ],
    },
)
