from setuptools import setup
from glob import glob

package_name = 'zuuu_hal'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.launch.py")),
        ("share/" + package_name, glob("launch/*_launch.py")),
        ("share/" + package_name + "/config", glob("config/*.*")),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='remi',
    maintainer_email='remifabre1800@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_me = zuuu_hal.follow_me:main',
            'hal = zuuu_hal.zuuu_hal:main',
            'teleop_keyboard = zuuu_hal.zuuu_teleop_keyboard:main',
            'teleop_joy = zuuu_hal.zuuu_teleop_joy:main',
            'laser_filter = zuuu_hal.laser_filter:main',
            'speed_calibration = zuuu_hal.zuuu_speed_calibration:main',
            'set_speed_service_test = zuuu_hal.set_speed_service_test:main',
            'go_to_service_test = zuuu_hal.go_to_service_test:main',
        ],
    },
)
