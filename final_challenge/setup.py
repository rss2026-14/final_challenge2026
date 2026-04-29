from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'final_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # UPDATE THIS LINE: Use * instead of *.py so it grabs your XML file
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='cheniseoharper@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'boating_executive = final_challenge.state_decider:main',
            'parking_meter_controller = final_challenge.parking_meter_controller:main',
            'person_controller = final_challenge.person_controller:main',
            'stop_sign_controller = final_challenge.stop_sign_controller:main',
            'traffic_light_controller = final_challenge.traffic_light_controller:main',
            'traffic_light_color_detector = final_challenge.traffic_light_color_detector:main',

        ],
    },
)
