from setuptools import setup, find_packages

package_name = 'mecharm_skills_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('scripts'),
    package_dir={'': 'scripts'},
    install_requires=['setuptools', 'paho-mqtt', 'pymycobot'],
    entry_points={
        'console_scripts': [
            'mqtt_client = mqtt_client:main',
            'main_node   = main_node:main',
            'fixed_move_to_object = mecharm_skills.skills.fixed_move_to_object:main',
        ],
    },
)
