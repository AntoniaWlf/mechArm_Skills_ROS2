from setuptools import setup, find_packages

package_name = 'mecharm_skills_ros2'  # muss exakt mit <name> in package.xml übereinstimmen

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('scripts'),     # durchsucht scripts/ nach Packages
    package_dir={'': 'scripts'},           # maps package root → scripts/

    install_requires=[
        'setuptools',
        'paho-mqtt',
        'pymycobot',
    ],

    author='Antonia Wolf',
    author_email='wolf_antonia@gmx.de',
    description='ROS2-Package zur Orchestrierung von MechArm-Skills und MQTT-Client',
    license='MIT',

    entry_points={
        'console_scripts': [
            'mqtt_client    = mqtt_client:main',
            'main_node      = main_node:main',
            'fixed_move_to_object = skills.fixed_move_to_object:main'
        ],
    },
)