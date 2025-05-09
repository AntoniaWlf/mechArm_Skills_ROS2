from setuptools import setup, find_packages

package_name = 'mecharm_skills'  # muss exakt mit <name> in package.xml übereinstimmen

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

    author='Dein Name',
    author_email='deine.email@example.com',
    description='ROS2-Package zur Orchestrierung von MechArm-Skills und MQTT-Client',
    license='MIT',

    entry_points={
        'console_scripts': [
            'mqtt_client    = mqtt_client:main',
            'main_node      = main_node:main',
            # falls du skill_server behältst:
            # 'skill_server   = Skill_server:main',
        ],
    },
)