from setuptools import find_packages, setup

package_name = 'tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/tracker.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='th3kit3',
    maintainer_email='tambuprecious60@gmail.com',
    description='Object tracking for the XRP stereo rig; publishes /cmd_vel.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = tracker.camera_node:main',
            'effects_node = tracker.effects_node:main',
            'tracker_node = tracker.tracker_node:main',
        ],
    },
)
