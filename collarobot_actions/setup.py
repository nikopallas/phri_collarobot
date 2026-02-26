from setuptools import find_packages, setup

package_name = 'collarobot_actions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [package_name + '/positions.toml']),
        ('share/' + package_name, [package_name + '/gestures.toml']),
        ('share/' + package_name + '/launch', ['launch/collarobot_actions.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phri3',
    maintainer_email='phri3',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_place_node = collarobot_actions.pick_place_node:main',
            'record_position = collarobot_actions.utils.record_position:main',
            'goto_position = collarobot_actions.utils.goto_position:main',
            'goto_carriage_lift = collarobot_actions.utils.goto_carriage_lift:main',
            'test_gripper = collarobot_actions.utils.test_gripper:main',
            'gesture_node = collarobot_actions.gesture_node:main',
            'record_gesture = collarobot_actions.utils.record_gesture:main',
        ],
    },
)
