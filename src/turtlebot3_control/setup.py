from setuptools import setup

package_name = 'turtlebot3_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jbeason',
    maintainer_email='jordan.beason@westpoint.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_test = turtlebot3_control.teleop_test:main',
            'local_controller = turtlebot3_control.local_controller:main',
        ],
    },
)
