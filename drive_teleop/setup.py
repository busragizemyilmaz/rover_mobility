from setuptools import setup

package_name = 'drive_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Teleoperation package for rover tank drive control.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tank_drive_joystick = drive_teleop.tank_drive_joystick:main',
        ],
    },
)

