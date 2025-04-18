from setuptools import setup

package_name = 'can_teleop_odrive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    maintainer='you',
    maintainer_email='you@example.com',
    description='Drive ODrive & PCA9685 from /joint_states',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_to_can = can_teleop_odrive.joint_to_can:main',
        ],
    },
)
