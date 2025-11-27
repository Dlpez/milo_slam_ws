from setuptools import setup

package_name = 'arduino_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fots',
    maintainer_email='fots@example.com',
    description='Nodo ROS2 para odometría Arduino',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arduino_odometry = arduino_odometry.arduino_odometry:main'
        ],
    },
)
