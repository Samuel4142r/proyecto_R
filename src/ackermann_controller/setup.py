from glob import glob

from setuptools import setup

package_name = 'ackermann_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipo Robot Ackermann',
    maintainer_email='team@example.com',
    description='Pure Pursuit y registro de datos para el robot Ackermann',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = ackermann_controller.pure_pursuit_node:main',
            'data_logger = ackermann_controller.data_logger_node:main',
        ],
    },
)
