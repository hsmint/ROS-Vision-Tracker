from glob import glob
from setuptools import setup

package = 'target_control'
setup(
    name=package,
    version='0.1.0',
    packages=[package],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package]),
        ('share/' + package, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    description='제어: /target → 추적 명령(tracking_controller), 하드웨어 출력(motor_driver)',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'controller = target_control.controller_node:main',
        'motor_driver = target_control.motor_driver_node:main',
    ]},
)
