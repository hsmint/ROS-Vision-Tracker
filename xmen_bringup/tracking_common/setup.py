from glob import glob
from setuptools import setup

package = 'tracking_common'
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
    description='토픽·타입·QoS·상태 값 단일 정의(인지·제어·시험·보고서 공용)',
    license='Apache-2.0',
    entry_points={'console_scripts': [
    ]},
)
