from glob import glob
from setuptools import setup

package = 'target_bringup'
setup(
    name=package,
    version='0.1.0',
    packages=[package],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package]),
        ('share/' + package, ['package.xml']),
        ('share/' + package + '/launch', glob('launch/*.py')),
        ('share/' + package + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    description='통합: 전체 실행 launch·노드 파라미터·검증 도구',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        f'{n} = target_bringup.{n}_node:main' for n in ['input_test', 'gimbal_sim', 'search_test', 'interface_check']
    ]},
)
