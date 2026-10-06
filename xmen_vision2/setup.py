from glob import glob
from setuptools import setup

package = 'xmen_vision'
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
    description='인지: RealSense 영상 수신·확인, HSV·컨투어·뎁스 검출 → /target',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'camera_viewer = xmen_vision.camera_viewer_node:main',
        'detector = xmen_vision.detector_node:main',
    ]},
)
