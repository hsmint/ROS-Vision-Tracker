from glob import glob

from setuptools import find_packages, setup

package_name = 'xmen_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/data', glob('data/*.png')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsmint',
    maintainer_email='hsmint.hong@gmail.com',
    description='Depth-validated target perception and pan/tilt tracking',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tracker_node = xmen_tracker.tracker_node:main',
            'preview_node = xmen_tracker.preview_node:main',
            'rviz_node = xmen_tracker.rviz_node:main',
            'tuning = xmen_tracker.tuning:main',
            'evaluate = xmen_tracker.evaluate:main',
        ],
    },
)
