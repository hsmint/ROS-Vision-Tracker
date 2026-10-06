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
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsmint',
    maintainer_email='hsmint.hong@gmail.com',
    description='Synchronized RealSense color and depth subscriber',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tracker_node = xmen_tracker.tracker_node:main',
            'rviz_node = xmen_tracker.rviz_node:main',
        ],
    },
)
