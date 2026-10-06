from setuptools import find_packages, setup

package_name = 'xmen_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
    description='RealSense color and aligned depth image publisher',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'realsense_node = xmen_vision.realsense_node:main',
        ],
    },
)
