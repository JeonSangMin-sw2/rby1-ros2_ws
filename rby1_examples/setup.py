import os
from glob import glob
from setuptools import setup

package_name = 'rby1_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsm',
    maintainer_email='sangmin.jeon@rainbow-robotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'power_control_example = rby1_examples.power_control_example:main',
            'single_joint_example = rby1_examples.single_joint_example:main',
            'multi_joint_example = rby1_examples.multi_joint_example:main',
        ],
    },
)
