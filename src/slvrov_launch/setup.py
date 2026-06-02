from glob import glob

from setuptools import find_packages, setup

package_name = 'slvrov_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/two_joystick_launch.py', 'launch/test_launch.py']),
        ('share/' + package_name + '/rov_config/rovs',
            glob('../../rov_config/rovs/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iroh_ubuntu',
    maintainer_email='calebhof11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
