from glob import glob

from setuptools import find_packages, setup


package_name = 'slvrov_science_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/docs',
            glob('slvrov_science_python/docs/*.md')),
        ('share/' + package_name + '/capture',
            glob('slvrov_science_python/capture/*.md')),
        ('share/' + package_name + '/data_logging',
            glob('slvrov_science_python/data_logging/*.md')),
        ('share/' + package_name + '/media',
            glob('slvrov_science_python/media/*.md')),
    ],
    package_data={
        package_name: [
            'docs/*.md',
            'capture/README.md',
            'data_logging/README.md',
            'media/README.md',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LegionaryOfLogic',
    maintainer_email='calebhof11@gmail.com',
    description='Science, camera, and data-collection skeleton for the SLVROV MVP.',
    license='TODO: License declaration',
    url='https://github.com/LegionaryOfLogic/slvrov_ros',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)
