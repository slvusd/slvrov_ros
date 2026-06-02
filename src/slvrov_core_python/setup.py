from glob import glob

from setuptools import find_packages, setup

package_name = 'slvrov_core_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web/docs',
            glob('slvrov_core_python/web/docs/*.md')),
        ('share/' + package_name + '/web/nodes',
            glob('slvrov_core_python/web/nodes/*.md')),
        ('share/' + package_name + '/web/routes',
            glob('slvrov_core_python/web/routes/*.md')),
        ('share/' + package_name + '/web/adapters',
            glob('slvrov_core_python/web/adapters/*.md')),
        ('share/' + package_name + '/web/static',
            glob('slvrov_core_python/web/static/*.md')),
        ('share/' + package_name + '/web/static/css',
            glob('slvrov_core_python/web/static/css/*.css')),
        ('share/' + package_name + '/web/static/js',
            glob('slvrov_core_python/web/static/js/*.js')),
        ('share/' + package_name + '/web/templates',
            glob('slvrov_core_python/web/templates/*.html')),
        ('share/' + package_name + '/science/docs',
            glob('slvrov_core_python/science/docs/*.md')),
        ('share/' + package_name + '/science/capture',
            glob('slvrov_core_python/science/capture/*.md')),
        ('share/' + package_name + '/science/data_logging',
            glob('slvrov_core_python/science/data_logging/*.md')),
        ('share/' + package_name + '/science/media',
            glob('slvrov_core_python/science/media/*.md')),
        ('share/' + package_name + '/mediamtx',
            glob('slvrov_core_python/mediamtx/*.md')),
    ],
    package_data={
        package_name: [
            'web/docs/*.md',
            'web/nodes/*.md',
            'web/routes/*.md',
            'web/adapters/*.md',
            'web/static/*.md',
            'web/static/css/*.css',
            'web/static/js/*.js',
            'web/templates/*.html',
            'science/docs/*.md',
            'science/capture/*.md',
            'science/data_logging/*.md',
            'science/media/*.md',
            'mediamtx/*.md',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LegionaryOfLogic',
    maintainer_email='calebhof11@gmail.com',
    description='Core SLVROV control nodes, Flask UI, MediaMTX support, and web adapters.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joystick_mapper=slvrov_core_python.joystick_mapper:main',
            'joystick_logic=slvrov_core_python.old_joystick_logic:main',
            'pca9685_node=slvrov_core_python.pca9685_node:main',
            'pin_mappings_client=slvrov_core_python.pin_mappings_client:main',
            'pin_mappings_server=slvrov_core_python.pin_mappings_server:main',
        ],
    },
)
