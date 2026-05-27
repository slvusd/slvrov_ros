from glob import glob

from setuptools import find_packages, setup


package_name = 'slvrov_web_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/docs',
            glob('slvrov_web_ui/docs/*.md')),
        ('share/' + package_name + '/nodes',
            glob('slvrov_web_ui/nodes/*.md')),
        ('share/' + package_name + '/routes',
            glob('slvrov_web_ui/routes/*.md')),
        ('share/' + package_name + '/ros_adapters',
            glob('slvrov_web_ui/ros_adapters/*.md')),
        ('share/' + package_name + '/static',
            glob('slvrov_web_ui/static/*.md')),
        ('share/' + package_name + '/static/css',
            glob('slvrov_web_ui/static/css/*.css')),
        ('share/' + package_name + '/static/js',
            glob('slvrov_web_ui/static/js/*.js')),
        ('share/' + package_name + '/templates',
            glob('slvrov_web_ui/templates/*.html')),
    ],
    package_data={
        package_name: [
            'docs/*.md',
            'nodes/README.md',
            'routes/README.md',
            'ros_adapters/README.md',
            'static/README.md',
            'static/css/*.css',
            'static/js/*.js',
            'templates/*.html',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LegionaryOfLogic',
    maintainer_email='calebhof11@gmail.com',
    description='Flask backend and browser UI skeleton for the SLVROV MVP.',
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
